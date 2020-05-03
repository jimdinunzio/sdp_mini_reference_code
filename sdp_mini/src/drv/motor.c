/*
 * SlamTec Base Ref Design
 * Copyright 2009 - 2017 RoboPeak
 * Copyright 2013 - 2017 Shanghai SlamTec Co., Ltd.
 * http://www.slamtec.com
 * All rights reserved.
 */
/*
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "common/common.h"
#include "motor.h"
#include "beep.h"
#include "drv/timer.h"
#include "drv/exti.h"
#include "math.h"

#define WALKINGMOTOR_CNT        2
#define WALKINGMOTOR_L_ID       0
#define WALKINGMOTOR_R_ID       1

//#define CONFIG_MOTOR_PID_DEBUG

/**
 @brief motor driving port configures.
 */
static motor_cfg_t _motor_cfg[] = {
    {   /**< Left walking motor configure. */
        {MOTO_L_PWM_PORT, MOTO_L_PWM_PIN, MOTO_L_PWM, MOTO_L_PWM_CHN},
        {MOTO_L_DIR_PORT, MOTO_L_DIR_PIN, HIGH},
        {NULL, 0, LOW},
        {ENCODER_LA_PORT, ENCODER_LA_PIN, ENCODER_LA_EXTI},
        {ENCODER_LB_PORT, ENCODER_LB_PIN, LOW},
        ODOMETER_EST_PULSE_PER_METER,
    },

    {   /**< Right walking motor configure. */
        {MOTO_R_PWM_PORT, MOTO_R_PWM_PIN, MOTO_R_PWM, MOTO_R_PWM_CHN},
        {MOTO_R_DIR_PORT, MOTO_R_DIR_PIN, HIGH},
        {NULL, 0, LOW},
        {ENCODER_RA_PORT, ENCODER_RA_PIN, ENCODER_RA_EXTI},
        {ENCODER_RB_PORT, ENCODER_RB_PIN, LOW},
        ODOMETER_EST_PULSE_PER_METER,
    },
};

static _s32 _motorDeltaTicks[WALKINGMOTOR_CNT];           //Cumulative encoder value of walking motor
static _u32  _motorDistAccumulated[WALKINGMOTOR_CNT];     // Cumulative walking distance in mm.
static float _motorDistTailing[WALKINGMOTOR_CNT];         // The remaining value of the accumulated walking distance is less than 1mm for the next accumulation.

static _s32 _encoder1TicksDelta[WALKINGMOTOR_CNT];        //Encoder value in delta time
static _s32 _lastEncoderTicksDelta[WALKINGMOTOR_CNT];     //Encoder value in the last delta time
static float _lastOdometerSpeedAbs[WALKINGMOTOR_CNT];     //Speed ​​value in the last delta time

#if defined(CONFIG_MOTOR_ENCODER_NUM) && (CONFIG_MOTOR_ENCODER_NUM == 2)
static _s32 _encoder2TicksDelta[WALKINGMOTOR_CNT];               //detla时间内的编码器值
#endif

static _u8 _motorCtrlStates[WALKINGMOTOR_CNT];                  //行走电机方向
static _s32 _motorSpeedMm[WALKINGMOTOR_CNT];                    //行走电机速度

//static const float Kp = 8;                     //PID 比例因子
//static const float Ki = 1.6;                   //PID 积分因子
//static const float Kd = 0.0;                   //PID 微分因子

static const float Kp = 1.5;                     //PID 比例因子
static const float Ki = 0.6;                   //PID 积分因子
static const float Kd = 0.0;                   //PID 微分因子

// first 1.0, 0.4, 0.0
// second 1.25, 0.6, 0.0
// third 1.5, 0.8, 0.0
// fourth 1.5 0.6 0.0

static float speedLastErr[WALKINGMOTOR_CNT];
static float speedErri[WALKINGMOTOR_CNT];
static float speed_PWMOUT[WALKINGMOTOR_CNT];

// Stall sensor filter
static _u32 _stallFilterBlindModeTS[WALKINGMOTOR_CNT];
static _u8  _stallBitmap;
static _u32 _stallDetectorTS = 0;
static _u32 _stallDetectorFilter[WALKINGMOTOR_CNT] = {0, 0};

static void _init_motor_pwm(const pwm_port_t *pwm)
{
    TIM_TimeBaseInitTypeDef tim_base;
    TIM_OCInitTypeDef       tim_oc;

    if (pwm == NULL || pwm->port == NULL) {
        return ;
    }

    pinMode(pwm->port, pwm->pin, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(pwm->port,  pwm->pin, Bit_SET);

    TIM_CtrlPWMOutputs(pwm->tim, DISABLE);
    TIM_ARRPreloadConfig(pwm->tim, DISABLE);
    TIM_Cmd(pwm->tim, DISABLE);

    /* Initialize motor control pwm. */
    tim_base.TIM_Period = (CONFIG_MOTOR_PWM_PERIOD - 1);
    tim_base.TIM_Prescaler = 0;
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(pwm->tim, &tim_base);

    /* Motor PWM ouput channel configure. */
    tim_oc.TIM_OCMode      = TIM_OCMode_PWM2;
    tim_oc.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc.TIM_Pulse       = 0;
    tim_oc.TIM_OCPolarity  = TIM_OCPolarity_Low;

    tim_oc_init(pwm->tim, pwm->tim_ch, &tim_oc);
    tim_oc_preloadcfg(pwm->tim, pwm->tim_ch, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(pwm->tim, ENABLE);
    TIM_ARRPreloadConfig(pwm->tim, ENABLE);
    TIM_Cmd(pwm->tim, ENABLE);
}

/**
 @brief Set motor pwm duty.
 @param pwm  - motor pwm port.
 @param duty - motor pwm duty.
 */
static void motor_set_duty(const pwm_port_t *pwm, int duty)
{
    if (pwm == NULL) {
        return ;
    }

    duty = abs(duty);
    if (duty > CONFIG_MOTOR_PWM_PERIOD) {
        duty = CONFIG_MOTOR_PWM_PERIOD;
    }
    tim_set_compare(pwm->tim, pwm->tim_ch, duty);

    if (duty > 0) {
        pinMode(pwm->port, pwm->pin, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    }
}

/**
 @brief motor brake.
 @param id - motor id.
 @return none.

*/
static void _set_walkingmotor_brake(_u8 id, int duty)
{
    if (id >= WALKINGMOTOR_CNT) {
        return ;
    }
    /**< Motor driver use pwm + direction. */
    /* Disable duty */
    motor_set_duty(&_motor_cfg[id].pwm, 0);
    pinSet(_motor_cfg[id].dir.port, _motor_cfg[id].dir.pin, Bit_RESET);
    return ;
}

/**
 @brief Drive motor forward.
 @param id - motor id.
 @return none.

*/
static void _set_walkingmotor_forward(_u8 id, int duty)
{
    if (id >= WALKINGMOTOR_CNT) {
        return ;
    }

    /* Enable forward duty. */
    //DBG_OUT("set walkingmotor forward, id = %d duty = %d\r\n", id, duty);
    motor_set_duty(&_motor_cfg[id].pwm, duty);
    pinSet(_motor_cfg[id].dir.port, _motor_cfg[id].dir.pin, 
           id == WALKINGMOTOR_L_ID ? Bit_RESET : Bit_SET);
    return ;
}

/**
 @brief Drive motor backward.
 @param id - motor id.
 @return none.

*/
static void _set_walkingmotor_backward(_u8 id, int duty)
{
    if (id >= WALKINGMOTOR_CNT) {
        return ;
    }

    /* Enable backward duty. */
    //DBG_OUT("set walkingmotor backward, id = %d, duty = %d\r\n", id, duty);
    motor_set_duty(&_motor_cfg[id].pwm, duty);
    pinSet(_motor_cfg[id].dir.port, _motor_cfg[id].dir.pin, 
           id == WALKINGMOTOR_L_ID ? Bit_SET : Bit_RESET);
    return ;
}

/**
 @brief Release motor driver.
 @param id - motor id.
 @return none.

*/
static void _set_walkingmotor_release(_u8 id)
{
    const pwm_port_t *pwm;

    if (id >= WALKINGMOTOR_CNT) {
        return ;
    }

    /* pwm output high. */
    //DBG_OUT("set walkingmotor release\r\n");
    pwm = &_motor_cfg[id].pwm;
    pinMode(pwm->port, pwm->pin, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(pwm->port, pwm->pin, Bit_RESET);
    pinSet(_motor_cfg[id].dir.port, _motor_cfg[id].dir.pin, Bit_RESET);
    _delay_us(20);
    return ;
}

// 根据指令进行电机控制
static inline void _set_walkingmotor(_u8 id, _s32 duty, _s32 ctrl)
{
    if (id >= WALKINGMOTOR_CNT) {
        return ;
    }

    switch (ctrl) {
    case MOTOR_CTRL_STATE_RELEASE:
        _set_walkingmotor_release(id);
        break;
    case MOTOR_CTRL_STATE_BRAKE:
        _set_walkingmotor_brake(id, duty);
        break;
    case MOTOR_CTRL_STATE_FORWARD:
        _set_walkingmotor_forward(id, duty);
        break;
    case MOTOR_CTRL_STATE_BACKWARD:
        _set_walkingmotor_backward(id, duty);
        break;
    }
    _motorCtrlStates[id] = ctrl;
}

/**
 @brief initialize out port
 @param out_port output port
 @return none
*/
void _init_out_port(const out_port_t *out_port)
{
    pinMode(out_port->port, out_port->pin, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(out_port->port, out_port->pin, (BitAction)out_port->level);
}

/*
 * Left and right walking motor initialization function
 */
void init_walkingmotor(void)
{
    _u8 i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);

    /* Power on the motor */
    // this turns on "CURRENT_SET" on J21-9, needed to get motor drive power
    pinMode(MOTO_EN_PORT, MOTO_EN_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(MOTO_EN_PORT,  MOTO_EN_PIN, Bit_SET);

    for (i = 0; i < _countof(_motor_cfg); i++) {
        _init_motor_pwm(&_motor_cfg[i].pwm);
        _init_out_port(&_motor_cfg[i].dir);
        if (_motor_cfg[i].oc_mon.port != NULL) {
            pinMode(_motor_cfg[i].oc_mon.port, _motor_cfg[i].oc_mon.pin, GPIO_Mode_IN_FLOATING, GPIO_Speed_10MHz);
        }
    }

    _set_walkingmotor(WALKINGMOTOR_L_ID, 0, MOTOR_CTRL_STATE_RELEASE);
    _set_walkingmotor(WALKINGMOTOR_R_ID, 0, MOTOR_CTRL_STATE_RELEASE);
    memset(_motorSpeedMm, 0, sizeof(_motorSpeedMm));
    memset(speedLastErr, 0, sizeof(speedLastErr));
    memset(speedErri, 0, sizeof(speedErri));
    DBG_OUT("Walking motor initialized with pid(%d.%d, %d.%d, %d.%d)\r\n",
            (int)Kp, (int)((Kp-(int)Kp)*10),
            (int)Ki, (int)((Ki-(int)Ki)*10),
            (int)Kd, (int)((Kd-(int)Kd)*10));
}

//   ENCODER CODE  ******************************************************


/* Encoder interrupt handlers for normal encoders (many tick/revolution).
 * Interrupt on rising and falling edges of channel A and determine direction from chan B
 */
void encoder_LA_exti_cb(void)   
{
    static u8 encoderLALast = LOW;
    static s8 dirL = 1;
    u8 encoderLA = pinRead(ENCODER_LA_PORT, ENCODER_LA_PIN);
    if (encoderLALast == LOW && encoderLA == HIGH)
    {
      u8 encoderLB = pinRead(ENCODER_LB_PORT, ENCODER_LB_PIN);
      if (encoderLB == LOW && dirL == -1) 
        dirL = 1;
      else if (encoderLB == HIGH && dirL == 1)
        dirL = -1;
    }
    _encoder1TicksDelta[WALKINGMOTOR_L_ID] += dirL;
    encoderLALast = encoderLA;
}

void encoder_RA_exti_cb(void)
{
    static u8 encoderRALast = LOW;
    static s8 dirR = 1;
    u8 encoderRA = pinRead(ENCODER_RA_PORT, ENCODER_RA_PIN);
    if (encoderRALast == LOW && encoderRA == HIGH)
    {
      u8 encoderRB = pinRead(ENCODER_RB_PORT, ENCODER_RB_PIN);
      if (encoderRB == LOW && dirR == 1) 
        dirR = -1;
      else if (encoderRB == HIGH && dirR == -1)
        dirR = 1;
    }
    _encoder1TicksDelta[WALKINGMOTOR_R_ID] += dirR;
    encoderRALast = encoderRA;
}

/*
 * Set up interrupt hardware
 */
static void init_extix(void)
{
    _u8 i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    for (i = 0; i < _countof(_motor_cfg); i++) {
        pinMode(_motor_cfg[i].encoderA.port, _motor_cfg[i].encoderA.pin, GPIO_Mode_IPU, GPIO_Speed_50MHz);
        pinMode(_motor_cfg[i].encoderB.port, _motor_cfg[i].encoderB.pin, GPIO_Mode_IPU, GPIO_Speed_50MHz);
    }

    GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_L_ID].encoderA.port), _motor_cfg[WALKINGMOTOR_L_ID].encoderA.exti);
    GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_R_ID].encoderA.port), _motor_cfg[WALKINGMOTOR_R_ID].encoderA.exti);
    exti_reg_callback(_motor_cfg[WALKINGMOTOR_L_ID].encoderA.exti, EXTI_Trigger_Rising_Falling, encoder_LA_exti_cb); 
    exti_reg_callback(_motor_cfg[WALKINGMOTOR_R_ID].encoderA.exti, EXTI_Trigger_Rising_Falling, encoder_RA_exti_cb); 
}

/*
 * MOTOR ODOMETER INITIALIZATION
 */
void init_walkingmotor_odometer(void)
{
    memset(_motorDeltaTicks, 0, sizeof(_motorDeltaTicks));
    memset(_motorDistAccumulated, 0, sizeof(_motorDistAccumulated));
    memset(_motorDistTailing, 0, sizeof(_motorDistTailing));
    
    memset(_encoder1TicksDelta, 0, sizeof(_encoder1TicksDelta));
    memset(_encoder2TicksDelta, 0, sizeof(_encoder2TicksDelta));
    memset(_lastEncoderTicksDelta, 0, sizeof(_lastEncoderTicksDelta));
    memset(_lastOdometerSpeedAbs, 0, sizeof(_lastOdometerSpeedAbs));
    init_extix();
}
/*
 * Refresh mileage data function of walking motor
 */
static void _refresh_walkingmotor_odometer(_u32 durationMs)
{
    _u8 cnt;
    float dist_mm;
    // disable interrupt :
    _u32 irqSave = enter_critical_section();     //disable interrupt
    for (cnt = 0; cnt < WALKINGMOTOR_CNT; ++cnt) {//Get the encoder data during this time
        _lastEncoderTicksDelta[cnt] = _encoder1TicksDelta[cnt];
        _encoder1TicksDelta[cnt] = 0;
    }
    leave_critical_section(irqSave);

    // Calculate odometry data
    if (durationMs == 0) //just to avoid divide by zero error
        durationMs = 1;

    for (cnt = 0; cnt < WALKINGMOTOR_CNT; ++cnt) {
        //Calculate the speed during this period based on the encoder data 
        dist_mm = (float)_lastEncoderTicksDelta[cnt] * (1000.0f/_motor_cfg[cnt].speed_factor);
		
        _lastOdometerSpeedAbs[cnt] = fabsf(dist_mm * 1000.0f / durationMs);
#ifdef CONFIG_MOTOR_PID_DEBUG
        if (_lastEncoderTicksDelta[cnt] > 0) {
            DBG_OUT("encoder[%d] = %d\r\n", cnt, (int)_lastEncoderTicksDelta[cnt]);
        }
#endif

        dist_mm += _motorDistTailing[cnt];
        _motorDistAccumulated[cnt] += (_u32)fabsf(dist_mm);
        _motorDistTailing[cnt] = dist_mm - (_s32)dist_mm;

        _motorDeltaTicks[cnt] += _lastEncoderTicksDelta[cnt];
#ifdef CONFIG_MOTOR_PID_DEBUG
        if (cnt == 0 && getms() - lastTime > 250) {
          DBG_OUT("dist_mm = %d, _motorDeltaTicks = %d, _lastOdometerSpeedAbs = %d\r\n", 
                  (int)dist_mm, (int)_motorDeltaTicks[cnt], (int)_lastOdometerSpeedAbs[cnt]);
          lastTime = getms();
        }
#endif
    }

}
/*
 * 计算左行走电机累计里程函数
 * 单位：mm
 */
_u32 walkingmotor_cumulate_ldist_mm(void)
{
    return _motorDistAccumulated[WALKINGMOTOR_L_ID];
}
/*
 * 计算右行走电机累计里程函数
 * 单位：mm
 */
_u32 walkingmotor_cumulate_rdist_mm(void)
{
    return _motorDistAccumulated[WALKINGMOTOR_R_ID];
}

float walkingmotor_delta_ldist_mm_f(void)
{
    float delta_dist = _motorDeltaTicks[WALKINGMOTOR_L_ID];
    _motorDeltaTicks[WALKINGMOTOR_L_ID] = 0;
    return delta_dist * (1000.f / _motor_cfg[WALKINGMOTOR_L_ID].speed_factor);
}

float walkingmotor_delta_rdist_mm_f(void)
{
    float delta_dist = _motorDeltaTicks[WALKINGMOTOR_R_ID];
    _motorDeltaTicks[WALKINGMOTOR_R_ID] = 0;
    return delta_dist * (1000.f / _motor_cfg[WALKINGMOTOR_R_ID].speed_factor);
}

/*
 * 计算左行走电机当前速度函数
 * 单位：mm/s
 */
_u32 get_walkingmotor_lspeed_mm_q16(void)
{
    return (_u32)(_lastOdometerSpeedAbs[WALKINGMOTOR_L_ID] * 65536.0);
}
/*
 * 计算右行走电机当前速度函数
 * 单位：mm/s
 */
_u32 get_walkingmotor_rspeed_mm_q16(void)
{
    return (_u32)(_lastOdometerSpeedAbs[WALKINGMOTOR_R_ID] * 65536.0);
}
/*
 * PID调节行走电机当前PWM输出函数
 */
static void _control_walkingmotor_speed_byid(int id)
{
    const float PWM_OUT_MAX = CONFIG_MOTOR_PWM_PERIOD;

    if (_motorSpeedMm[id] == 0) {                                               // Set the speed to 0, then stop the walking motor immediately
        _set_walkingmotor(id, 0, MOTOR_CTRL_STATE_BRAKE);
        speed_PWMOUT[id] = 0;
    } else {
        int desiredCtrl = (_motorSpeedMm[id] > 0) ? MOTOR_CTRL_STATE_FORWARD : MOTOR_CTRL_STATE_BACKWARD;

        if (desiredCtrl != _motorCtrlStates[id]) {                              // Direction change, stop the walking motor first
            stalldetector_enterBlindMode(id);
            if (_lastOdometerSpeedAbs[id] > 1.0f) {
                _set_walkingmotor(id, 0, MOTOR_CTRL_STATE_BRAKE);
                speed_PWMOUT[id] = 0;
                return;
            }
            speedLastErr[id] = 0;
            speedErri[id] = 0;
        }
        int desiredSpdAbs = abs(_motorSpeedMm[id]);
        float speedCurrentErr = (float) desiredSpdAbs - _lastOdometerSpeedAbs[id];
        float speedCurrentErrd = speedCurrentErr - speedLastErr[id];
        speedErri[id] += speedCurrentErr;
        speedLastErr[id] = speedCurrentErr;

        speed_PWMOUT[id] = (Kp * speedCurrentErr + Ki * speedErri[id] + Kd * speedCurrentErrd); //PID calculates the next PWM duty cycle value
#ifdef CONFIG_MOTOR_PID_DEBUG
        if (id == WALKINGMOTOR_L_ID) {
            DBG_OUT("%d, pwm %5d, vo %4d, vi %4d\r\n",
                    desiredCtrl, (int)speed_PWMOUT, (int)_lastOdometerSpeedAbs[id], _motorSpeedMm[id]);
            DBG_OUT("speedCurrentErr = %d, speedCurrentErrd = %d, pwm %d, motorSpeed %d\r\n",
                    (int)speedCurrentErr, (int)speedCurrentErrd, (int)speed_PWMOUT[id], _motorSpeedMm[id]);
        }
#endif
        if (speed_PWMOUT[id] > PWM_OUT_MAX)
            speed_PWMOUT[id] = PWM_OUT_MAX;
        if (speed_PWMOUT[id] < 0)
            speed_PWMOUT[id] = 0;

        _set_walkingmotor(id, (int) speed_PWMOUT[id], desiredCtrl);            // Set the PWM duty cycle value calculated by PID
    }
}
/*
 * 左右行走电机速度控制函数
 */
void control_walkingmotor_speed(void)
{
    for (size_t id = 0; id < WALKINGMOTOR_CNT; ++id) {
        _control_walkingmotor_speed_byid(id);
    }
}
/*
 * 设定左右电机速度，单位：mm/s
 */
void set_walkingmotor_speed(_s32 lSpeed, _s32 rSpeed)
{
    _motorSpeedMm[WALKINGMOTOR_L_ID] = lSpeed;
    _motorSpeedMm[WALKINGMOTOR_R_ID] = rSpeed;
}
/*
 * 获得左电机速度，单位：mm/s
 */
_s32 get_walkingmotor_lspeed_set(void)
{
    return (_motorSpeedMm[WALKINGMOTOR_L_ID]);
}
/*
 * 获得右电机速度，单位：mm/s
 */
_s32 get_walkingmotor_rspeed_set(void)
{
    return (_motorSpeedMm[WALKINGMOTOR_R_ID]);
}
/*
 * 行走电机停止并等待函数
 * 预留
 */
void wait_walkingmotor_brake(void)
{
    brake_walkingmotor();
    _u32 startts = getms();
    while ((0 != get_walkingmotor_lspeed_mm_q16()) || (0 != get_walkingmotor_rspeed_mm_q16())) {
        if (getms() - startts > 30000) {
            return;
        }
    }
}
/*
 * 行走电机停止函数
 */
void brake_walkingmotor(void)
{
    _set_walkingmotor(WALKINGMOTOR_L_ID, 0, MOTOR_CTRL_STATE_BRAKE);
    _set_walkingmotor(WALKINGMOTOR_R_ID, 0, MOTOR_CTRL_STATE_BRAKE);
}
/*
 * 机器人是否在地上检测初始化函数
 */
void init_ontheground_detect(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = ONGROUND_RIGHT | ONGROUND_LEFT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ONGROUND_GPIO, &GPIO_InitStructure);
}
/*
 * 机器人是否在地上检测函数
 */
_u8 is_ontheground(void)
{
    if ((0 == GPIO_ReadInputDataBit(ONGROUND_GPIO, ONGROUND_RIGHT))
     || (0 == GPIO_ReadInputDataBit(ONGROUND_GPIO, ONGROUND_LEFT))) {
        return 0;
    } else {
        return 1;
    }
}
/*
 * 边刷电机初始化函数
 */
void init_brushmotor(void)
{
    PIN_SET(BRUSH_L_GPIO, BRUSH_L_PIN, LOW);
    PIN_SET(BRUSH_R_GPIO, BRUSH_R_PIN, LOW);
    pinMode(BRUSH_L_GPIO, BRUSH_L_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinMode(BRUSH_R_GPIO, BRUSH_R_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
}
static _u32 speedctl_frequency = 0;
static _u32 ontheground_frequency = 0;
/*
 * 行走电机速度控制和反馈编码检测函数
 */
void speedctl_heartbeat(void)
{
    _u32 currentTs = getms();
    _u32 delta = currentTs - speedctl_frequency;

    if (delta >= CONF_MOTOR_HEARTBEAT_DURATION) {
        speedctl_frequency = currentTs;
        _refresh_walkingmotor_odometer(delta);                  //定时获取反馈编码值
        control_walkingmotor_speed();                           //进而进行速度控制
    }

    if ((currentTs - ontheground_frequency) >= 1000) {          //1秒落地检测 ，落地发声
        ontheground_frequency = currentTs;
        if (!is_ontheground()) {
            beep_beeper(4000, 80, 2);
        }
    }
}

/////////////////////////

void stalldetector_init(void)
{
    memset(_stallFilterBlindModeTS, 0, sizeof(_stallFilterBlindModeTS));
    memset(_stallDetectorFilter, 0, sizeof(_stallDetectorFilter));
    _stallDetectorTS = 0;
    _stallBitmap = 0;
}

static _u8 countbit (_u32 x) {
    _u32 n;

    n = (x >> 1) & 033333333333;
    x = x - n;
    n = (n >> 1) & 033333333333;
    x = x - n;
    x = (x + (x >> 3)) & 030707070707;
    return (_u8)(x % 63);
}

void stalldetector_enterBlindMode(_u8 id)
{
    _stallFilterBlindModeTS[id] = getms();
}

/*
 * 速度堵转检测
 */
void stalldetector_heartbeat(void)
{
    _u8 id;
    _u8 n;

    if (getms() - _stallDetectorTS < CONF_MOTOR_HEARTBEAT_DURATION) {
        return ;
    }
    _stallDetectorTS = getms();

    for (id = 0; id < WALKINGMOTOR_CNT; id++) {
        if (getms() - _stallFilterBlindModeTS[id] < CONF_MOTOR_STALL_BLINDMODE_DURATION) {
            _stallDetectorFilter[id] = 0;
            _stallBitmap = 0;
            continue;
        }

        _stallDetectorFilter[id] <<= 1;
        if (speed_PWMOUT[id] > CONF_MOTOR_STALL_PWM_THRESHOLD && _lastOdometerSpeedAbs[id] < 1.0f) {
            _stallDetectorFilter[id] |= 0x01;
        } else {
            _stallDetectorFilter[id] &= ~0x01;
        }

        n = countbit(_stallDetectorFilter[id]);
        if (n > 25) {
            /* Stall is detected. */
            if (!(_stallBitmap & (1 << id))) {
                DBG_OUT("motor %d stall flag set.\r\n", id);
            }
            _stallBitmap |= (1 << id);
            beep_beeper(4000, 100, 2);
        } else {
            /* No stall, clean flag. */
            if (_stallBitmap & (1 << id)) {
                DBG_OUT("motor %d stall flag clear.\r\n", id);
            }
            _stallBitmap &= ~(1 << id);
        }
    }
}
/*
 * Get the locked-rotation map into the collision state
 */
_u8 stalldetector_is_bumped(void)
{
    _u8 id;
    _u8 bump_flag = 0xff;

    for (id = 0; id < WALKINGMOTOR_CNT; id++) {
        if (_stallBitmap & (1 << id)) {
            bump_flag &= ~(1 << id);
        }
    }
    return bump_flag;
}
/*
 * 获取堵转状态
 */
_u8 stalldetector_is_stalled(void)
{
    return _stallBitmap;
}

