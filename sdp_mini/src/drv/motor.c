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

#define WALKINGMOTOR_CNT        2
#define WALKINGMOTOR_L_ID       0
#define WALKINGMOTOR_R_ID       1

//#define CONFIG_MOTOR_PID_DEBUG      1

/**
 @brief motor driving port configures.
 */
static motor_cfg_t _motor_cfg[] = {
    {   /**< Left walking motor configure. */
        {MOTO_LF_PWM_PORT, MOTO_LF_PWM_PIN, MOTO_LF_PWM, MOTO_LF_PWM_CHN},
        {MOTO_LB_PWM_PORT, MOTO_LB_PWM_PIN, MOTO_LB_PWM, MOTO_LB_PWM_CHN},
        {NULL, 0, LOW},
        {
            {ENCODER_L1_PORT, ENCODER_L1_PIN, ENCODER_L1_EXTI},
            {ENCODER_L2_PORT, ENCODER_L2_PIN, ENCODER_L2_EXTI},
        },
        ODOMETER_EST_PULSE_PER_METER,
    },

    {   /**< Right walking motor configure. */
        {MOTO_RF_PWM_PORT, MOTO_RF_PWM_PIN, MOTO_RF_PWM, MOTO_RF_PWM_CHN},
        {MOTO_RB_PWM_PORT, MOTO_RB_PWM_PIN, MOTO_RB_PWM, MOTO_RB_PWM_CHN},
        {NULL, 0, LOW},
        {
            {ENCODER_R1_PORT, ENCODER_R1_PIN, ENCODER_R1_EXTI},
            {ENCODER_R2_PORT, ENCODER_R2_PIN, ENCODER_R2_EXTI},
        },
        ODOMETER_EST_PULSE_PER_METER,
    },
};

static encoder_filter_t _encoder_filters[WALKINGMOTOR_CNT][CONFIG_MOTOR_ENCODER_NUM];

static _u32 _motorDeltaTicks[WALKINGMOTOR_CNT];           //行走电机的累计编码器值
static _u32  _motorDistAccumulated[WALKINGMOTOR_CNT];     // 累计的行走距离，单位mm.
static float _motorDistTailing[WALKINGMOTOR_CNT];         // 累计的行走距离不到1mm的剩余值，用于下一次累计.

static _u32 _encoder1TicksDelta[WALKINGMOTOR_CNT];               //detla时间内的编码器值
static _u32 _lastEncoderTicksDelta[WALKINGMOTOR_CNT];           //最近一次 detla时间内的编码器值
static float _lastOdometerSpeedAbs[WALKINGMOTOR_CNT];           //最近一次 detla时间内的速度值

#if defined(CONFIG_MOTOR_ENCODER_NUM) && (CONFIG_MOTOR_ENCODER_NUM == 2)
static _u32 _encoder2TicksDelta[WALKINGMOTOR_CNT];               //detla时间内的编码器值
#endif

static _u8 _motorCtrlStates[WALKINGMOTOR_CNT];                  //行走电机方向
static _s32 _motorSpeedMm[WALKINGMOTOR_CNT];                    //行走电机速度

static const float Kp = 8;                     //PID 比例因子
static const float Ki = 1.6;                   //PID 积分因子
static const float Kd = 0.0;                   //PID 微分因子

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
    tim_oc.TIM_Pulse       = CONFIG_MOTOR_PWM_PERIOD;
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
    tim_set_compare(pwm->tim, pwm->tim_ch, CONFIG_MOTOR_PWM_PERIOD - duty);

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

    /**< Motor driver use 2 pwm(forward, backward). */
    /* Disable forward duty, disable backward duty. */
    motor_set_duty(&_motor_cfg[id].fw_pwm, duty);
    motor_set_duty(&_motor_cfg[id].bk_pwm, duty);
    return ;
}

/**
 @brief Drive motor forward.
 @param id - motor id.
 @return none.

*/
static void _set_walkingmotor_forward(_u8 id, int duty)
{
    const pwm_port_t *pwm;

    if (id >= WALKINGMOTOR_CNT) {
        return ;
    }

    /* Enable forward duty. */
    motor_set_duty(&_motor_cfg[id].fw_pwm, duty);
    /* Disable backward duty. */
    pwm = &_motor_cfg[id].bk_pwm;
    pinMode(pwm->port, pwm->pin, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(pwm->port, pwm->pin, Bit_SET);
    return ;
}

/**
 @brief Drive motor backward.
 @param id - motor id.
 @return none.

*/
static void _set_walkingmotor_backward(_u8 id, int duty)
{
    const pwm_port_t *pwm;

    if (id >= WALKINGMOTOR_CNT) {
        return ;
    }

    /* Disable forward duty. */
    pwm = &_motor_cfg[id].fw_pwm;
    pinMode(pwm->port, pwm->pin, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(pwm->port, pwm->pin, Bit_SET);
    /* Enable backward duty. */
    motor_set_duty(&_motor_cfg[id].bk_pwm, duty);
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

    /* Formward pwm output high. */
    pwm = &_motor_cfg[id].fw_pwm;
    pinMode(pwm->port, pwm->pin, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(pwm->port, pwm->pin, Bit_SET);

    /* Backward pwm output high. */
    pwm = &_motor_cfg[id].bk_pwm;
    pinMode(pwm->port, pwm->pin, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(pwm->port, pwm->pin, Bit_SET);
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

/*
 * 左右行走电机初始化函数
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

    /* 开启电机电源. */
    pinMode(MOTO_EN_PORT, MOTO_EN_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(MOTO_EN_PORT,  MOTO_EN_PIN, Bit_SET);

    for (i = 0; i < _countof(_motor_cfg); i++) {
        _init_motor_pwm(&_motor_cfg[i].fw_pwm);
        _init_motor_pwm(&_motor_cfg[i].bk_pwm);

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


static _u8 encoder_filter(_u8 motor, _u8 id)
{
#ifdef CONFIG_MOTOR_ENCODER_FILTER   /* Enable encoder filter. */
    _u32 level = PIN_READ(_motor_cfg[motor].encoder[id].port,
                          _motor_cfg[motor].encoder[id].pin);
    _u32 ts = getus();
    encoder_filter_t *filter = &_encoder_filters[motor][id];


    if (ts - filter->ts < MOTOR_ENCODER_FILTER_INTERVAL) {
        filter->err++;
        return 0;
    }

    if (level == filter->level) {
        filter->err++;
        return 0;
    }
    filter->ts = ts;
    filter->level = level;
#endif
    return 1;
}

/*
 * 左行走电机编码器中断函数
 * 外部中断，双边沿触发
 */
void encoder_l1_exti_cb(void)
{
    _encoder1TicksDelta[WALKINGMOTOR_L_ID] += encoder_filter(0, 0);
}
void encoder_l2_exti_cb(void)
{
    _encoder2TicksDelta[WALKINGMOTOR_L_ID] += encoder_filter(0, 1);
}

/*
 * 右行走电机编码器中断函数
 * 外部中断，双边沿触发
 */
void encoder_r1_exti_cb(void)
{
    _encoder1TicksDelta[WALKINGMOTOR_R_ID] += encoder_filter(1, 0);
}
void encoder_r2_exti_cb(void)
{
    _encoder2TicksDelta[WALKINGMOTOR_R_ID] += encoder_filter(1, 1);
}

/*
 * 左右行走电机编码器初始化函数
 * 外部中断接收，双边沿触发
 */
static void init_extix(void)
{
    _u8 i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    for (i = 0; i < _countof(_motor_cfg); i++) {
        pinMode(_motor_cfg[i].encoder[0].port, _motor_cfg[i].encoder[0].pin, GPIO_Mode_IPU, GPIO_Speed_50MHz);
        pinMode(_motor_cfg[i].encoder[1].port, _motor_cfg[i].encoder[1].pin, GPIO_Mode_IPU, GPIO_Speed_50MHz);
    }

    GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_L_ID].encoder[0].port), _motor_cfg[WALKINGMOTOR_L_ID].encoder[0].exti);
    GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_R_ID].encoder[0].port), _motor_cfg[WALKINGMOTOR_R_ID].encoder[0].exti);
    exti_reg_callback(_motor_cfg[WALKINGMOTOR_L_ID].encoder[0].exti, EXTI_Trigger_Rising_Falling, encoder_l1_exti_cb);
    exti_reg_callback(_motor_cfg[WALKINGMOTOR_R_ID].encoder[0].exti, EXTI_Trigger_Rising_Falling, encoder_r1_exti_cb);

#if defined(CONFIG_MOTOR_ENCODER_NUM) && (CONFIG_MOTOR_ENCODER_NUM == 2)
    GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_L_ID].encoder[1].port), _motor_cfg[WALKINGMOTOR_L_ID].encoder[1].exti);
    GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_R_ID].encoder[1].port), _motor_cfg[WALKINGMOTOR_R_ID].encoder[1].exti);
    exti_reg_callback(_motor_cfg[WALKINGMOTOR_L_ID].encoder[1].exti, EXTI_Trigger_Rising_Falling, encoder_l2_exti_cb);
    exti_reg_callback(_motor_cfg[WALKINGMOTOR_R_ID].encoder[1].exti, EXTI_Trigger_Rising_Falling, encoder_r2_exti_cb);
#endif
}
/*
 * 左右行走电机编码器初始化函数
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
 * 刷新行走电机的里程数据函数
 */
static void _refresh_walkingmotor_odometer(_u32 durationMs)
{
    _u8 cnt;
    float dist_mm;
    // disable interrupt :
    _u32 irqSave = enter_critical_section();                                    //临界资源保护
    for (cnt = 0; cnt < WALKINGMOTOR_CNT; ++cnt) {                       //得到这段时间内的编码器数据
#ifdef CONFIG_MOTOR_ENCODER_FILTER
        _encoder_filters[cnt][0].err = 0;
        _encoder_filters[cnt][1].err = 0;
#endif
#if defined(CONFIG_MOTOR_ENCODER_NUM) && (CONFIG_MOTOR_ENCODER_NUM == 2)
        if (_encoder1TicksDelta[cnt] > _encoder2TicksDelta[cnt] + 2) {
            _encoder2TicksDelta[cnt] = _encoder1TicksDelta[cnt];
        } else if (_encoder2TicksDelta[cnt] > _encoder1TicksDelta[cnt] + 2) {
            _encoder1TicksDelta[cnt] = _encoder2TicksDelta[cnt];
        }
#endif

        _lastEncoderTicksDelta[cnt] = _encoder1TicksDelta[cnt];
        _encoder1TicksDelta[cnt] = 0;

#if defined(CONFIG_MOTOR_ENCODER_NUM) && (CONFIG_MOTOR_ENCODER_NUM == 2)
        _lastEncoderTicksDelta[cnt] += _encoder2TicksDelta[cnt];
        _encoder2TicksDelta[cnt] = 0;
#endif
    }
    leave_critical_section(irqSave);

    if (durationMs == 0)                                                        //防止除零
        durationMs = 1;

    for (cnt = 0; cnt < WALKINGMOTOR_CNT; ++cnt) {                       //根据这段时间内的编码器数据计算这段时间内速度，即当前速度
        dist_mm = (float)_lastEncoderTicksDelta[cnt] * (1000.0/_motor_cfg[cnt].speed_factor);
		
        _lastOdometerSpeedAbs[cnt] = dist_mm * 1000.0 / durationMs;
#ifdef CONFIG_MOTOR_PID_DEBUG
        if (_lastEncoderTicksDelta[cnt] > 0) {
//            DBG_OUT("encoder[%d] = %d\r\n", cnt, (int)_lastEncoderTicksDelta[cnt]);
        }
#endif

        dist_mm += _motorDistTailing[cnt];
        _motorDistAccumulated[cnt] += (_u32)dist_mm;
        _motorDistTailing[cnt] = dist_mm - (_u32)dist_mm;

        _motorDeltaTicks[cnt] += _lastEncoderTicksDelta[cnt];
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
    _u32 delta_dist = _motorDeltaTicks[WALKINGMOTOR_L_ID];
    _motorDeltaTicks[WALKINGMOTOR_L_ID] = 0;
    return delta_dist * (1000.f / _motor_cfg[WALKINGMOTOR_L_ID].speed_factor);
}

float walkingmotor_delta_rdist_mm_f(void)
{
    _u32 delta_dist = _motorDeltaTicks[WALKINGMOTOR_R_ID];
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

    if (_motorSpeedMm[id] == 0) {                                               //设定速度为0，则立即停止行走电机
        _set_walkingmotor(id, 0, MOTOR_CTRL_STATE_BRAKE);
        speed_PWMOUT[id] = 0;
    } else {
        int desiredCtrl = (_motorSpeedMm[id] > 0) ? MOTOR_CTRL_STATE_FORWARD : MOTOR_CTRL_STATE_BACKWARD;

        if (desiredCtrl != _motorCtrlStates[id]) {                              //方向改变，则先停止行走电机
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

        speed_PWMOUT[id] = (Kp * speedCurrentErr + Ki * speedErri[id] + Kd * speedCurrentErrd); //PID计算下一个PWM占空比值
#ifdef CONFIG_MOTOR_PID_DEBUG
        if (id == WALKINGMOTOR_L_ID) {
//            DBG_OUT("%d, pwm %5d, vo %4d, vi %4d\r\n",
//                    desiredCtrl, (int)speed_PWMOUT, (int)_lastOdometerSpeedAbs[id], _motorSpeedMm[id]);
            DBG_OUT("%5d, %4d, %4d\r\n",
                    (int)speed_PWMOUT[id], (int)_lastOdometerSpeedAbs[id], _motorSpeedMm[id]);
        }
#endif
        if (speed_PWMOUT[id] > PWM_OUT_MAX)
            speed_PWMOUT[id] = PWM_OUT_MAX;
        if (speed_PWMOUT[id] < 0)
            speed_PWMOUT[id] = 0;

        _set_walkingmotor(id, (int) speed_PWMOUT[id], desiredCtrl);            //将PID计算得到的PWM占空比值设定
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
 * 获取堵转映射成碰撞状态
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

