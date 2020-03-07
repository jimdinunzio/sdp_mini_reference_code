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
// This code has been extensively modified by Alex Brown  rbirac@cox.net
// The above disclaimers apply to the modifications.

#include "common/common.h"
#include "motor.h"
#include "beep.h"
#include "drv/timer.h"
#include "drv/exti.h"
#include "math.h"          //  for fabs() only used once!!!

#define WALKINGMOTOR_CNT        2
#define WALKINGMOTOR_L_ID       0
#define WALKINGMOTOR_R_ID       1

#define CONFIG_MOTOR_PID_DEBUG      1

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


static float _motorDeltaDist[WALKINGMOTOR_CNT];
static float  _motorDistAccumulated[WALKINGMOTOR_CNT];

static _s32 _encoderTicksDelta[WALKINGMOTOR_CNT];       // counts encoder ticks
static float _lastEncoderTicksDelta[WALKINGMOTOR_CNT];  //    ticks
static float _odometerSpeed[WALKINGMOTOR_CNT];          // mm/sec
static float _speedLimited[WALKINGMOTOR_CNT];           // mm/sec

static _s32 _motorSpeedMm[WALKINGMOTOR_CNT];

static const float Kf = PID_KF;  
static const float Kp = PID_KP;  
static const float Ki = PID_KI;
static const float Kd = PID_KD;   
static const float Spd_SF = (float)(PWM_MAX - PWM_ZRO) / (float) SPD_MAX;

static const _u32 turn_radius = TURN_RADIUS;        // in mm.  
static _s32 velocity_Cmd = 0;                       // in mm/sec
static _s32 turnRate_Cmd = 0;                       // in milliradians/sec

static float speedLastErr[WALKINGMOTOR_CNT];
static float speedErri[WALKINGMOTOR_CNT];
static float PWMOUT[WALKINGMOTOR_CNT];

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
    tim_base.TIM_Period = (PWM_PERIOD - 1);    // set overall PWM period
    tim_base.TIM_Prescaler = 72;               // set to count microseconds
    tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(pwm->tim, &tim_base);

    /* Motor PWM ouput channel configure. */
    tim_oc.TIM_OCMode      = TIM_OCMode_PWM2;
    tim_oc.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc.TIM_Pulse       = PWM_ZRO; 
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

    if (duty > PWM_MAX) {
        duty = PWM_MAX;
    }
    if (duty < PWM_MIN) {
        duty = PWM_MIN;
    }

//    tim_set_compare(pwm->tim, pwm->tim_ch, PWM_PERIOD - duty); 
   tim_set_compare(pwm->tim, pwm->tim_ch, duty);

    if (duty > 0) {
        pinMode(pwm->port, pwm->pin, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    }
}


/**
 @brief Drive motor PWM set
 @param id - motor id.
 @return none.

*/
static void _set_walkingmotor_PWM(_u8 id, int duty)
{
//    const pwm_port_t *pwm;

    if (id >= WALKINGMOTOR_CNT) {
        return ;
    }

    // Enable PWM fw signal pin for pulse                                          //attempt to square up output pulses
    motor_set_duty(&_motor_cfg[id].fw_pwm, duty);                                  // removing capacitors on L1101 outputs reduces
    motor_set_duty(&_motor_cfg[id].bk_pwm, duty);                                  // removing capacitors on L1101 outputs reduces
    // Set PWM bk signal pin low   to allow pulse on fw of any duration            // pulse turn off time constant to about 25 usec.
//    pwm = &_motor_cfg[id].bk_pwm;
//    pinMode(pwm->port, pwm->pin, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
//    pinSet(pwm->port,  pwm->pin, Bit_RESET);pwm = &_motor_cfg[id].bk_pwm;

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

//   /* 开启电机电源. */
    // this turns on "CURRENT_SET" on J21-9, needed to get motor drive power
    pinMode(MOTO_EN_PORT, MOTO_EN_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinSet(MOTO_EN_PORT,  MOTO_EN_PIN, Bit_SET);

    for (i = 0; i < _countof(_motor_cfg); i++) {
        _init_motor_pwm(&_motor_cfg[i].fw_pwm);
        _init_motor_pwm(&_motor_cfg[i].bk_pwm);

        if (_motor_cfg[i].oc_mon.port != NULL) {
            pinMode(_motor_cfg[i].oc_mon.port, _motor_cfg[i].oc_mon.pin, GPIO_Mode_IN_FLOATING, GPIO_Speed_10MHz);
        }
    }

    memset(_motorSpeedMm, 0, sizeof(_motorSpeedMm));
    memset(speedLastErr, 0, sizeof(speedLastErr));
    memset(speedErri, 0, sizeof(speedErri));
    DBG_OUT("Walking motor initialized with pid(%d.%d, %d.%d, %d.%d, %d.%d)\r\n",
            (int)Kf, (int)((Kf-(int)Kf)*10),
            (int)Kp, (int)((Kp-(int)Kp)*10),
            (int)Ki, (int)((Ki-(int)Ki)*10),
            (int)Kd, (int)((Kd-(int)Kd)*10));
}


//   ENCODER CODE  ******************************************************

        
        
/* Encoder interrupt handlers for normal encoders (many tick/revolution).
 * Interrupt on rising edge channel A and determine direction from chan B
 */
void encoder_LA_exti_cb(void)   
{
    if (pinRead(ENCODER_L2_PORT, ENCODER_L2_PIN)==1){
      _encoderTicksDelta[WALKINGMOTOR_L_ID] += 1;
    }
    else {
      _encoderTicksDelta[WALKINGMOTOR_L_ID] -= 1;
    }
//    DBG_OUT("LaR %d \r\n",_encoderTicksDelta[WALKINGMOTOR_L_ID]);
}

void encoder_RA_exti_cb(void)
{
    if (pinRead(ENCODER_R2_PORT, ENCODER_R2_PIN)==1){
      _encoderTicksDelta[WALKINGMOTOR_R_ID] += 1;
    }
    else {
      _encoderTicksDelta[WALKINGMOTOR_R_ID] -= 1;
    }
//    DBG_OUT("RaR %d \r\n",_encoderTicksDelta[WALKINGMOTOR_R_ID]);
}

/* Encoder interrupt handlers for low count encoders (few tick/revolution).
 * Interrupt on rising and falling edge of each channel
 */

void encoder_HLA_exti_cb(void)
{
    if (pinRead(ENCODER_L2_PORT, ENCODER_L2_PIN)==pinRead(ENCODER_L1_PORT, ENCODER_L1_PIN)){
        _encoderTicksDelta[WALKINGMOTOR_L_ID] += 1;
    }
    else {
        _encoderTicksDelta[WALKINGMOTOR_L_ID] -= 1;
    }
//    DBG_OUT("L1 %d\r\n", _encoderTicksDelta[WALKINGMOTOR_L_ID]);
}

void encoder_HLB_exti_cb(void)
{
      if (pinRead(ENCODER_L2_PORT, ENCODER_L2_PIN)==pinRead(ENCODER_L1_PORT, ENCODER_L1_PIN)){  //swapped + & -
        _encoderTicksDelta[WALKINGMOTOR_L_ID] -= 1;
    }
    else {
        _encoderTicksDelta[WALKINGMOTOR_L_ID] += 1;
    }
//    DBG_OUT("L2 %d\r\n", _encoderTicksDelta[WALKINGMOTOR_L_ID]);
}

void encoder_HRA_exti_cb(void)
{
    if (pinRead(ENCODER_R2_PORT, ENCODER_R2_PIN)==pinRead(ENCODER_R1_PORT, ENCODER_R1_PIN)){
        _encoderTicksDelta[WALKINGMOTOR_R_ID] += 1;
    }
    else {
        _encoderTicksDelta[WALKINGMOTOR_R_ID] -= 1;
    }
//    DBG_OUT("R1\r\n");
}
void encoder_HRB_exti_cb(void)
{
    if (pinRead(ENCODER_R2_PORT, ENCODER_R2_PIN)==pinRead(ENCODER_R1_PORT, ENCODER_R1_PIN)){
        _encoderTicksDelta[WALKINGMOTOR_R_ID] -= 1;
    }
    else {
        _encoderTicksDelta[WALKINGMOTOR_R_ID] += 1;
    }
//    DBG_OUT("r2\r\n");
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
        pinMode(_motor_cfg[i].encoder[0].port, _motor_cfg[i].encoder[0].pin, GPIO_Mode_IPU, GPIO_Speed_50MHz);
        pinMode(_motor_cfg[i].encoder[1].port, _motor_cfg[i].encoder[1].pin, GPIO_Mode_IPU, GPIO_Speed_50MHz);
    }

    if(HIRES_ENC == 0){   // If HIRES turned off, encoder interrupts only on rising edge of channel A
      GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_L_ID].encoder[0].port), _motor_cfg[WALKINGMOTOR_L_ID].encoder[0].exti);
      GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_R_ID].encoder[0].port), _motor_cfg[WALKINGMOTOR_R_ID].encoder[0].exti);
      exti_reg_callback(_motor_cfg[WALKINGMOTOR_L_ID].encoder[0].exti, EXTI_Trigger_Rising, encoder_LA_exti_cb); 
      exti_reg_callback(_motor_cfg[WALKINGMOTOR_R_ID].encoder[0].exti, EXTI_Trigger_Rising, encoder_RA_exti_cb); 
    }
    else{                // interrupts on both rising and falling edges of both channels A & B
      GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_L_ID].encoder[0].port), _motor_cfg[WALKINGMOTOR_L_ID].encoder[0].exti);
      GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_R_ID].encoder[0].port), _motor_cfg[WALKINGMOTOR_R_ID].encoder[0].exti);
      exti_reg_callback(_motor_cfg[WALKINGMOTOR_L_ID].encoder[0].exti, EXTI_Trigger_Rising_Falling, encoder_HLA_exti_cb);
      exti_reg_callback(_motor_cfg[WALKINGMOTOR_R_ID].encoder[0].exti, EXTI_Trigger_Rising_Falling, encoder_HRA_exti_cb);

      GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_L_ID].encoder[1].port), _motor_cfg[WALKINGMOTOR_L_ID].encoder[1].exti);
      GPIO_EXTILineConfig(GPIO_PortSource(_motor_cfg[WALKINGMOTOR_R_ID].encoder[1].port), _motor_cfg[WALKINGMOTOR_R_ID].encoder[1].exti);
      exti_reg_callback(_motor_cfg[WALKINGMOTOR_L_ID].encoder[1].exti, EXTI_Trigger_Rising_Falling, encoder_HLB_exti_cb);
      exti_reg_callback(_motor_cfg[WALKINGMOTOR_R_ID].encoder[1].exti, EXTI_Trigger_Rising_Falling, encoder_HRB_exti_cb);
    }
}

/*
 * MOTOR ODOMETER INITIALIZATION
 */
void init_walkingmotor_odometer(void)
{
    memset(_motorDeltaDist, 0, sizeof(_motorDeltaDist));
    memset(_motorDistAccumulated, 0, sizeof(_motorDistAccumulated));
    
    memset(_encoderTicksDelta, 0, sizeof(_encoderTicksDelta));
    memset(_lastEncoderTicksDelta, 0, sizeof(_lastEncoderTicksDelta));
    memset(_odometerSpeed, 0, sizeof(_odometerSpeed));
    init_extix();
}

/*
 * REFRESH ODOMETRY (periodic update)
 */
static void _refresh_walkingmotor_odometer(_u32 durationMs)
{
    _u8 cnt;
    float dist_mm;
    
    // Read ticks since last refresh
    _u32 irqSave = enter_critical_section();      // disable interrupt 
    for (cnt = 0; cnt < WALKINGMOTOR_CNT; ++cnt) 
    { 
        _lastEncoderTicksDelta[cnt] = (float) _encoderTicksDelta[cnt];
        _encoderTicksDelta[cnt] = 0;
    }
    leave_critical_section(irqSave);  // enable interrupt

    // Calculate odometry data
    if (durationMs != 0){       // just to avoid divide by zero error 
        for (cnt = 0; cnt < WALKINGMOTOR_CNT; ++cnt) {
           
            dist_mm = 1000.0 * ( _lastEncoderTicksDelta[cnt] /(float)_motor_cfg[cnt].speed_factor);
            
            // calculate motor l/r speed  in mm/sec (float)
            _odometerSpeed[cnt] = dist_mm *1000.0 / (float) durationMs; //in mm/sec
            
            // calculate total distance traveled, dist fwd + dist bkwd forever
           _motorDistAccumulated[cnt] += fabs(dist_mm);  // in mm. float
           
           // calculate distance traveled since last call from Request handler
           // forward and backward cancel. 
           _motorDeltaDist[cnt] += dist_mm;  // in mm. float
 //DBG_OUT("odo2  %d   %d  %d \r\n", cnt, (int)_motorDistAccumulated[0],(int)_motorDistAccumulated[1] );        
        }
    }
}

//  return Left cumulative distance  (to RequestHandler)
_u32 walkingmotor_cumulate_ldist_mm(void)
{
    return (_u32) _motorDistAccumulated[WALKINGMOTOR_L_ID];
}
//  return Right cumulative distance  (to RequestHandler)
_u32 walkingmotor_cumulate_rdist_mm(void)
{
    return (_u32) _motorDistAccumulated[WALKINGMOTOR_R_ID];
}

//  return Left delta distance  (to RequestHandler)
float walkingmotor_delta_ldist_mm_f(void)
{
    float delta_dist = _motorDeltaDist[WALKINGMOTOR_L_ID];
     _motorDeltaDist[WALKINGMOTOR_L_ID] = 0;   // reset delta odometry
    return delta_dist;
}

//  return Right delta distance  (to RequestHandler)

float walkingmotor_delta_rdist_mm_f(void)
{
    float delta_dist = _motorDeltaDist[WALKINGMOTOR_R_ID];
    _motorDeltaDist[WALKINGMOTOR_R_ID] = 0;   // reset delta odometry
    return delta_dist;
}

//  return  Left motor speed (mm/sec)  (to motor control below)
_u32 get_walkingmotor_lspeed_mm_q16(void)
{
    return (_u32)(_odometerSpeed[WALKINGMOTOR_L_ID] * 65536.0);
}

//  return  Right motor speed (mm/sec)  (to motor control below)
_u32 get_walkingmotor_rspeed_mm_q16(void)
{
    return (_u32)(_odometerSpeed[WALKINGMOTOR_R_ID] * 65536.0);
}



/*
 * MOTOR CONTROL  (periodic update)  ***************************************
 */


static void _control_walkingmotor_speed_byid(int id)
{
  //  Rate limit input speed command
  float temp = _motorSpeedMm[id] - _speedLimited[id];                           //AB test. only works for positive increase now !!!
  if(temp > 8.0) temp= 8.0;
  if(temp < -8.0) temp = -8.0;
  _speedLimited[id] += temp;
/*
if (_motorSpeedMm[id] == 0) {    //if zero speed command, stop ASAP         //AB should this always brake like this??? how hold on slope?
        PWMOUT[id] = PWM_ZRO;
        
    } else */{
//        float speedCurrentErr = _motorSpeedMm[id] - _odometerSpeed[id];
        float speedCurrentErr = _speedLimited[id] - _odometerSpeed[id];
        float speedCurrentErrd = speedCurrentErr - speedLastErr[id];
        speedLastErr[id] = speedCurrentErr;
        speedErri[id] += speedCurrentErr;
DBG_OUT("odo2   %d  %d    %d  %d \r\n",  (int)_motorSpeedMm[0],(int)_motorSpeedMm[1],(int)PWMOUT[0],(int)PWMOUT[1] );    
        if(SPEED_MODE == 1)  // if output represents CORE speed command
        {
          PWMOUT[id] = PWM_ZRO + Spd_SF * _motorSpeedMm[id];
        }
        else            // if output represents motor drive PWM
        {
          PWMOUT[id] = PWM_ZRO +(Kf * _speedLimited[id]+ Kp * speedCurrentErr + Ki * speedErri[id] + Kd * speedCurrentErrd); 
        }
        
        if (PWMOUT[id] > PWM_MAX)
            PWMOUT[id] = PWM_MAX;
        if (PWMOUT[id] < PWM_MIN)
            PWMOUT[id] = PWM_MIN;
       
    }
    _set_walkingmotor_PWM(id, (int)PWMOUT[id]);
//DBG_OUT("motor22  %d  %d  %d\r\n",id, (int)_motorSpeedMm[id],(int)PWMOUT[id]);
}
/*
 * Do Motor Control for both motors
 */
void control_walkingmotor_speed(void)
{
    for (size_t id = 0; id < WALKINGMOTOR_CNT; ++id) {
        _control_walkingmotor_speed_byid(id);
    }
}
/*
 * Commanded speed for L & R motors mm/sec   (from main, RequestHandler & bump_monitor)
 */
void set_walkingmotor_speed(_s32 lSpeed, _s32 rSpeed)
{
    _motorSpeedMm[WALKINGMOTOR_L_ID] = lSpeed;
    _motorSpeedMm[WALKINGMOTOR_R_ID] = rSpeed;
    
    velocity_Cmd = (rSpeed + lSpeed) / 2;            // added these to calc cmdvel
    turnRate_Cmd = (rSpeed - lSpeed) * 500 /(s32) turn_radius;  // milliradians/sec
}

/*
 * radius of turning = distance between wheel centers / 2  needed by RequestHandler
 */
_u32 walkingmotor_get_radius(void)        // radius to wheel centers in mm.
{
    return turn_radius;
}

/*
 * commanded velocity in mm/ second.  + forward, - backward
 * +100000 offset to represent signed numbers in a u32
 * and - ROBOT_RADIUS to cancel out sonar correction made in CORE processor.
 * These are called only by sonar.c which sends them back to the user.
 */
_u32 walkingmotor_get_veloCmd(void)
{
    return (u32) (velocity_Cmd + 50000 - ROBOT_RADIUS);
}
/*
 * commanded rate of turn in milliradians/second. + left turn, - right turn.
 */
_u32 walkingmotor_get_turnCmd(void) 
{
    return (u32) (turnRate_Cmd + 50000 - ROBOT_RADIUS);
}

/*
 * 行走电机停止函数
 */
void brake_walkingmotor(void)           // called by bump monitor
{
//    _set_walkingmotor(WALKINGMOTOR_L_ID, 0, MOTOR_CTRL_STATE_BRAKE);          //AB  figure out how to handle this for bump monitor!!!!!!!!
//    _set_walkingmotor(WALKINGMOTOR_R_ID, 0, MOTOR_CTRL_STATE_BRAKE);
}
/*
 * 机器人是否在地上检测初始化函数
 */
void init_ontheground_detect(void)                                              //AB  what is this
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = ONGROUND_RIGHT | ONGROUND_LEFT;               // right is PD10, Left is PD1 from daughter board???
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ONGROUND_GPIO, &GPIO_InitStructure);
}
/*
 * ON THE GROUND LOGIC
 */
_u8 is_ontheground(void)    // called by RequestHandler                         //AB more what does this do???
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
void init_brushmotor(void)              // called by main                       //AB  looks like not used, delete call in future!!!
{
//    PIN_SET(BRUSH_L_GPIO, BRUSH_L_PIN, LOW);
//    PIN_SET(BRUSH_R_GPIO, BRUSH_R_PIN, LOW);
//    pinMode(BRUSH_L_GPIO, BRUSH_L_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
//    pinMode(BRUSH_R_GPIO, BRUSH_R_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
}
static _u32 speedctl_frequency = 0;
static _u32 ontheground_frequency = 0;
/*
 * EXECUTIVE CALLING ODOMETRY AND MOTOR CONTROL
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

void stalldetector_enterBlindMode(_u8 id)                                       //AB  not called
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

    if (getms() - _stallDetectorTS < CONF_MOTOR_HEARTBEAT_DURATION) {  // if elapsed time < 16 msec
        return ;
    }
    _stallDetectorTS = getms();

    for (id = 0; id < WALKINGMOTOR_CNT; id++) {
        if (getms() - _stallFilterBlindModeTS[id] < CONF_MOTOR_STALL_BLINDMODE_DURATION) {  // duration = 600 msec ..ModeTS only set above which is never called???
            _stallDetectorFilter[id] = 0;                                                  // so this may never execute!!!
            _stallBitmap = 0;
            continue;
        }

        _stallDetectorFilter[id] <<= 1;
        if (PWMOUT[id] > CONF_MOTOR_STALL_PWM_THRESHOLD && _odometerSpeed[id] < 1.0f) {  //AB is odometerspeed float like comparison???
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
    return 0xFF;                                                                //AB override.  was  bump_flag;  TEST!!!
}
/*
 * 获取堵转状态
 */
_u8 stalldetector_is_stalled(void)
{
    return 0;                                                                   //AB override.  was _stallBitmap; TEST!!!
}

