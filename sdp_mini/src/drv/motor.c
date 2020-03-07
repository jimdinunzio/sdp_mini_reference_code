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
#define WALKINGMOTOR_CNT       2
#define WALKINGMOTOR_LEFT_ID   0
#define WALKINGMOTOR_RIGHT_ID  1

static _u32 _motorDeltaTicks[WALKINGMOTOR_CNT];           //行走电机的累计编码器值
static _u32  _motorDistAccumulated[WALKINGMOTOR_CNT];     // 累计的行走距离，单位mm.
static float _motorDistTailing[WALKINGMOTOR_CNT];         // 累计的行走距离不到1mm的剩余值，用于下一次累计.

static _u32 _encoderTicksDelta[WALKINGMOTOR_CNT];               //detla时间内的编码器值
static _u32 _lastEncoderTicksDelta[WALKINGMOTOR_CNT];           //最近一次 detla时间内的编码器值
static float _lastOdometerSpeedAbs[WALKINGMOTOR_CNT];           //最近一次 detla时间内的速度值

static _u8 _motorCtrlStates[WALKINGMOTOR_CNT];                  //行走电机方向
static _s32 _motorSpeedMm[WALKINGMOTOR_CNT];                    //行走电机速度

const float Kp = 25.0;                  //PID 比例因子
const float Ki = 6.0;                   //PID 积分因子
const float Kd = 0.0;                   //PID 微分因子

static float speedLastErr[WALKINGMOTOR_CNT];
static float speedErri[WALKINGMOTOR_CNT];

#define PWM_MAX (10000)
static inline void _set_walkingmotor_duty(_u32 id, _s32 duty, _s32 ctrl)
{
    if (id == WALKINGMOTOR_LEFT_ID) {
        set_walkingmotor_lduty(duty, ctrl);
    } else {
        set_walkingmotor_rduty(duty, ctrl);
    }
}
/*
 * 左右行走电机初始化函数
 */
void init_walkingmotor(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = MOTO_LF_EN | MOTO_LB_EN | MOTO_RF_EN | MOTO_RB_EN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTO_GPIO, &GPIO_InitStructure);

    PIN_SET(MOTO_L_PWM_GPIO_PORT, MOTO_L_PWM_GPIO_PIN, HIGH);
    PIN_SET(MOTO_R_PWM_GPIO_PORT, MOTO_R_PWM_GPIO_PIN, HIGH);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = MOTO_R_PWM_GPIO_PIN | MOTO_L_PWM_GPIO_PIN;
    GPIO_Init(MOTO_R_PWM_GPIO_PORT, &GPIO_InitStructure);

    PIN_SET(MOTO_GPIO, MOTO_LB_EN, LOW);
    PIN_SET(MOTO_GPIO, MOTO_LF_EN, LOW);
    PIN_SET(MOTO_GPIO, MOTO_RF_EN, LOW);
    PIN_SET(MOTO_GPIO, MOTO_RB_EN, LOW);
    TIM_TimeBaseStructure.TIM_Period = (PWM_MAX - 1);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = PWM_MAX;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    set_walkingmotor_lduty(0, MOTOR_CTRL_STATE_RELEASE);
    set_walkingmotor_rduty(0, MOTOR_CTRL_STATE_RELEASE);
    memset(_motorSpeedMm, 0, sizeof(_motorSpeedMm));
    memset(speedLastErr, 0, sizeof(speedLastErr));
    memset(speedErri, 0, sizeof(speedErri));
}

/*
 * 左行走电机释放函数
 * 不使能PWM输出和控制脚
 */
static void _release_walkingmotor_l()
{
    PIN_SET(MOTO_L_PWM_GPIO_PORT, MOTO_L_PWM_GPIO_PIN, HIGH);
    pinMode(MOTO_L_PWM_GPIO_PORT, MOTO_L_PWM_GPIO_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);

    _delay_us(20);              //等效于死区

    PIN_SET(MOTO_GPIO, MOTO_LB_EN, LOW);
    PIN_SET(MOTO_GPIO, MOTO_LF_EN, LOW);
}
/*
 * 右行走电机释放函数
 * 不使能PWM输出和控制脚
 */
static void _release_walkingmotor_r()
{
    PIN_SET(MOTO_R_PWM_GPIO_PORT, MOTO_R_PWM_GPIO_PIN, HIGH);
    pinMode(MOTO_R_PWM_GPIO_PORT, MOTO_R_PWM_GPIO_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);

    _delay_us(20);              //等效于死区

    PIN_SET(MOTO_GPIO, MOTO_RF_EN, LOW);
    PIN_SET(MOTO_GPIO, MOTO_RB_EN, LOW);
}
/*
 * 设置左行走电机占空比和方向函数
 */
void set_walkingmotor_lduty(int dutyCycle, int ctrl)
{
    dutyCycle = abs(dutyCycle);
    if (_motorCtrlStates[WALKINGMOTOR_LEFT_ID] != ctrl) {
        _release_walkingmotor_l();                                              //方向变化，先停止电机
    }
    if (ctrl == MOTOR_CTRL_STATE_RELEASE || ctrl == MOTOR_CTRL_STATE_BRAKE) {
        dutyCycle = 0;
    } else {
        if (dutyCycle > PWM_MAX)
            dutyCycle = PWM_MAX;
    }
    RAW_PWM_SET(MOTO_L_PWM_CHN, MOTO_L_PWM_ID, PWM_MAX - dutyCycle);            //设定PWM占空比=(TIM3_CCR4/ TIM3_ARR)*100

    switch (ctrl) {                                                             //根据设定方向，进行变向或停止
    case MOTOR_CTRL_STATE_RELEASE:
        break;
    case MOTOR_CTRL_STATE_BRAKE:
        PIN_SET(MOTO_GPIO, MOTO_LB_EN, HIGH);
        PIN_SET(MOTO_GPIO, MOTO_LF_EN, HIGH);
        break;
    case MOTOR_CTRL_STATE_FORWARD:
        PIN_SET(MOTO_GPIO, MOTO_LF_EN, LOW);
        PIN_SET(MOTO_GPIO, MOTO_LB_EN, HIGH);
        break;
    case MOTOR_CTRL_STATE_BACKWARD:
        PIN_SET(MOTO_GPIO, MOTO_LB_EN, LOW);
        PIN_SET(MOTO_GPIO, MOTO_LF_EN, HIGH);
        break;
    }
    if (dutyCycle && (_motorCtrlStates[WALKINGMOTOR_LEFT_ID] != ctrl)) {
        pinMode(MOTO_L_PWM_GPIO_PORT, MOTO_L_PWM_GPIO_PIN, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    }
    _motorCtrlStates[WALKINGMOTOR_LEFT_ID] = ctrl;
}
/*
 * 设置右行走电机占空比和方向函数
 */
void set_walkingmotor_rduty(_s32 dutyCycle,_s32 ctrl)
{
    dutyCycle = abs(dutyCycle);
    if (_motorCtrlStates[WALKINGMOTOR_RIGHT_ID] != ctrl) {
        _release_walkingmotor_r();                                              //方向变化，先停止电机
    }
    if (ctrl == MOTOR_CTRL_STATE_RELEASE || ctrl == MOTOR_CTRL_STATE_BRAKE) {
        dutyCycle = 0;
    } else {
        if (dutyCycle > PWM_MAX)
            dutyCycle = PWM_MAX;
    }
    RAW_PWM_SET(MOTO_R_PWM_CHN, MOTO_R_PWM_ID, PWM_MAX - dutyCycle);            //设定PWM占空比=(TIM3_CCR4/ TIM3_ARR)*100
    switch (ctrl) {                                                             //根据设定方向，进行变向或停止
    case MOTOR_CTRL_STATE_RELEASE:
        break;
    case MOTOR_CTRL_STATE_BRAKE:
        PIN_SET(MOTO_GPIO, MOTO_RB_EN, HIGH);
        PIN_SET(MOTO_GPIO, MOTO_RF_EN, HIGH);
        break;
    case MOTOR_CTRL_STATE_FORWARD:
        PIN_SET(MOTO_GPIO, MOTO_RF_EN, LOW);
        PIN_SET(MOTO_GPIO, MOTO_RB_EN, HIGH);
        break;
    case MOTOR_CTRL_STATE_BACKWARD:
        PIN_SET(MOTO_GPIO, MOTO_RB_EN, LOW);
        PIN_SET(MOTO_GPIO, MOTO_RF_EN, HIGH);
        break;
    }
    if (dutyCycle && (_motorCtrlStates[WALKINGMOTOR_RIGHT_ID] != ctrl)) {
        pinMode(MOTO_R_PWM_GPIO_PORT, MOTO_R_PWM_GPIO_PIN, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    }
    _motorCtrlStates[WALKINGMOTOR_RIGHT_ID] = ctrl;
}
/*
 * 左右行走电机编码器初始化函数
 * 外部中断接收，双边沿触发
 */
static void init_extix(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = ODOMETER_RIGHT | ODOMETER_LEFT;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;

    GPIO_Init(ODOMETER_GPIO, &GPIO_InitStructure);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);

    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/*
 * 右行走电机编码器中断函数
 * 外部中断，双边沿触发
 */
void EXTI2_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) != RESET) {

        EXTI_ClearITPendingBit(EXTI_Line2);
        ++_encoderTicksDelta[WALKINGMOTOR_RIGHT_ID];
    }
}
/*
 * 左行走电机编码器中断函数
 * 外部中断，双边沿触发
 */
void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {

        EXTI_ClearITPendingBit(EXTI_Line3);
        ++_encoderTicksDelta[WALKINGMOTOR_LEFT_ID];
    }
}
/*
 * 左右行走电机编码器初始化函数
 */
void init_walkingmotor_odometer(void)
{
    memset(_motorDeltaTicks, 0, sizeof(_motorDeltaTicks));
    memset(_motorDistAccumulated, 0, sizeof(_motorDistAccumulated));
    memset(_motorDistTailing, 0, sizeof(_motorDistTailing));
    
    memset(_encoderTicksDelta, 0, sizeof(_encoderTicksDelta));
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
        _lastEncoderTicksDelta[cnt] = _encoderTicksDelta[cnt];
        _encoderTicksDelta[cnt] = 0;
    }
    leave_critical_section(irqSave);

    if (durationMs == 0)                                                        //防止除零
        durationMs = 1;

    for (cnt = 0; cnt < WALKINGMOTOR_CNT; ++cnt) {                       //根据这段时间内的编码器数据计算这段时间内速度，即当前速度
        dist_mm = (float)_lastEncoderTicksDelta[cnt] * (1000.0/ODOMETER_EST_PULSE_PER_METER);
		
        _lastOdometerSpeedAbs[cnt] = dist_mm * 1000.0 / durationMs;

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
    return _motorDistAccumulated[WALKINGMOTOR_LEFT_ID];
}
/*
 * 计算右行走电机累计里程函数
 * 单位：mm
 */
_u32 walkingmotor_cumulate_rdist_mm(void)
{
    return _motorDistAccumulated[WALKINGMOTOR_RIGHT_ID];
}

float walkingmotor_delta_ldist_mm_f(void)
{
    _u32 delta_dist = _motorDeltaTicks[WALKINGMOTOR_LEFT_ID];
    _motorDeltaTicks[WALKINGMOTOR_LEFT_ID] = 0;
    return delta_dist * (1000.f / ODOMETER_EST_PULSE_PER_METER);
}

float walkingmotor_delta_rdist_mm_f(void)
{
    _u32 delta_dist = _motorDeltaTicks[WALKINGMOTOR_RIGHT_ID];
    _motorDeltaTicks[WALKINGMOTOR_RIGHT_ID] = 0;
    return delta_dist * (1000.f / ODOMETER_EST_PULSE_PER_METER);
}

/*
 * 计算左行走电机当前速度函数
 * 单位：mm/s
 */
_u32 get_walkingmotor_lspeed_mm_q16(void)
{
    return (_u32)(_lastOdometerSpeedAbs[WALKINGMOTOR_LEFT_ID] * 65536.0);
}
/*
 * 计算右行走电机当前速度函数
 * 单位：mm/s
 */
_u32 get_walkingmotor_rspeed_mm_q16(void)
{
    return (_u32)(_lastOdometerSpeedAbs[WALKINGMOTOR_RIGHT_ID] * 65536.0);
}
/*
 * PID调节行走电机当前PWM输出函数
 */
static void _control_walkingmotor_speed_byid(int id)
{
    const float PWM_OUT_MAX = PWM_MAX;
    if (_motorSpeedMm[id] == 0) {                                               //设定速度为0，则立即停止行走电机
        _set_walkingmotor_duty(id, 0, MOTOR_CTRL_STATE_BRAKE);
    } else {
        int desiredCtrl = (_motorSpeedMm[id] > 0) ? MOTOR_CTRL_STATE_FORWARD : MOTOR_CTRL_STATE_BACKWARD;

        if (desiredCtrl != _motorCtrlStates[id]) {                              //方向改变，则先停止行走电机
            if (_lastOdometerSpeedAbs[id] > 1.0f) {
                _set_walkingmotor_duty(id, 0, MOTOR_CTRL_STATE_BRAKE);
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

        float speed_PWMOUT = (Kp * speedCurrentErr + Ki * speedErri[id] + Kd * speedCurrentErrd); //PID计算下一个PWM占空比值
        if (speed_PWMOUT > PWM_OUT_MAX)
            speed_PWMOUT = PWM_OUT_MAX;
        if (speed_PWMOUT < 0)
            speed_PWMOUT = 0;

        _set_walkingmotor_duty(id, (int) speed_PWMOUT, desiredCtrl);            //将PID计算得到的PWM占空比值设定
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
    _motorSpeedMm[WALKINGMOTOR_LEFT_ID] = lSpeed;
    _motorSpeedMm[WALKINGMOTOR_RIGHT_ID] = rSpeed;
}
/*
 * 获得左电机速度，单位：mm/s
 */
_s32 get_walkingmotor_lspeed_set(void)
{
    return (_motorSpeedMm[WALKINGMOTOR_LEFT_ID]);
}
/*
 * 获得右电机速度，单位：mm/s
 */
_s32 get_walkingmotor_rspeed_set(void)
{
    return (_motorSpeedMm[WALKINGMOTOR_RIGHT_ID]);
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
    set_walkingmotor_lduty(0, MOTOR_CTRL_STATE_BRAKE);
    set_walkingmotor_rduty(0, MOTOR_CTRL_STATE_BRAKE);
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
    if ((0 == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1)) || (0 == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10))) {
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
