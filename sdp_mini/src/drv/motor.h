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

#pragma once

#define CONFIG_MOTOR_ENCODER_NUM        2

#define CONFIG_MOTOR_ENCODER_FILTER     1
#define MOTOR_ENCODER_FILTER_INTERVAL   100     /**< Encoder filter interval in us. */

#define CONFIG_MOTOR_PWM_PERIOD         (5000)

/**
 @brief Global motor enable port.
 */
#define MOTO_EN_PORT                GPIOB
#define MOTO_EN_PIN                 GPIO_Pin_15  /**< PB15 is motor enable. */

/**
 @brief Left walking motor hardware configurations.
 */
#define MOTO_LF_PWM_PORT            GPIOE       /** PE13 is left forward motor pwm. */
#define MOTO_LF_PWM_PIN             GPIO_Pin_13

#define MOTO_LF_PWM_ID              1           /**< PE13 is TIM1_CH4 */
#define MOTO_LF_PWM_CHN             3
#define MOTO_LF_PWM                 (GET_TIM(MOTO_LF_PWM_ID))

#define MOTO_LB_PWM_PORT            GPIOE       /** PE14 is left forward motor pwm. */
#define MOTO_LB_PWM_PIN             GPIO_Pin_14

#define MOTO_LB_PWM_ID              1           /**< PE14 is TIM1_CH3 */
#define MOTO_LB_PWM_CHN             4
#define MOTO_LB_PWM                 (GET_TIM(MOTO_LB_PWM_ID))

#define MOTO_LI_MONI_PORT           GPIOD       /**< PD5 is left motor overcurrent monitor. */
#define MOTO_LI_MONI_PIN            GPIO_Pin_5

/**
 @brief Right walking motor hardware configurations.
 */
#define MOTO_RF_PWM_PORT            GPIOB       /** PB9 is right motor pwm. */
#define MOTO_RF_PWM_PIN             GPIO_Pin_9

#define MOTO_RF_PWM_ID              4
#define MOTO_RF_PWM_CHN             4           /**< PB9 is TIM4_CH4 */
#define MOTO_RF_PWM                 (GET_TIM(MOTO_RF_PWM_ID))

#define MOTO_RB_PWM_PORT            GPIOB       /** PB8 is right motor pwm. */
#define MOTO_RB_PWM_PIN             GPIO_Pin_8

#define MOTO_RB_PWM_ID              4
#define MOTO_RB_PWM_CHN             3           /**< PB8 is TIM4_CH3 */
#define MOTO_RB_PWM                 (GET_TIM(MOTO_RB_PWM_ID))

#define MOTO_RI_MONI_PORT           GPIOC       /**< PC5 is right motor overcurrent monitor. */
#define MOTO_RI_MONI_PIN            GPIO_Pin_5

/**
 @brief Encoder configure.
 */
#define ENCODER_L1_PORT             GPIOD
#define ENCODER_L1_PIN              GPIO_Pin_6
#define ENCODER_L1_EXTI             6

#define ENCODER_L2_PORT             GPIOD
#define ENCODER_L2_PIN              GPIO_Pin_4
#define ENCODER_L2_EXTI             4

#define ENCODER_R1_PORT             GPIOD
#define ENCODER_R1_PIN              GPIO_Pin_2
#define ENCODER_R1_EXTI             2

#define ENCODER_R2_PORT             GPIOD
#define ENCODER_R2_PIN              GPIO_Pin_3
#define ENCODER_R2_EXTI             3

/**< This value is calibrate by actual test. */
#if defined(CONFIG_MOTOR_ENCODER_NUM) && (CONFIG_MOTOR_ENCODER_NUM == 2)
#define ODOMETER_EST_PULSE_PER_METER  (13500UL*4/5)
#else
#define ODOMETER_EST_PULSE_PER_METER  6750UL
#endif

/**
 @brief motor driving port configure.
 */
typedef struct _motor_cfg {
    pwm_port_t  fw_pwm;         /**< Forward control pwm port. */
    pwm_port_t  bk_pwm;         /**< Backward control pwm port. */
    in_port_t   oc_mon;         /**< Over-current monitoring port. */
    exti_port_t encoder[CONFIG_MOTOR_ENCODER_NUM]; /**< Motor encoder odometer port. */
    _u16        speed_factor;   /**< Odometer speed factor, in pulse per meter. */
} motor_cfg_t;

/**
 @brief motor encoder filter.
 */
typedef struct _encoder_filter {
    _u32 ts;                    /**< Encoder trigger timestamp in us. */
    _u32 level;                 /**< Encoder trigger level. */
    _u16 err;                   /**< Enocder error count. */
} encoder_filter_t;

//边刷电机管脚定义
#define BRUSH_L_GPIO     GPIOC
#define BRUSH_L_PIN      GPIO_Pin_8
#define BRUSH_R_GPIO     GPIOB
#define BRUSH_R_PIN      GPIO_Pin_0

//落地检测相关
#define ONGROUND_GPIO       GPIOD
#define ONGROUND_LEFT       GPIO_Pin_10
#define ONGROUND_RIGHT      GPIO_Pin_1

//行走电机速度控制频率：60hz
#define CONF_MOTOR_HEARTBEAT_FREQ     60
#define CONF_MOTOR_HEARTBEAT_DURATION (1000/(CONF_MOTOR_HEARTBEAT_FREQ))

#define CONF_MOTOR_STALL_STABLE_DURATION    100
#define CONF_MOTOR_STALL_BLINDMODE_DURATION 600
#define CONF_MOTOR_STALL_PWM_THRESHOLD      (CONFIG_MOTOR_PWM_PERIOD*8/10)

#define CONF_MOTOR_SPEED_STALL_DURATION     1000

enum motorCtrlState_t {
    MOTOR_CTRL_STATE_RELEASE = 0,
    MOTOR_CTRL_STATE_FORWARD = 1,
    MOTOR_CTRL_STATE_BACKWARD = 2,
    MOTOR_CTRL_STATE_BRAKE = 3,
};

typedef enum speed_monitor {
    MOTOR_SPEED_MONI_NONE,  /**< No motor speed stall monitor. */
    MOTOR_SPEED_MONI_BUMP,  /**< Motor speed stall mapping as bump action. */
    MOTOR_SPEED_MONI_STALL, /**< Motor speed stall mapping as stall action. */
} speed_monitor_t;


void init_walkingmotor(void);
void set_walkingmotor_lduty(_s32 lDuty, _s32 ctrl);
void set_walkingmotor_rduty(_s32 rDuty, _s32 ctrl);
void init_walkingmotor_odometer(void);
void control_walkingmotor_speed(void);
void set_walkingmotor_speed(_s32 lSpeed, _s32 rSpeed);
_u32 get_walkingmotor_lspeed_mm_q16(void);
_u32 get_walkingmotor_rspeed_mm_q16(void);
_s32 get_walkingmotor_lspeed_set(void);
_s32 get_walkingmotor_rspeed_set(void);
void wait_walkingmotor_brake(void);
void brake_walkingmotor(void);
_u32 walkingmotor_cumulate_ldist_mm(void);
_u32 walkingmotor_cumulate_rdist_mm(void);
float walkingmotor_delta_ldist_mm_f(void);
float walkingmotor_delta_rdist_mm_f(void);

void init_ontheground_detect(void);
_u8 is_ontheground(void);
void init_brushmotor(void);
void speedctl_heartbeat(void);

void stalldetector_init(void);
_u8 stalldetector_is_bumped(void);
_u8 stalldetector_is_stalled(void);
void stalldetector_heartbeat(void);
void stalldetector_enterBlindMode(_u8 id);

#define init_stalldetector      stalldetector_init
