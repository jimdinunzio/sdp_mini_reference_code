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

#pragma once

/*****************************************************************************
  User defined constants
  This section defines constants which may be unique to each robot configuration.
  It has multiple sets of data.  Uncomment the one to be used.

Notes:
  The PWM signal can represent either the speed commanded by the CORE module
  or a motor command calculated from the PID gains.  If SPEED_MODE = 1, the 
  SPD_MAX signal must be set as desired.  If SPEED_MODE = 0, then the motor 
  command is calculated from the PID gains, and SPD_MAX value has no effect.

  The "sdpmini_config.c" file may also need to be changed using the 
  "slamware_config_tool", placed in the source code and recompiled.
*/

//========================================================================
//AB constants
/*
#define SPEED_MODE          (1)      // motor PWM = 0, speed PWM = 1
#define SPD_MAX             (1000)   // Max +/- speed cmd in mm/sec 

// set to 1 for low resolution encoders, 0 for higher resolution encoders
#define HIRES_ENC           (0)      // should be least 1000 counts/meter 

#define PWM_MAX             (2000)   // pwm at max fwd motor cmd or speed cmd
#define PWM_ZRO             (1500)   // pwm at zero motor cmd or speed cmd 
#define PWM_MIN             (1000)   // pwm at max bkwd motor cmd or speed cmd
#define PWM_PERIOD          (20000)  // period between pulses

#define PID_KF              (0.4)    // FeedForward gain 
#define PID_KP              (0.2)    // proportional gain
#define PID_KI              (0.0)    // integral gain                        
#define PID_KD              (0.0)    // derivative gain

// TURN_RADIUS is half the distance between the wheel centers
#define TURN_RADIUS                     (150)    // radius for turning in mm. 
    
// ROBOT_RADIUS is half the overall diameter of the robot
#define ROBOT_RADIUS                    (180)   
    
//< This value is calibrated by actual test./
#define ODOMETER_EST_PULSE_PER_METER  12580UL
*/
//*****************************************************************************

/*
 //BW constants

#define SPEED_MODE          ( 1 )    // motor PWM = 0, speed PWM = 1
#define SPD_MAX             (500)   // Max +/- speed cmd in mm/sec 

// set to 1 for low resolution encoders, 0 for higher resolution encoders
#define HIRES_ENC           (0)      // should be least 1000 counts/meter below

#define PWM_MAX             (2000)   // pwm at max fwd motor cmd or speed cmd
#define PWM_ZRO             (1500)   // pwm at zero motor cmd or speed cmd 
#define PWM_MIN             (1000)   // pwm at max bkwd motor cmd or speed
#define PWM_PERIOD          (20000)  // period between pulses

#define PID_KF              (0.4)    // FeedForward gain 
#define PID_KP              (0.2)    // proportional gain
#define PID_KI              (0.0)    // integral gain                        
#define PID_KD              (0.0)    // derivative gain

// TURN_RADIUS is half the distance between the wheel centers
#define TURN_RADIUS                     (180)    // radius for turning in mm. 

// ROBOT_RADIUS is half the overall diameter of the robot
#define ROBOT_RADIUS                    (180)  

//< This value is calibrate by actual test. 
#define ODOMETER_EST_PULSE_PER_METER  67703UL //135417UL
*/
//*****************************************************************************
/*
 //GM constants

#define SPEED_MODE          (1)      // motor PWM = 0, speed PWM = 1
#define SPD_MAX             (1000)   // Max +/- speed cmd in mm/sec 

// set to 1 for low resolution encoders, 0 for higher resolution encoders
#define HIRES_ENC           (1)      // should be least 1000 counts/meter below

#define PWM_MAX             (2000)   // pwm at max fwd motor cmd or speed cmd
#define PWM_ZRO             (1500)   // pwm at zero motor cmd or speed cmd 
#define PWM_MIN             (1000)   // pwm at max bkwd motor cmd or speed cmd
#define PWM_PERIOD          (20000)  // period between pulses

#define PID_KF              (0.4)    // FeedForward gain 
#define PID_KP              (0.2)    // proportional gain
#define PID_KI              (0.0)    // integral gain                        
#define PID_KD              (0.0)    // derivative gain

// TURN_RADIUS is half the distance between the wheel centers
#define TURN_RADIUS                     (195)    // radius for turning in mm. 

// ROBOT_RADIUS is half the overall diameter of the robot
#define ROBOT_RADIUS                    (225)      

//< This value is calibrate by actual test./
#define ODOMETER_EST_PULSE_PER_METER  300UL
*/
//******************************************************************************
//BL constants

#define SPEED_MODE          (1)      // motor PWM = 0, speed PWM = 1
#define SPD_MAX             (1000)   // Max +/- speed cmd in mm/sec 

// set to 1 for low resolution encoders, 0 for higher resolution encoders
#define HIRES_ENC           (0)      // should be least 1000 counts/meter 

#define PWM_MAX             (2000)   // pwm at max fwd motor cmd or speed cmd
#define PWM_ZRO             (1500)   // pwm at zero motor cmd or speed cmd 
#define PWM_MIN             (1000)   // pwm at max bkwd motor cmd or speed cmd
#define PWM_PERIOD          (20000)  // period between pulses

#define PID_KF              (0.4)    // FeedForward gain 
#define PID_KP              (0.2)    // proportional gain
#define PID_KI              (0.0)    // integral gain                        
#define PID_KD              (0.0)    // derivative gain

// TURN_RADIUS is half the distance between the wheel centers
#define TURN_RADIUS                     (244)    // radius for turning in mm. 
    
// ROBOT_RADIUS is half the overall diameter of the robot
#define ROBOT_RADIUS                    (610)   
    
//< This value is calibrated by actual test./
#define ODOMETER_EST_PULSE_PER_METER  29880L



//*****************************************************************************

#define CONFIG_MOTOR_ENCODER_NUM        2

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
 @brief Right walking motor hardware durations.
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


/**
 @brief motor driving port configure.
 */
typedef struct _motor_cfg {
    pwm_port_t  fw_pwm;         /**< Forward control pwm port. */
    pwm_port_t  bk_pwm;         /**< Backward control pwm port. */
    in_port_t   oc_mon;         /**< Over-current monitoring port. */
    exti_port_t encoder[CONFIG_MOTOR_ENCODER_NUM]; /**< Motor encoder odometer port. */
    _u32        speed_factor;   /**< Odometer speed factor, in pulse per meter. */
} motor_cfg_t;


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
#define CONF_MOTOR_STALL_PWM_THRESHOLD      (PWM_PERIOD*8/10)                   //AB

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

_u32 walkingmotor_get_radius(void);                                                          //AB radius to wheel centers in mm.
_u32 walkingmotor_get_veloCmd(void);                                                         //AB forward velocity command in mm/sec
_u32 walkingmotor_get_turnCmd(void);                                                         //AB turnrate in milliradians/sec

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

