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

/**
 @breif PCB REF REV 3.0
 *   
 *    I/O     DEFINITION                   I/O    DEFINITION                 I/O     DEFINITION                    I/O    DEFINITION              I/O     DEFINITION
 *
 *    PA0     CHARGE_PWM(ADC123_IN0)       PB0                               PC0                                   PD0                            PE0 
 *    PA1                                  PB1                               PC1     BOTTOM_IR_R2(ADC123_IN11)     PD1    GROUND_DETECT_L(GPIO)   PE1 
 *    PA2     DISPLAY_RX(USART2_TX)        PB2                               PC2     BOTTOM_IR_R1(ADC123_IN12)     PD2    ENCODER_SENSOR_R(EXTI)  PE2     BATT_READY(GPIO)      
 *    PA3     DISPLAY_TX(USART2_RX)        PB3                               PC3     BATT_MONITOR(ADC123_IN13)     PD3    ENCODER_SENSOR_L(EXTI)  PE3     BATT_CHRG(GPIO)     
 *    PA4     BOTTOM_IR_R4(ADC12_IN4)      PB4                               PC4     BOTTOM_IR_R3(ADC12_IN14)      PD4    MOTO_LF_EN(GPIO)        PE4     BATT_FAULT(GPIO)        
 *    PA5                                  PB5    BUMP_DETECT_L(GPIO)        PC5     MOTO_RI_MONITOR(GPIO)         PD5    MOTO_LI_MONITOR(GPIO)   PE5     SONAR_ECHO1(GPIO)      
 *    PA6     BATT_DETECT(ADC12_IN6)       PB6                               PC6                                   PD6    MOTO_RF_EN(GPIO)        PE6     
 *    PA7                                  PB7                               PC7     BOTTOM_IR_E(BASIC TIMER)      PD7    MOTO_RB_EN(GPIO)        PE7     SONAR_ECHO2
 *    PA8                                  PB8    MOTO_RB_PWM(TIM4_CH3)      PC8                                   PD8                            PE8     SONAR_ECHO3
 *    PA9     PCIE_CRX(USART1_TX)          PB9    MOTO_RF_PWM(TIM4_CH4)      PC9     PCIE_nCCMD(GPIO)              PD9    MOTO_LB_EN(GPIO)        PE9     SONAR_ECHO4
 *    PA10    PCIE_CTX(USART1_RX)          PB10                              PC10    PC10_TX(USART3_TX)            PD10   GROUND_DETECT_R(GPIO)   PE10    SONAR_TRIG1
 *    PA11                                 PB11                              PC11    PC11_RX(USART3_RX)            PD11                           PE11    SONAR_TRIG2
 *    PA12    PCIE_CBUSY(GPIO)             PB12   LED_WS2812(GPIO)           PC12                                  PD12   HOME_IR_R1(TIM4_CH1)    PE12    SONAR_TRIG3
 *    PA13    PROGRAMMER(SWDIO)            PB13   BUMP_DETECT_R(GPIO)        PC13                                  PD13   HOME_IR_R2(TIM4_CH2)    PE13    MOTO_LF_PWM(TIM1_CH3)
 *    PA14    PROGRAMMER(SWCLK)            PB14                              PC14                                  PD14   HOME_IR_R3(TIM4_CH3)    PE14    MOTO_LB_PWM(TIM1_CH4)
 *    PA15    BEEP_PWM(TIM2_CH1_ETR)       PB15   CURRENT_SET(GPIO)          PC15                                  PD15                           PE15    SONAR_TRIG4
 *
 * Change since PCB REV 1.0
 *    PE5, PE7 ~ PE12, PE15 as SONAR interface
 *    MOTO_L_PWM move to PE14
 *    BOTTOM_IR_R1 move to PC2
 *    PC10, PC11 as UART
 *    Remove FRONT_IR
 *    PB12 as color LED WS2812 control.
 */
 
#include "common/common.h"
#include "drv/serial_channel.h"
#include "drv/time.h"
#include "drv/beep.h"
#include "drv/battery.h"
#include "drv/motor.h"
#include "drv/bump.h"
#include "drv/distir.h"
#include "drv/homeir.h"
#include "drv/watchdog.h"
#include "drv/sonar.h"
#include "drv/led.h"
#include "drv/drv_ctrlbus.h"
#include "drv/wifi_monitor.h"
#include "bump_monitor.h"
/*
* 初始化板级外设函数
*/
static _s32 init_dev(void)
{
#ifdef _DEBUG
    usart_begin(GET_USART(DBG_USART_ID), 115200);                       //初始化调试串口2
#endif
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
    drv_led_init();
#endif
    drv_serialchannel_init(GET_USART(USART_CTRLBUS_ID), 115200);        //初始化跟slamcore通讯的串口1
    net_bind(drv_serialchannel_getchannel());                           //将该串口绑定上interchip协议
    init_battery();                                                     //初始化电池电量、充电相关
    init_beep();                                                        //初始化蜂鸣器，用于各种声音提示
    init_drv_ctrlbus();                                                 //初始化ctrlbus相关
    init_distir();                                                      //初始化IR测距相关，包括正前下4个红外（测正下方）
    init_homeir();                                                      //初始化IR测距相关，包括3个 Homeir
    init_brushmotor();                                                  //
    init_walkingmotor();                                                //初始化两路行走电机，输出的速度分辨率为-1000 ~ 1000
    init_walkingmotor_odometer();                                       //初始化两路行走电机上的编码器，中断输入
    set_walkingmotor_speed(0, 0);                                       //速度设定 左：0mm/s 右：0mm/s
    init_ontheground_detect();                                          //初始化是否在地上的检测脚，用于判断本机时候在地上还是架空
    init_bump_detect();                                                 //初始化碰撞检测脚
    init_bumpermonitor();
    init_stalldetector();
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
    init_sonar();
#endif
    init_wifi_monitor();
    return 1;
}

extern _u8 sdp_status;
_u32 shutdownHeartbeatFrequency = 0;
/*
 * 与slamcore连接保持判定函数
 */
void shutdown_heartbeat(void)
{
    if ((sdp_status == SdpStatusStartingUp)
     || (sdp_status == SdpStatusUpgradingFirmware)) {
        return ;
    }
    if ((getms() - shutdownHeartbeatFrequency) > 1000)
    {
        shutdownHeartbeatFrequency = getms();
        set_walkingmotor_speed(0, 0);                           //与slamcore断开连接则停止行走
        sdp_status = SdpStatusCoreDisconnected;
    }
}
void shutdown_ticks_update(void)
{
    shutdownHeartbeatFrequency = getms();
    if (sdp_status == SdpStatusStartingUp || sdp_status == SdpStatusCoreDisconnected) {
        /* Switch status to idle when core is re-connected. */
        sdp_status = SdpStatusIdle;
    }
}
/*
 * 关闭设备函数
 * 此处可以关闭一些外设，具体视情况而定
 */
static void dev_shutdown(void)
{
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
    drv_led_shutdown();
#endif
    drv_serialchannel_shutdown(GET_USART(DATAOUT_USART_ID));
    shutdown_bumpermonitor();
}
/*
 * 模块循环处理函数
 */
static void dev_heartbeat(void)
{
    heartbeat_battery();
    heartbeat_bumpermonitor();
    heartbeat_beep();
    heartbeat_distir();
    heartbeat_homeir();

#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
    heartbeat_sonar();
#endif
    shutdown_heartbeat();
    speedctl_heartbeat();
    stalldetector_heartbeat();
}
/*
 * 故障模式处理函数
 */
static void on_abort_mode(void)                                 //故障模式处理，此处可以关机、关中断等，视具体情况而定
{
    dev_shutdown();
    cli();
    while (1);
}

static _u32 _led_ticks = 0;
static _u8  _led_status = 0;

/*
 * 指示灯心跳函数
 */
static void led_heartbeat(void)
{
    _u8  n;

    n = get_electricitypercentage();
    if (n < 15) {
        if (getms() - _led_ticks < 250) {
            return ;
        }
        /* Blink red led when battery is less than 15%. */
        drv_led_set(_led_status, 0, 0);
        _led_status = _led_status ? 0 : 1;
        _led_ticks = getms();
        return ;
    } else if (n < 30) {
        if (getms() - _led_ticks < 250) {
            return ;
        }
        /* Blink yellow led when battery is less than 30%. */
        drv_led_set(_led_status, _led_status, 0);
        _led_status = _led_status ? 0 : 1;
        _led_ticks = getms();
        return ;
    }

    switch (sdp_status) {
    case SdpStatusStartingUp:
    case SdpStatusCoreDisconnected:
        /* Blink cyan led when starting up. */
        if (getms() - _led_ticks < 250) {
            return ;
        }
        drv_led_set(0, _led_status, _led_status);
        _led_status = _led_status ? 0 : 1;
        _led_ticks = getms();
        break;
    case SdpStatusIdle:
        if (stalldetector_is_stalled()) {
            drv_led_set(1, 0, 0);
        } else {
            drv_led_set(0, 1, 1);
        }
        break;
    case SdpStatusStartSweep:
        if (getms() - _led_ticks < 100) {
            return ;
        }
        _led_ticks = getms();
        if (stalldetector_is_stalled()) {
            drv_led_set(1, 0, 0);
            return ;
        }
        /* Color led rolling. */
        drv_led_set((_led_status>>2)&0x01, (_led_status>>1)&0x01, _led_status&0x01);
        _led_status >>= 1;
        if (_led_status == 0) {
            _led_status = (1 << 2);
        }
        break;
    default:
        break;
    }

}

extern void on_host_request(infra_channel_desc_t * channel);
extern void alarm_heartbeat(void);

/*
 * 主循环函数
 */
static inline _s32 loop(void)
{
    
    if (net_poll_request(drv_serialchannel_getchannel())) {     //侦听来自slamcore的interchip协议报文

        on_host_request(drv_serialchannel_getchannel());        //响应来自slamcore的interchip协议报文
    }
    led_heartbeat();
    dev_heartbeat();                                            //处理各个功能模块
    alarm_heartbeat();
    return 1;
}
/*
 * 主程序函数
 */
int main(void)
{
    board_set_abort_proc(on_abort_mode);                //设置故障处理函数
    _delay_ms(100);                                     //等待上电电源稳定

    init_board();                                       //MCU低级初始化
    if (!init_dev()) {                                  //所有外设初始化
        goto _on_fail;
    }

    play_poweron();                                     //开机发声
    enable_watchdog();

#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
    DBG_OUT("Slarmware base breakout rev %d.0.\r\n", CONFIG_BREAKOUT_REV);
#endif

    while (loop()) {
        mark_watchdog();
    }

  _on_fail:
    disable_watchdog();
    board_abort_mode();
    return 0;
}
