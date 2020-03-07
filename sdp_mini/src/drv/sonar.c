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
#include "utils/filters.h"
#include "drv/gpio.h"
#include "drv/led.h"
#include "sonar.h"

#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
/**
 @defgroup  sonar ultrasonic sonar module.
 @addtogroup drivers
 @{

  Resource consumption: GPIOE 5, 7 ~ 12, 15.
  EXTI line 5,7,8,9 with interrupt.
  TIM6 as counter timer.
 */

#define sonar_dbg(fmt, args...)     DBG_OUT("[sonar]: " fmt, ##args)

#define EXTI_LINE(l)        (EXTI_Line0 << (l))

#define SONAR_TIMER         TIM6

static uint8_t g_sonar_ch = 0;  /**< Active sonar channel index. */

/**
 @brief Global sonar channel descriptors.
 */
static sonar_channel_t g_sonar[CONFIG_SONAR_CHANNEL_NUM];

/**
 @brief Global sonar channel configurations.
 */
static const sonar_cfg_t g_sonar_cfg[CONFIG_SONAR_CHANNEL_NUM] = {
    {SONAR_TRIG1_PORT, SONAR_TRIG1_PIN, SONAR_ECHO1_PORT, SONAR_ECHO1_PIN, 5},
    {SONAR_TRIG2_PORT, SONAR_TRIG2_PIN, SONAR_ECHO2_PORT, SONAR_ECHO2_PIN, 7},
    {SONAR_TRIG3_PORT, SONAR_TRIG3_PIN, SONAR_ECHO3_PORT, SONAR_ECHO3_PIN, 8},
    {SONAR_TRIG4_PORT, SONAR_TRIG4_PIN, SONAR_ECHO4_PORT, SONAR_ECHO4_PIN, 9},
};

/**
 @brief Sonar echo read.
 @param ch    - sonar channel index, 0 ~ CONFIG_SONAR_CHANNEL_NUM.
 @return none.
 */
static inline uint8_t SONAR_ECHO(uint8_t ch)
{
    return GPIO_ReadInputDataBit(g_sonar_cfg[ch].echo_port, g_sonar_cfg[ch].echo_pin);
}

/**
 @brief Sonar trigger function.
 @param ch    - sonar channel index, 1 ~ CONFIG_SONAR_CHANNEL_NUM-1.
 @param level - sonar trigger level.
 @return none.
 */
static inline void SONAR_TRIG(uint8_t ch, uint8_t level)
{
    if (level == HIGH) {
        GPIO_SetBits(g_sonar_cfg[ch].trig_port, g_sonar_cfg[ch].trig_pin);
    } else {
        GPIO_ResetBits(g_sonar_cfg[ch].trig_port, g_sonar_cfg[ch].trig_pin);
    }
}

/**
 @brief Trigger a sonar channel.
 @param ch - channel number to be triggered, 1 ~ CONFIG_SONAR_CHANNEL_NUM.
 @return none.

 */
static void sonar_trigger(uint8_t ch)
{
    EXTI_InitTypeDef exti;

    if (ch >= CONFIG_SONAR_CHANNEL_NUM) {
        return ;
    }

    /* Send trigger wave. */
    SONAR_TRIG(ch, HIGH);
    _delay_us(20);
    SONAR_TRIG(ch, LOW);

    /* Prepare for echo rising edge interrupt. */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, g_sonar_cfg[ch].exti_line);

    /* Configure EXTI lines. */
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti.EXTI_LineCmd = ENABLE;
    exti.EXTI_Line    = EXTI_LINE(g_sonar_cfg[ch].exti_line);
    EXTI_Init(&exti);
    return ;
}

/**
 @brief Shutdown a sonar channel.
 @param ch - channel number to be shutdown, 0 ~ CONFIG_SONAR_CHANNEL_NUM.
 @return none.

 */
static void sonar_shutdown(uint8_t ch)
{
    EXTI_InitTypeDef exti;

    if (ch >= CONFIG_SONAR_CHANNEL_NUM) {
        return ;
    }

    SONAR_TRIG(ch, LOW);

    /* Configure EXTI lines. */
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
    exti.EXTI_LineCmd = DISABLE;
    exti.EXTI_Line    = EXTI_LINE(g_sonar_cfg[ch].exti_line);
    EXTI_Init(&exti);
    return ;
}

/**
 @brief Calculate sonar distance by sample value.
 @param ch - sonar channel, 0 ~ CONFIG_SONAR_CHANNEL_NUM
 @return none.

 This distance is in mm. 
 */
static void sonar_distance(uint8_t ch)
{
    uint8_t  i;
    uint32_t avg;
    float    d;

    if (ch >= CONFIG_SONAR_CHANNEL_NUM) {
        return ;
    }

    avg = 0;
    for (i = 0; i < g_sonar[ch].cnt; i++) {
        avg += g_sonar[g_sonar_ch].sample[i];
    }
    if (i > 0) {
        avg /= i;
    } else {
        avg = 0;
    }

    d = CONFIG_SONAR_COE_A + CONFIG_SONAR_COE_B * CONFIG_SONAR_COE_T;
    d *= avg / 2 / 1000.0;
#ifdef CONFIG_SONAR_DISTANCE_Q16
    g_sonar[g_sonar_ch].distance = (uint32_t)FP_Q16(d);
#else
    g_sonar[g_sonar_ch].distance = (uint32_t)d;
#endif
}

/**
 @brief external line interrupt handler.
 @param none.
 @return none.
 */
void EXTI9_5_IRQHandler(void)
{
    uint8_t  id;
    uint8_t  status;
    uint32_t line;
    TIM_TimeBaseInitTypeDef tim_base;

    /* Get EXTI for the active channel. */
    line = EXTI_LINE(g_sonar_cfg[g_sonar_ch].exti_line);
    if (EXTI_GetITStatus(line) != RESET) {
        EXTI_ClearITPendingBit(line);

        status = SONAR_ECHO(g_sonar_ch);
        if (status == HIGH) {
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
            /* Start counter timer. */
            tim_base.TIM_Period = 60000;
            tim_base.TIM_Prescaler = (SYSTICK_1MS_TICKS/1000-1);
            tim_base.TIM_ClockDivision = TIM_CKD_DIV1;
            tim_base.TIM_CounterMode = TIM_CounterMode_Up;

            TIM_TimeBaseInit(SONAR_TIMER, &tim_base);
            TIM_SetCounter(SONAR_TIMER, 0);
            TIM_Cmd(SONAR_TIMER, ENABLE);
        } else {
            /* Stop counter timer. */
            TIM_Cmd(SONAR_TIMER, DISABLE);

            id = g_sonar[g_sonar_ch].id;
            if (id >= CONFIG_SONAR_SAMPLE_SIZE) {
                id = 0;
            }
            g_sonar[g_sonar_ch].sample[id++] = TIM_GetCounter(SONAR_TIMER);
            g_sonar[g_sonar_ch].state++;    /* Move to next state. */
            g_sonar[g_sonar_ch].id = id;
            if (g_sonar[g_sonar_ch].cnt < CONFIG_SONAR_SAMPLE_SIZE) {
                g_sonar[g_sonar_ch].cnt++;
            }
        }
    }
}

/**
 @brief ultrasonic sonar module heartbeat.
 @param None.
 @return None.
            ___                                                         ___
 TRIG   ___|   |_______________________________________________________|   |____
            >10us _   _   _   _   _   _   _   _          at least 10ms wait
 SENSOR _________| |_| |_| |_| |_| |_| |_| |_| |________________________________
                      Send 8 40KHz wave
                                                   ________
 ECHO   __________________________________________|        |____________________
                                                  10us ~ 18ms
 */
void sonar_heartbeat(void)
{
    if (g_sonar_ch >= CONFIG_SONAR_CHANNEL_NUM) {
        g_sonar_ch = 0;
        g_sonar[g_sonar_ch].state = SONAR_INIT;
    }

    switch (g_sonar[g_sonar_ch].state) {
    case SONAR_INIT:
        sonar_trigger(g_sonar_ch);
        g_sonar[g_sonar_ch].state++;    /* Move to next state. */
        g_sonar[g_sonar_ch].ticks = getms();
        break;
    case SONAR_MEASURE: /* Wait until measurement done. */
        if (getms() - g_sonar[g_sonar_ch].ticks > CONFIG_SONAR_TIMEOUT_MS) {
            /* Timeout. Abort measurement and move to next channel. */
            TIM_Cmd(SONAR_TIMER, DISABLE);
            sonar_shutdown(g_sonar_ch);
            g_sonar[g_sonar_ch].ticks = getms();
            g_sonar[g_sonar_ch].state = SONAR_IDLE;
			memset(g_sonar[g_sonar_ch].sample, 0, sizeof(g_sonar[g_sonar_ch].sample));
            g_sonar_ch++;
            if (g_sonar_ch >= CONFIG_SONAR_CHANNEL_NUM) {
                g_sonar_ch = 0;
            }
            drv_led_set(0, 0, 0);
        }
        break;
    case SONAR_DONE:    /* Measurement is done. */
        if (g_sonar[g_sonar_ch].cnt == 0) {
            break;
        }
        if (g_sonar[g_sonar_ch].cnt >= CONFIG_SONAR_SAMPLE_SIZE) {
            g_sonar[g_sonar_ch].cnt = CONFIG_SONAR_SAMPLE_SIZE;
        }
        sonar_distance(g_sonar_ch);
        g_sonar[g_sonar_ch].state++;
        /* Blink the led by channel. */
        drv_led_set((g_sonar_ch%3)==0?0x10:0, (g_sonar_ch%3)==1?0x10:0, (g_sonar_ch%3)==2?0x10:0);
        sonar_dbg("ch %d, distance %d\r\n", g_sonar_ch, g_sonar[g_sonar_ch].distance);
        break;
    case SONAR_EXIT:    /* Channel measurement is done. Move to next. */
        sonar_shutdown(g_sonar_ch);
        g_sonar[g_sonar_ch].ticks = getms();
        g_sonar[g_sonar_ch].state = SONAR_IDLE;
        g_sonar_ch++;
        if (g_sonar_ch >= CONFIG_SONAR_CHANNEL_NUM) {
            g_sonar_ch = 0;
        }
        break;
    default:
        g_sonar[g_sonar_ch].state = SONAR_INIT;
        break;
    }

    return ;
}

/**
 @brief Get Sonar channel distance measurement value.
 @param ch - Sonar channel number, 1 ~ CONFIG_SONAR_CHANNEL_NUM.
 @return return disatance measured in mm.
 */
uint32_t sonar_get(uint8_t ch)
{
    if (ch >= CONFIG_SONAR_CHANNEL_NUM) {
        return 0;
    }

    sonar_distance(ch);
    return g_sonar[ch].distance;
}

/**
 @brief Initialize ultrasonic sonar module.
 @param None.
 @return None.
 */
void sonar_init(void)
{
    uint8_t ch;

    /* These pins are pull up by default. So pull down them. */
    for (ch = 0; ch < CONFIG_SONAR_CHANNEL_NUM; ch++) {
        pinMode(g_sonar_cfg[ch].echo_port, g_sonar_cfg[ch].echo_pin,
                GPIO_Mode_IPD, GPIO_Speed_10MHz);
        pinMode(g_sonar_cfg[ch].trig_port, g_sonar_cfg[ch].trig_pin,
                GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
        SONAR_TRIG(ch, 0);
    }

    /* Enable line5 ~ line9 external interrupt. */

    memset(g_sonar, 0, sizeof(g_sonar));
}

/**
 @brief Clean ultrasonic sonar module.
 @param None.
 @return None.
 */
void sonar_exit(void)
{
    uint8_t ch;

    for (ch = 0; ch < CONFIG_SONAR_CHANNEL_NUM; ch++) { 
        SONAR_TRIG(ch, 0);
    }

    /* Disable line5 ~ line9 external interrupt. */

    memset(g_sonar, 0, sizeof(g_sonar));
}

/** @} */
#endif
