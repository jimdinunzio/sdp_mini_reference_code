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

static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void NVIC_Configuration(void);
static void ADC_Configuration(void);

/*
 * 系统时钟设定函数，时间：1ms
 */
static inline void set_board_systick()
{
    SysTick->LOAD = SYSTICK_1MS_TICKS - 1;
    NVIC_SetPriority(SysTick_IRQn, 0);
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    softdelay_calibrate();
}
/*
 * RCC clock配置函数
 */
static void RCC_Configuration(void)
{
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(APB2PERIPH_INIT_LIST, ENABLE);
    RCC_APB1PeriphClockCmd(APB1PERIPH_INIT_LIST, ENABLE);
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 2)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif    
}
/*
 * 中断控制器配置函数
 */
static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    set_board_systick();

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannel = GET_USARTINT_IRQ(USART1_ID);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannel = GET_USARTINT_IRQ(USART2_ID);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannel = GET_USARTINT_IRQ(USART3_ID);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable line5 ~ line9 external interrupt. */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);  
}
/*
 * GPIO配置函数
 */
static void GPIO_Configuration(void)
{
    PERFORM_IO_REMAP();
#ifdef _DEBUG
    pinMode(DBG_USART_PORT, DBG_USART_TX_PIN, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    pinMode(DBG_USART_PORT, DBG_USART_RX_PIN, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
#endif
    pinMode(USART1_PORT, USART1_TX_PIN, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    pinMode(USART1_PORT, USART1_RX_PIN, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
    pinMode(USART2_PORT, USART2_TX_PIN, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    pinMode(USART2_PORT, USART2_RX_PIN, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
    pinMode(USART3_PORT, USART3_TX_PIN, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    pinMode(USART3_PORT, USART3_RX_PIN, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
#endif
}
/*
 * ADC配置函数
 */
static void ADC_Configuration(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_ResetCalibration(ADC1);
    ADC_ResetCalibration(ADC2);

    while (ADC_GetResetCalibrationStatus(ADC1)
           || ADC_GetResetCalibrationStatus(ADC2));

    ADC_StartCalibration(ADC1);
    ADC_StartCalibration(ADC2);
    while (ADC_GetCalibrationStatus(ADC1)
           || ADC_GetCalibrationStatus(ADC2));
}
/*
 * MCU低级初始化
 */
_s32 init_board()
{
    RCC_Configuration();
    GPIO_Configuration();
    NVIC_Configuration();
    ADC_Configuration();
    FLASH_Unlock();
    return 1;
}
