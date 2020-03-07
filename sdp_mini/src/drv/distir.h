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
//红外发送相关定义
#define FRONT_IR_E_PORT      GPIOC
#define FRONT_IR_E_PIN       GPIO_Pin_6 //PC6
#define FRONT_IR_E_TIMER_CH  1
#define BOTTOM_IR_E_PORT     GPIOC
#define BOTTOM_IR_E_PIN      GPIO_Pin_7 //PC7
#define BOTTOM_IR_E_TIMER_CH 2
#define IR_EMITTER_TIMER_ID  3
//红外接收相关定义
#define FRONT_IR_R1_PORT     GPIOC
#define FRONT_IR_R1_PIN      GPIO_Pin_0 //PC0 ADC123_IN10
#define FRONT_IR_R1_ADC_CH   10
#define FRONT_IR_R2_PORT     GPIOA
#define FRONT_IR_R2_PIN      GPIO_Pin_5 //PA5 ADC12_IN5
#define FRONT_IR_R2_ADC_CH   5
#define FRONT_IR_R3_PORT     GPIOA
#define FRONT_IR_R3_PIN      GPIO_Pin_7 //PA7 ADC12_IN7
#define FRONT_IR_R3_ADC_CH   7
#define FRONT_IR_R4_PORT     GPIOB
#define FRONT_IR_R4_PIN      GPIO_Pin_1 //PB1 ADC12_IN9
#define FRONT_IR_R4_ADC_CH   9
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
#define BOTTOM_IR_R1_PORT    GPIOC
#define BOTTOM_IR_R1_PIN     GPIO_Pin_2 //PC2 ADC123_IN12
#define BOTTOM_IR_R1_ADC_CH  12
#else
#define BOTTOM_IR_R1_PORT    GPIOA
#define BOTTOM_IR_R1_PIN     GPIO_Pin_0 //PA0 ADC123_IN0
#define BOTTOM_IR_R1_ADC_CH  0
#endif
#define BOTTOM_IR_R2_PORT    GPIOC
#define BOTTOM_IR_R2_PIN     GPIO_Pin_1 //PC1 ADC123_IN11
#define BOTTOM_IR_R2_ADC_CH  11
#define BOTTOM_IR_R3_PORT    GPIOC
#define BOTTOM_IR_R3_PIN     GPIO_Pin_4 //PC4 ADC123_IN14
#define BOTTOM_IR_R3_ADC_CH  14
#define BOTTOM_IR_R4_PORT    GPIOA
#define BOTTOM_IR_R4_PIN     GPIO_Pin_4 //PA4 ADC12_IN4
#define BOTTOM_IR_R4_ADC_CH  4
#define IR_SENSOR_SAMPLE_ADC     2
//红外传感器ID号定义
#define IRSENSOR_FRONT_R1_ID    0
#define IRSENSOR_FRONT_R2_ID    1
#define IRSENSOR_FRONT_R3_ID    2
#define IRSENSOR_FRONT_R4_ID    3
#define IRSENSOR_BOTTOM_R1_ID   0
#define IRSENSOR_BOTTOM_R2_ID   1
#define IRSENSOR_BOTTOM_R3_ID   2
#define IRSENSOR_BOTTOM_R4_ID   3

typedef struct _irDistance {
    _u32 frontSensorDistMmQ16[4];
    _u32 bottomSensorBitmap;
} irDistance_t;

void init_distir(void);
void heartbeat_distir(void);
const irDistance_t *get_distir_value();


