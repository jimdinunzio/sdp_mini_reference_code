/*
 * SlamTec Infra Runtime Public
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

#ifndef RPSTM32_HEADER_H
#define RPSTM32_HEADER_H

#define EXPAND_WRAPPER( __NEXTLEVEL__, ...)  __NEXTLEVEL__( __VA_ARGS__ )

// Helper Macros

#define _GET_NORMAL_TIMER_PERIPH(x) RCC_APB1Periph_TIM##x
#define _GET_ADC_PERIPH(x) RCC_APB2Periph_ADC##x
#define _GET_EXTINT_PIN(x) GPIO_PinSource##x
#define _GET_EXTINT_LINE(x) EXTI_Line##x

#define _GET_X_IRQ_Y(x, y) x##y##_IRQn
#define _GET_EXTINT_IRQ(x) _GET_X_IRQ_Y(EXTI, x)
#define _GET_TIMINT_IRQ(x) _GET_X_IRQ_Y(TIM, x)
#define _GET_USARTINT_IRQ(x) _GET_X_IRQ_Y(USART, x)
#define _GET_ADC_CHANNEL(x) ADC_Channel_##x

#define _GET_I2C_EVT_IRQ(x) I2C##x##_EV_IRQn
#define _GET_I2C_ER_IRQ(x) I2C##x##_ER_IRQn

#define _GET_USART(x) USART##x
#define _GET_TIM(x)   TIM##x
#define _GET_ADC(x)   ADC##x
#define _GET_SPI(x)   SPI##x
#define _GET_I2C(x)   I2C##x

#define GET_NORMAL_TIMER_PERIPH(x)  EXPAND_WRAPPER(_GET_NORMAL_TIMER_PERIPH,x)
#define GET_ADC_PERIPH(x)  EXPAND_WRAPPER(_GET_ADC_PERIPH,x)
#define GET_EXTINT_PIN(x)  EXPAND_WRAPPER(_GET_EXTINT_PIN,x)
#define GET_EXTINT_LINE(x)  EXPAND_WRAPPER(_GET_EXTINT_LINE,x)
#define GET_EXTINT_IRQ(x)  EXPAND_WRAPPER(_GET_EXTINT_IRQ,x)
#define GET_TIMINT_IRQ(x)  EXPAND_WRAPPER(_GET_TIMINT_IRQ,x)
#define GET_USARTINT_IRQ(x)  EXPAND_WRAPPER(_GET_USARTINT_IRQ,x)
#define GET_ADC_CHANNEL(x)  EXPAND_WRAPPER(_GET_ADC_CHANNEL,x)
#define GET_I2C_EVT_IRQ(x)  EXPAND_WRAPPER(_GET_I2C_EVT_IRQ,x)
#define GET_I2C_ER_IRQ(x)  EXPAND_WRAPPER(_GET_I2C_ER_IRQ,x)


#define GET_USART(x)  EXPAND_WRAPPER(_GET_USART,x)
#define GET_TIM(x)    EXPAND_WRAPPER(_GET_TIM,x)
#define GET_ADC(x)    EXPAND_WRAPPER(_GET_ADC,x)
#define GET_SPI(x)    EXPAND_WRAPPER(_GET_SPI,x)
#define GET_I2C(x)    EXPAND_WRAPPER(_GET_I2C,x)

#if defined(__CC_ARM) || defined(__ICCARM__)
#define WEAK_SYMBOL_DEF __weak
#else
#error "The WEAK_SYMBOL_DEF should be defined for this compiler"
#endif

// ST-FWlib
#include "stm32f10x.h"

// Lib-c
#include <stdlib.h>
#include <string.h>

// IAR only
#include <stdbool.h>

// Infra Common
#define _CPU_ENDIAN_SMALL

#include "hal/types.h"
#include "hal/util.h"

#define MIN(a,b) ( ((a) > (b))?(b):(a))
#define MAX(a,b) ( ((a) < (b))?(b):(a))

// Board Def
#include "boarddef.h"

// Board HAL
#include "boardfunc.h"



#endif // #ifndef RPSTM32_HEADER_H