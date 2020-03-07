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

#define SYSTICK_1MS_TICKS    (CPU_FREQ/1000)

#define FLASH_ADDR_BASE 0x08000000
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
  #define FLASH_PAGE_SIZE    ((uint16_t)0x800)
#else
  #define FLASH_PAGE_SIZE    ((uint16_t)0x400)
#endif

#define STM32_SELFID_LOCATION 0x1FFFF7E8
#define STM32_SELFID_LEN     3
#define STM32_FSIZE_LOCATION  0x1FFFF7E0


/*
* ADC相关
*/
#define ADC_INNER_TEMPER_CH     16
#define ADC_INNER_TEMPER_PORT   1

#define ADC_INNER_TEMPER_2V5    1.43
#define ADC_INNER_TEMPER_SLOPE  4.3
#define ADC_INNER_VOLT_CH     17
#define ADC_INNER_VOLT_PORT   1

#define ADC_RES_BIT  12
#define ADC_REFINT_VOLT 1.2

#define cli __disable_irq
#define sei __enable_irq

static inline _u32 enter_critical_section(void)
{
    _u32 context=__get_PRIMASK();
    cli();
    return context;
}

static inline void leave_critical_section(_u32 context)
{
    __set_PRIMASK(context);
}


typedef void (* abort_proc_t ) (void);
void board_abort_mode();
void board_set_abort_proc( abort_proc_t proc);
uint32_t getms();
uint64_t getus();
void delay(uint32_t ms);
void delay_alert(uint32_t ms);
extern void _delay_us(volatile uint32_t us);
static inline void _delay_ms(uint32_t ms)
{
  while(ms--)
  {
     _delay_us(1000);
  }
}
void alert(void);
void clear_alert(void);
int is_alert(void);

static inline uint32_t * get_self_id()
{
  return (uint32_t *)(STM32_SELFID_LOCATION);
}

static inline uint32_t board_get_flash_size()
{
  return ((uint32_t)*(uint16_t *)(STM32_FSIZE_LOCATION))<<10;
}
/*
* GPIO相关
*/
#define HIGH 1
#define LOW  0

#define _PIN_SET_1(port, pin) port->BSRR = pin
#define _PIN_SET_0(port, pin) port->BRR = pin
#define _PIN_SET(port, pin, val) _PIN_SET_##val(port, pin)

#define PIN_SET(port, pin, val) EXPAND_WRAPPER(_PIN_SET, port, pin, val)
#define PIN_READ(port, pin) ((port)->IDR & pin)

#define pinSet   GPIO_WriteBit
#define pinRead  GPIO_ReadInputDataBit


#define _INIT_OC_NOPRELOAD_FOR(channel, timer)  \
  TIM_OC##channel##Init(TIM##timer, &TIM_OCInitStructure); \
  TIM_OC##channel##PreloadConfig(TIM##timer, TIM_OCPreload_Disable)

#define INIT_OC_NOPRELOAD_FOR( channel, timer) \
  EXPAND_WRAPPER(_INIT_OC_NOPRELOAD_FOR, channel, timer)


#define _INIT_OC_FOR(channel, timer)  \
  TIM_OC##channel##Init(TIM##timer, &TIM_OCInitStructure); \
  TIM_OC##channel##PreloadConfig(TIM##timer, TIM_OCPreload_Enable)

#define INIT_OC_FOR( channel, timer) \
  EXPAND_WRAPPER(_INIT_OC_FOR, channel, timer)


#define _RAW_PWM_SET(channel, timer, val) \
  TIM##timer->CCR##channel = val

#define RAW_PWM_SET(channel, timer, val) \
  EXPAND_WRAPPER(_RAW_PWM_SET, channel, timer, val)


#define _RAW_PWM_GET(channel, timer) \
  TIM##timer->CCR##channel

#define RAW_PWM_GET(channel, timer) \
  EXPAND_WRAPPER(_RAW_PWM_GET, channel, timer)


static inline void pinMode(GPIO_TypeDef* GPIOx, uint16_t pin,
                           GPIOMode_TypeDef mode, GPIOSpeed_TypeDef GPIO_Speed)
{
  GPIO_InitTypeDef init = { pin, GPIO_Speed, mode};
  GPIO_Init(GPIOx, &init);
}

/*
* ADC相关
*/
void adc_read_start(ADC_TypeDef * adc_dev, uint8_t ADC_Channel);
uint16_t adc_read_final(ADC_TypeDef * adc_dev);
uint16_t adc_read_wait(ADC_TypeDef * adc_dev);
static inline int adc_read_is_ready(ADC_TypeDef * adc_dev)
{
  return (adc_dev->SR & ADC_FLAG_EOC);
}
uint16_t board_get_temperature();
#include "softdelay.h"
#include "usart.h"
