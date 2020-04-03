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

#include "common/common.h"

static void _default_abort_proc();

static volatile uint8_t alert_flg = 0;

static abort_proc_t _abort_proc = _default_abort_proc;

static volatile uint32_t _systick_val = 0;

static void _default_abort_proc()
{
    while(1);
}

/*
 * 系统定时器节拍中断
 * 节拍:1ms
 */
void SysTick_Handler(void)
{
    _systick_val ++;
}

void alert(void)
{
    alert_flg = 1;
}

int is_alert(void)
{
    return alert_flg;
}


void clear_alert(void)
{
    alert_flg = 0;
}

void board_set_abort_proc( abort_proc_t proc)
{
    _abort_proc = proc;
}

void board_abort_mode()
{
    cli();
    _abort_proc();
}

/*
 * 获取毫秒总累计数
 * 单位:ms
 */
uint32_t getms()
{
    return _systick_val;
}


#define SYSTICK_uS_PER_TICK      1000L/SYSTICK_1MS_TICKS
/*
 * Get the total number of milliseconds
 * Unit: ms
 */
uint64_t getus()
{
    register _u32 cached_ms, cached_tick;

    _u32 context = enter_critical_section();

    cached_ms = _systick_val;
    cached_tick = SysTick->VAL;

    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {
        ++cached_ms;
        cached_tick = SysTick->VAL;
    }

    leave_critical_section(context);

    return (_u64)cached_ms*1000 + ((SYSTICK_1MS_TICKS-1 - cached_tick)*SYSTICK_uS_PER_TICK);

}

/*
 * 不可打断的延时函数
 * 延时单位:ms
 */
void delay(uint32_t ms)
{
    uint32_t targettime = getms() + ms;

    while( getms() < targettime);
}
/*
 * 可打断的延时函数
 * 延时单位:ms
 */
void delay_alert(uint32_t ms)
{
    uint32_t targettime = getms() + ms;
    while( getms() < targettime && (!alert_flg));
    clear_alert();
}

/*
 * ADC采样启动函数
 */
void adc_read_start(ADC_TypeDef * adc_dev, uint8_t ADC_Channel)
{

    ADC_RegularChannelConfig(adc_dev, ADC_Channel, 1,
                             ADC_Channel==ADC_INNER_TEMPER_CH? ADC_SampleTime_239Cycles5:
                                 ADC_SampleTime_55Cycles5);

    ADC_Cmd(adc_dev, ENABLE);
}


/*
 * 读取ADC采样值函数
 */
uint16_t adc_read_final(ADC_TypeDef * adc_dev)
{
    adc_dev->SR = ~(uint32_t)ADC_FLAG_EOC;
    return adc_dev->DR;
}
/*
 * 等待ADC采样完成函数
 */
uint16_t adc_read_wait(ADC_TypeDef * adc_dev)
{
    while(!adc_read_is_ready(adc_dev));
    return adc_read_final(adc_dev);
}

/*
 * 获得芯片温度函数
 * 预留
 */
uint16_t board_get_temperature()
{
    uint16_t adc_raw;
    adc_read_start( GET_ADC(ADC_INNER_TEMPER_PORT),
                   GET_ADC_CHANNEL(ADC_INNER_TEMPER_CH));
    adc_raw = adc_read_wait(GET_ADC(ADC_INNER_TEMPER_PORT));
    return ( (_s32)(ADC_INNER_TEMPER_2V5*10000)
            -  (adc_raw*((_s32)(ADC_REF_VOLT*10000)))/ (0x1<<ADC_RES_BIT) )
        / ((_s32)ADC_INNER_TEMPER_SLOPE) + 250;
}
