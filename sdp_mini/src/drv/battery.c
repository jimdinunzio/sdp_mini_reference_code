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
#include "battery.h"
#include "beep.h"
#include "utils/filters.h"

#define CONFIG_POWERSTATE_CHECK_DURATION       10       //单位：ms
#define CONFIG_CHARGEBASE_REMOVAL_THRESHOLDTS  100      //单位：ms
static _u32 batteryFrequency = 0;
static _u32 batteryElectricityCalibrate = 0;
static _u32 batteryElectricityPercentage = 0;
static _u8 chargeSound = 2;
static _u8 isChargeInserted = 0;
static _u8 chargeInsertedTs = 0;
static _u8 isDcInserted = 0;
static _u8 dcInsertedTs = 0;

// avg filter queue for power supply sampling...
#define PWR_FILTER_QUEUE_BIT    3
#define PWR_FILTER_QUEUE_SIZE   (0x1<<PWR_FILTER_QUEUE_BIT)
static _u16 _pwrCachedBattVoltADCVal = 0;
static _u16 _pwrFilterBattVoltQueue[PWR_FILTER_QUEUE_SIZE+2];
static _u8  _pwrFilterPos;
enum {
    BATTERY_ADC_STATE_IDLE = 0,
    BATTERY_ADC_STATE_WAITING = 1,
};
static _u8  _pwrAdcSampleState = 0;
static void _battery_sample_batteryvoltage();
/*
 * 电池容量ADC检测初始化函数
 */
static void init_electricity_detect(void)
{
    _pwrAdcSampleState = BATTERY_ADC_STATE_IDLE;
    _pwrFilterPos = 0;
    // preheat the adc avg-filter queue...
    do{
          _battery_sample_batteryvoltage();
      } while (_pwrFilterPos!=0);
}
/*
 * 获取电池电压函数
 * 返回电压值，单位：mV
 */
_u32 get_electricity(void)
{
    const _u32  ADC_LEVELS = (0x1<<ADC_RES_BIT); //4096
  //  ADC_REF = 2.495
  //  VBATT / BATT_DETECT_ADC_RATIO = adc_val * ADC_REF / 4096
  const float ADC_TO_BATT_VOLT_FACTOR = (BATT_DETECT_ADC_REF * BATT_DETECT_ADC_RATIO) / ADC_LEVELS;
  const _u32  ADC_TO_BATT_VOLT_FACTOR_fixQ10 = (_u32)(ADC_TO_BATT_VOLT_FACTOR * 1024.0);
  return (_pwrCachedBattVoltADCVal * ADC_TO_BATT_VOLT_FACTOR_fixQ10)>>10;
}
/*
 * 获取电池容量百分比函数
 * 返回百分比0-100%
 */
_u8 get_electricitypercentage(void)
{
    return batteryElectricityPercentage;
}
/*
 * 电池充电电平检测初始化函数
 */
static void init_charge_detect(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = BATT_FAULT | BATT_CHRG | BATT_READY;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = HOCHARGE_DETECT | DCCHARGE_DETECT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = BATT_DETECT_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(BATT_DETECT_PORT, &GPIO_InitStructure);
}
/*
 * 电池充电状态检测函数
 * 返回正在充电或不在充电状态
 */
_u8 charge_detect_getstatus(void)
{
    if ((isChargeInserted || isDcInserted) && (0 == GPIO_ReadInputDataBit(GPIOE, BATT_READY))) {
        return ISCHARGE_CHRG;
    } else {
        return ISCHARGE_NOCHRG;
    }
}
/*
 * 获取电池DC电源插头充电状态函数
 */
_s8 get_dc_charge_status(void)
{
    return isDcInserted;
}
/*
 * 获取电池充电桩充电状态函数
 */
_s8 get_home_charge_status(void)
{
    return isChargeInserted;
}
static void _on_battery_adc_data_ready(_u16 adcData)
{
    _pwrCachedBattVoltADCVal = add_to_avg_filter_u16(adcData, _pwrFilterBattVoltQueue, _pwrFilterPos, PWR_FILTER_QUEUE_SIZE);
    if ( (++_pwrFilterPos) >= PWR_FILTER_QUEUE_SIZE) {
        _pwrFilterPos = 0;
    }
}

static void _battery_sample_batteryvoltage()
{
    switch (_pwrAdcSampleState) {
    case BATTERY_ADC_STATE_IDLE:
        adc_read_start(GET_ADC(BATT_DETECT_ADC), BATT_DETECT_ADC_CHN);
        _pwrAdcSampleState = BATTERY_ADC_STATE_WAITING;
        break;
    case BATTERY_ADC_STATE_WAITING:
        if (adc_read_is_ready(GET_ADC(BATT_DETECT_ADC))) {
            _on_battery_adc_data_ready(adc_read_final(GET_ADC(BATT_DETECT_ADC)));
            _pwrAdcSampleState = BATTERY_ADC_STATE_IDLE;
        }
        break;
    }
}
static _s32 _battery_volume_calculate(void)
{
    _s32 percent;
    _u32 currentVolt = get_electricity();

    if (currentVolt < BATTERY_VOLTAGE_EMPTY) {
        percent = 0;
    }   else if (currentVolt > BATTERY_VOLTAGE_FULL) {
        percent = 100;
    } else {
        percent = (currentVolt - BATTERY_VOLTAGE_EMPTY) * 100 / (BATTERY_VOLTAGE_FULL - BATTERY_VOLTAGE_EMPTY);
    }
    return percent;
}

static void _battery_volume_update(void)
{
    _s32 percent = _battery_volume_calculate();

    if (ISCHARGE_CHRG != charge_detect_getstatus()) {
        /* Discharging. Volume is always getting down. */
        if (percent >= batteryElectricityPercentage) {
            return ;
        }
        percent = batteryElectricityPercentage - 1;
    } else {
        /* Charging. Volume is always getting up. */
        if (percent <= batteryElectricityPercentage) {
            return ;
        }
        percent = batteryElectricityPercentage + 1;
    }

    if (percent < 0) {
        percent = 0;
    } else if (percent > 100) {
        percent = 100;
    }
    batteryElectricityPercentage = percent;
}

/*
 * 电池相关初始化函数
 * 初始化电池容量检测
 * 初始化电池充电检测
 */
void init_battery(void)
{
    init_electricity_detect();
    init_charge_detect();
}
/*
 * 电池相关模块函数
 * 充电状态的判定等
 */
void heartbeat_battery(void)
{
    if (!batteryElectricityCalibrate) {
        _battery_sample_batteryvoltage();

        if ((getms() - batteryFrequency) >= BATT_VOLUME_CALIBRATING_DURATION) {
            batteryElectricityPercentage = _battery_volume_calculate();
            batteryElectricityCalibrate = true;
            batteryFrequency = getms();
            DBG_OUT("Battery calibration done, voltage %d, volume %d.\r\n",
                    get_electricity(), batteryElectricityPercentage);
        }
        return ;
    }

    _battery_sample_batteryvoltage();
    if ((getms() - batteryFrequency) >= BATT_VOLUME_UPDATE_DURATION) {
        //3秒检测一次电池容量及计算百分比
        batteryFrequency = getms();

        //检测电池容量及计算百分比
        _battery_volume_update();
        DBG_OUT("Battery voltage %d%%, %dmv.\r\n", batteryElectricityPercentage, get_electricity());


        if (batteryElectricityPercentage < 15 && ISCHARGE_CHRG != charge_detect_getstatus()) {
            {

//                beep_beeper(3000, 400, chargeSound);
                if ((chargeSound += 1) >= 250) {
                    chargeSound = 250;
                }
            }
        } else {
            chargeSound = 2;
        }
    }
#if 0
    if (isChargeInserted) {
        //是否处在充电桩充电状态
        if (!PIN_READ(GPIOB, HOCHARGE_DETECT)) {
            //在充电桩充电下：检测是否脱离充电桩，拔出则更改充电桩充电状态
            if (getms() - chargeInsertedTs >= CONFIG_CHARGEBASE_REMOVAL_THRESHOLDTS) {
                beep_beeper(5000, 80, 2);
                isChargeInserted = 0;
            }
        } else {
            chargeInsertedTs = getms();
        }

    } else {
        //不在充电桩充电下：检测是否脱离充电桩，插入则更改充电桩充电状态
        if (PIN_READ(GPIOB, HOCHARGE_DETECT)) {
            isChargeInserted = 1;
            beep_beeper(5000, 80, 2);
            chargeInsertedTs = getms();
        }

    }
    if (isDcInserted) {
        //是否处在DC电源充电状态
        if (!PIN_READ(GPIOB, DCCHARGE_DETECT)) {
            //在DC电源充电下：检测是否拔出DC电源，拔出则更改DC电源充电状态
            if (getms() - dcInsertedTs >= CONFIG_CHARGEBASE_REMOVAL_THRESHOLDTS * 2) {
                //防抖动
                isDcInserted = 0;
            }
        } else {
            dcInsertedTs = getms();
        }

    } else {
        //不在DC电源充电下：检测DC电源是否插入，插入则更改DC电源充电状态
        if (PIN_READ(GPIOB, DCCHARGE_DETECT)) {
            isDcInserted = 1;
            dcInsertedTs = getms();
        }
    }
#endif
}
