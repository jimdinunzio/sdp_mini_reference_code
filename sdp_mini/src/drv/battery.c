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
#include <math.h>

#define CONFIG_POWERSTATE_CHECK_DURATION       10       //ms
#define CONFIG_CHARGEBASE_REMOVAL_THRESHOLDTS  100      //ms

static _u32 batteryFrequency = 0;
static _u32 batterySampleFrequency = 0;
static _u32 chargeFrequency = 0;
static _u32 batteryElectricityCalibrate = 0;
static _u32 chargeCurrentCalibrate = 0;
static _u32 batteryElectricityPercentage = 0;
static _u8 chargeSound = 2;
static _u8 isChargeInserted = 0;
static _s16 chargeDetectZero = 2047;

static _s32 lastChargeCurrent = 0;

//static _u8 isDcInserted = 0;
//static _u8 dcInsertedTs = 0;

// avg filter queue for power supply sampling...
#define PWR_FILTER_QUEUE_SIZE  10
static _u16 _pwrCachedBattVoltADCVal = 0;
static _u16 _pwrFilterBattVoltQueue[PWR_FILTER_QUEUE_SIZE+2];
static _u8  _pwrFilterPos;
enum {
    BATTERY_ADC_STATE_IDLE = 0,
    BATTERY_ADC_STATE_WAITING = 1,
};
static _u8  _pwrAdcSampleState = 0;
static void _battery_sample_batteryvoltage();

#define CHARGE_FILTER_QUEUE_SIZE   10
static _u16 _chargeCachedCurrentADCVal = 0;
static _u16 _chargeFilterCurrentQueue[CHARGE_FILTER_QUEUE_SIZE+2];
static _u8  _chargeFilterPos;
enum {
    CHARGE_ADC_STATE_IDLE = 0,
    CHARGE_ADC_STATE_WAITING = 1,
};
static _u8  _chargeAdcSampleState = 0;
static void _charge_sample_chargecurrent();

enum {
  ADC_BATTERY_DETECT = 0,
  ADC_CHARGE_CURRENT = 1,
};

static _u8 _currentAdcRead = 1;

/*
 * charge ADC detection initialization function
 */
static void init_charge_current_detect(void)
{
    _chargeAdcSampleState = CHARGE_ADC_STATE_IDLE;
    _chargeFilterPos = 0;
    // preheat the adc avg-filter queue...
    _charge_sample_chargecurrent();
    while (_chargeFilterPos == 0) {
      _charge_sample_chargecurrent();
    }
    do {
      _charge_sample_chargecurrent();
    } while (_chargeFilterPos!=0);
    DBG_OUT("preheat charge current queue avg = %d.\r\n", _chargeCachedCurrentADCVal);
}

/*
 * Battery capacity ADC detection initialization function
 */
static void init_electricity_detect(void)
{
    _pwrAdcSampleState = BATTERY_ADC_STATE_IDLE;
    _pwrFilterPos = 0;
    // preheat the adc avg-filter queue...
    _battery_sample_batteryvoltage();
    while (_pwrFilterPos == 0) {
      _battery_sample_batteryvoltage();
    }
    do {
      _battery_sample_batteryvoltage();
    } while (_pwrFilterPos!=0);
    DBG_OUT("preheat battery voltage queue avg = %d.\r\n", _pwrCachedBattVoltADCVal);
}

// ACS712 outputs 2.5v for I=0, and here R1=R2=10K voltage divder divides it 
// in half to 1.25v for I=0. V per A needs to be divided in half as well.
#define ACS712_05A_V_PER_A (0.185 / 2.0)

/*
 * Get charge current function
 * Return current value, unit: mA
 */
_s32 get_charge_current(void)
{
    const _u32  ADC_LEVELS = (0x1<<ADC_RES_BIT) - 1; //4095
    // ADC_REF = 2.495
    // Vin = adc_val * ADC_REF / 4096
    
    const float ADC_TO_CHARGE_CURRENT_FACTOR = (float)HOCHARGE_DETECT_ADC_REF / ADC_LEVELS / ACS712_05A_V_PER_A;      
    const _s32  ADC_TO_CHARGE_CURRENT_FACTOR_fixQ10 = (_s32)(ADC_TO_CHARGE_CURRENT_FACTOR * 1024.0);
    return (_s32)(((_s32)_chargeCachedCurrentADCVal - chargeDetectZero) * ADC_TO_CHARGE_CURRENT_FACTOR_fixQ10) / 1024;
}

/*
 * Get battery voltage function
 * Return voltage value, unit: mV
 */
_u32 get_electricity(void)
{
    const _u32  ADC_LEVELS = (0x1<<ADC_RES_BIT) - 1; //4095
  //  ADC_REF = 2.495
  //  VBATT / BATT_DETECT_ADC_RATIO = adc_val * ADC_REF / 4096
  const float ADC_TO_BATT_VOLT_FACTOR = (BATT_DETECT_ADC_REF * BATT_DETECT_ADC_RATIO) / ADC_LEVELS;
  const _u32  ADC_TO_BATT_VOLT_FACTOR_fixQ10 = (_u32)(ADC_TO_BATT_VOLT_FACTOR * 1024.0);
  return (_pwrCachedBattVoltADCVal * ADC_TO_BATT_VOLT_FACTOR_fixQ10)>>10;
}
/*
 * Get battery capacity percentage function
 * Return percentage 0-100%
 */
_u8 get_electricitypercentage(void)
{
    return batteryElectricityPercentage;
}
/*
 * Battery charge level detection initialization function
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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = HOCHARGE_DETECT_PIN | BATT_DETECT_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(BATT_AND_CHARGE_DETECT_PORT, &GPIO_InitStructure);
}
/*
 * Battery charge status detection function
 * Return to charging or not charging state
 */
_u8 charge_detect_getstatus(void)
{
    if (isChargeInserted) {
        return ISCHARGE_CHRG;
    } else {
        return ISCHARGE_NOCHRG;
    }
}
#if 0
/*
 * Get the battery DC power plug charging status function
 */
_s8 get_dc_charge_status(void)
{
    return isDcInserted;
}
#endif
/*
 * Get the charging status function of the battery charging pile
 */
_s8 get_home_charge_status(void)
{
    return isChargeInserted;
}

static void _on_charge_adc_data_ready(_u16 adcData)
{
    _chargeCachedCurrentADCVal = add_to_avg_filter_u16(adcData, 
      _chargeFilterCurrentQueue, _chargeFilterPos, CHARGE_FILTER_QUEUE_SIZE);
    if ( (++_chargeFilterPos) >= CHARGE_FILTER_QUEUE_SIZE) {
        _chargeFilterPos = 0;
    }
}

static void _charge_sample_chargecurrent()
{
    switch (_chargeAdcSampleState) {
    case CHARGE_ADC_STATE_IDLE:
        adc_read_start(GET_ADC(HOCHARGE_ADC), HOCHARGE_DETECT_ADC_CHN);
        _chargeAdcSampleState = CHARGE_ADC_STATE_WAITING;
        break;
    case CHARGE_ADC_STATE_WAITING:
        if (adc_read_is_ready(GET_ADC(HOCHARGE_ADC))) {
            _on_charge_adc_data_ready(adc_read_final(GET_ADC(HOCHARGE_ADC)));
            _chargeAdcSampleState = CHARGE_ADC_STATE_IDLE;
        }
        break;
    }
}

static void _on_battery_adc_data_ready(_u16 adcData)
{
    _pwrCachedBattVoltADCVal = add_to_avg_filter_u16(adcData, 
      _pwrFilterBattVoltQueue, _pwrFilterPos, PWR_FILTER_QUEUE_SIZE);
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
        percent = (int)(100.0f * (-0.1700162f - (-0.005948003f / -0.3530656f) 
                                  * (1.0f - expf(0.0003530656f * currentVolt))));
    }
    return percent;
}

static void _battery_volume_update(void)
{
    _s32 percent = _battery_volume_calculate();

    if (ISCHARGE_CHRG != charge_detect_getstatus()) {
        /* Discharging. Volume is always getting down. */
        //if (percent >= batteryElectricityPercentage) {
        //    return ;
        //}
        //percent = batteryElectricityPercentage - 1;
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
 * Battery related initialization function
 * Initialize battery capacity detection
 * Initialize battery charge detection
 */
void init_battery(void)
{
    init_charge_detect();
    init_electricity_detect();
    init_charge_current_detect();
    chargeFrequency = getms();
}
/*
 * Battery related module functions
 * Judgment of charging status, etc.
 */
void heartbeat_battery(void)
{
    if (!chargeCurrentCalibrate) {
        _charge_sample_chargecurrent();

        if ((getms() - chargeFrequency) >= CHARGE_CURRENT_CALIBRATING_DURATION) {
            while (_chargeAdcSampleState != CHARGE_ADC_STATE_IDLE) { // wait until last read is finished
              _charge_sample_chargecurrent();
            }
            chargeCurrentCalibrate = true;
            chargeDetectZero = _chargeCachedCurrentADCVal;
            batteryFrequency = getms();            
            DBG_OUT("Charge current calibration done, charge detect zero = %d/4095 or %dmv current = %d.\r\n",
                    chargeDetectZero, chargeDetectZero * HOCHARGE_DETECT_ADC_REF / 4095, get_charge_current());
        }
        return ;
    }

    if (!batteryElectricityCalibrate) {
        _battery_sample_batteryvoltage();

        if ((getms() - batteryFrequency) >= BATT_VOLUME_CALIBRATING_DURATION) {
            while (_pwrAdcSampleState != BATTERY_ADC_STATE_IDLE) { // wait until last read is finished
              _battery_sample_batteryvoltage();
            }          
            batteryElectricityPercentage = _battery_volume_calculate();
            batteryElectricityCalibrate = true;
            batteryFrequency = getms();
            chargeFrequency = getms();
            batterySampleFrequency = getms();

            DBG_OUT("Battery calibration done, voltage %d, volume %d.\r\n",
                    get_electricity(), batteryElectricityPercentage);
            lastChargeCurrent = get_charge_current();

        }
        return ;
    }

    if (_currentAdcRead == ADC_CHARGE_CURRENT) {
      _charge_sample_chargecurrent();
      if (_chargeAdcSampleState == CHARGE_ADC_STATE_IDLE && 
          getms() - batterySampleFrequency >= BATT_SAMPLE_DURATION) {
        batterySampleFrequency = getms();
        _currentAdcRead = ADC_BATTERY_DETECT;  
      }
    }
    if (_currentAdcRead == ADC_BATTERY_DETECT) {
      _battery_sample_batteryvoltage();
      if (_pwrAdcSampleState == BATTERY_ADC_STATE_IDLE)
      {
        _currentAdcRead = ADC_CHARGE_CURRENT;
      }
    }

    // Check battery percentage
    if ((getms() - batteryFrequency) >= BATT_VOLUME_UPDATE_DURATION) {
        // Detect battery capacity and calculate percentage every 30 seconds
        batteryFrequency = getms();

        //Check battery capacity and calculate percentage
        _battery_volume_update();
        bool isCharging = ISCHARGE_CHRG == charge_detect_getstatus();
        DBG_OUT("%d: Battery voltage %d%%, %dmv%s.\r\n", getms(), 
          batteryElectricityPercentage, get_electricity(), isCharging ?
            " [Charging]" : "");
        if (batteryElectricityPercentage < 15 && !isCharging) {
            {
                //beep_beeper(3000, 400, chargeSound);
                if ((chargeSound += 1) >= 250) {
                    chargeSound = 250;
                }
            }
        } else {
            chargeSound = 2;
        }
    }
    
    // Check if commenced or stopped charging 
    if ((getms() - chargeFrequency) >= HOCHARGE_DETECT_UPDATE_DURATION) {
        //DBG_OUT("Charge ADC out avg: %d = %dmv\r\n", _chargeCachedCurrentADCVal, _chargeCachedCurrentADCVal * HOCHARGE_DETECT_ADC_REF / 4095 );
        //DBG_OUT("Charge current %dma.\r\n", get_charge_current());
      chargeFrequency = getms();

      _s32 current = get_charge_current();
      //DBG_OUT("%d: Volts = %d, I = %d\r\n",getms(), _chargeCachedCurrentADCVal * HOCHARGE_DETECT_ADC_REF / 4095, current);
      if (isChargeInserted) {
          //Whether it is in the charging state of the charging pile
          if (current > lastChargeCurrent + 800) { 
            //under charging pile charging: detect whether it is detached from the charging pile,
            //and change the charging state of the charging pile after pulling it out
             DBG_OUT("DETACH from charger detected current %dma.\r\n", get_charge_current());
             isChargeInserted = 0;
             beep_beeper(5000, 80, 2);
          }
      }
      else { // !isChargeInserted
        // Not under charging from the charging station: check whether it 
        // has made contact and started charging
        if (current < lastChargeCurrent - 800) { // count sequential - monotonic samples in current
          isChargeInserted = 1;
          beep_beeper(5000, 80, 2);
          DBG_OUT("ATTACH to charger detected, current %dma.\r\n", get_charge_current());
        } 
      }      
      lastChargeCurrent = current;
    }
#if 0
    if (isDcInserted) {
        //Whether it is in DC power charging state
        if (!PIN_READ(GPIOB, DCCHARGE_DETECT)) {
            //Under DC power charging: check whether the DC power is unplugged, 
            // and change the charging state of the DC power if unplugged
            if (getms() - dcInsertedTs >= CONFIG_CHARGEBASE_REMOVAL_THRESHOLDTS * 2) {
                //Anti-bounce
                isDcInserted = 0;
            }
        } else {
            dcInsertedTs = getms();
        }

    } else {
        //Not under DC power charging: detect whether the DC power is plugged in,
        // and change the charging state of the DC power when plugged in
        if (PIN_READ(GPIOB, DCCHARGE_DETECT)) {
            isDcInserted = 1;
            dcInsertedTs = getms();
        }
    }
#endif
}
