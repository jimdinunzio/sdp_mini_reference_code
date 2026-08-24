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

// --- Home-charger detection (ACS712ELCTR-05B, in series with the charger leg) ---
// The sensor carries charger current only - the load current does not pass through it.
// So this is a clean two-state signal, not an edge to chase. Measured on this board with
// the LattePanda and Jetson running:
//     disconnected  ~0 mA    (spread roughly +/-50 mA once the zero is trimmed)
//     connected     ~1790 mA
// An absolute threshold with hysteresis is all that is needed.
//
// The ACS712's zero-current output is VCC/2 (ratiometric). VCC is a regulated 5.01V, so
// VIOUT sits at 2505mV at 0A, and the 1/2 divider halves it to 1252.5mV at PC3.
// Confirmed by a clean capture: disconnected reads ~1257mV (raw ~2065), i.e. ratio 0.502.
// (0.185 V/A is the Allegro ACS712ELCTR-05B figure; the 200mV/A on sales listings is wrong.)
#define ACS712_05A_V_PER_A          (0.185f / 2.0f)

// The 0A reference is self-calibrated at boot (see heartbeat_battery). This is only the
// power-on default, used until that runs. Theoretical (VCC/2)/2:
//     1252.5mV * 4095 / 2495 = 2056 counts   (measured disconnected: ~2065, sensor offset)
// NOTE: boot self-zero assumes the robot is NOT on the charger at power-up. It never is
// (the switch is unreachable while docked). Booting on the charger would calibrate the
// charge current away as "0A" and break detection.
#define CHARGE_DETECT_ZERO_ADC      2056

// Disconnected sits at 0mA with a spread of about +/-50mA. The charger is CC/CV, so the
// connected current is NOT fixed: ~1790mA into a low pack, tapering toward zero as it
// approaches full. So the attach threshold is set well below the CC current - low enough
// to catch a partly-tapered charge, still far above the disconnected noise. Detach is
// lower again, giving hysteresis so a taper cannot chatter the state.
//
// KNOWN LIMIT: in the final CV tail the charge current genuinely falls to ~0, which is
// indistinguishable from being undocked. A full battery on the dock will therefore read
// "not charging". Current sensing cannot resolve that case; only a charger-presence
// signal (status pin, or bus voltage pinned at the CV setpoint) could.
// Placement rules, given disconnected spans -66..+125mA (191mA peak-to-peak) and a CC
// charge is ~1790mA:
//   - DETACH must sit above the disconnected noise peak (125mA), with margin.
//   - ATTACH must sit above DETACH by MORE than the noise peak-to-peak, otherwise noise
//     alone can cross both lines while the current drifts through the band during taper.
//   - ATTACH must stay below the weakest real charge current we ever expect to see.
// 300/700 gives 175mA of clearance under DETACH and a 400mA hysteresis band (~2.1x the
// noise). Revisit ATTACH once the charge current at a realistic docking SoC is measured -
// it must stay comfortably below that number.
#define CHARGE_ATTACH_THRESHOLD_MA  700
#define CHARGE_DETACH_THRESHOLD_MA  300
#define CHARGE_DEBOUNCE_SAMPLES     2   // x HOCHARGE_DETECT_UPDATE_DURATION

static _u32 batteryFrequency = 0;
static _u32 batterySampleFrequency = 0;
static _u32 chargeFrequency = 0;
static _u32 batteryElectricityCalibrate = 0;
static _u32 chargeCurrentCalibrate = 0;
static _u32 batteryElectricityPercentage = 0;
static _u8 chargeSound = 2;
static _u8 isChargeInserted = 0;
static _s16 chargeDetectZero = CHARGE_DETECT_ZERO_ADC;
static _u8  chargeDebounceCnt = 0;

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

// At a BATT_VOLUME_UPDATE_DURATION cadence of 30s, alpha 0.10 is roughly a 5-minute time
// constant. This is the smoothing that keeps load sag off the gauge.
#define EMA_ALPHA 0.10f
static float filteredVoltage = 0.0f;  // persistent across calls

// Was 500mV, to pull the PA6 reading (nominal 11:1 ratio, never calibrated) up to what the
// LED volt display showed. That fudge was tied to the old ratio, so it goes away with it.
// Zero is the honest value here: the remaining error lives in BATT_DETECT_ADC_RATIO, which
// is provisional (see battery.h), and stacking a second correction on top would only hide
// which one is wrong. Leave this at 0 and fix the ratio once PA0 taps the raw pack.
#define BATT_VOLT_BIAS  0

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
  return ((_pwrCachedBattVoltADCVal * ADC_TO_BATT_VOLT_FACTOR_fixQ10)>>10) + BATT_VOLT_BIAS;
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

    // Battery voltage sense is on PA0 (ADC123_IN0), where the battery divider is wired.
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = BATT_DETECT_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(BATT_AND_CHARGE_DETECT_PORT, &GPIO_InitStructure);

    // Charge current sense is on PC3 (ADC123_IN13), where the ACS712 divider is wired.
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = HOCHARGE_DETECT_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(HOCHARGE_DETECT_PORT, &GPIO_InitStructure);
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

// Standard 18650 Li-ion open-circuit discharge curve, scaled x3 for the 3S4P pack
// (TalentCell PB120B1: 12.6V full = 4.20V/cell, 9.0V empty = 3.00V/cell). Parallel groups
// do not change the voltage, only the capacity, so the per-cell curve applies as-is.
//
// A quadratic cannot represent this shape: Li-ion is flat from 90% down to 20% and then
// falls off a cliff, so the table is dense through the knee and coarse across the plateau.
// Must stay ordered from full to empty - the lookup below walks it in that direction.
static const struct {
    _u16 mv;
    _u8  percent;
} BATT_SOC_CURVE[] = {
    {12600, 100},   // 4.20V/cell
    {12180,  90},   // 4.06
    {11940,  80},   // 3.98
    {11760,  70},   // 3.92
    {11610,  60},   // 3.87
    {11460,  50},   // 3.82
    {11370,  40},   // 3.79
    {11250,  30},   // 3.75
    {11100,  20},   // 3.70
    {10800,  10},   // 3.60
    {10500,   5},   // 3.50
    { 9000,   0},   // 3.00 - knee; below here the pack is done
};
#define BATT_SOC_CURVE_POINTS (sizeof(BATT_SOC_CURVE) / sizeof(BATT_SOC_CURVE[0]))

static _s32 _battery_volume_calculate(void)
{
    _s32 percent;
    _u32 currentVolt = get_electricity(); // Already averaged 3s samples
    _u32 i;

    // Initialize EMA on first run
    if (filteredVoltage == 0.0f)
        filteredVoltage = (float)currentVolt;
    else
        filteredVoltage = EMA_ALPHA * (float)currentVolt + (1.0f - EMA_ALPHA)
          * filteredVoltage;

   if (filteredVoltage >= BATTERY_VOLTAGE_FULL) {
        percent = 100;
   } else if (filteredVoltage <= BATTERY_VOLTAGE_EMPTY) {
        percent = 0;
   } else {
        _s32 mv = (_s32)filteredVoltage;

        // Walk down to the first point at or below mv, then interpolate linearly between
        // that point and the one above it. i >= 1 here: the mv >= FULL case is handled above.
        for (i = 1; i < BATT_SOC_CURVE_POINTS; ++i) {
            if (mv >= (_s32)BATT_SOC_CURVE[i].mv) break;
        }

        percent = BATT_SOC_CURVE[i].percent
                + (mv - (_s32)BATT_SOC_CURVE[i].mv)
                  * (BATT_SOC_CURVE[i-1].percent - BATT_SOC_CURVE[i].percent)
                  / ((_s32)BATT_SOC_CURVE[i-1].mv - (_s32)BATT_SOC_CURVE[i].mv);

        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;
   }

    return percent;
}
// Phone-style gauge: the displayed number moves at most one point per update, and it only
// moves in the direction the pack is actually going - down while discharging, up while
// charging. It never jumps and never reverses mid-discharge.
//
// This replaces the old voltage-watermark + 750mV recovery scheme. That was a second ratchet,
// on voltage, layered under this one, and the recovery branch was the only path that could
// push the displayed percentage back UP during a discharge - the exact bounce we do not want.
// The ratchet below already delivers monotonicity on its own, so the watermark bought nothing
// the EMA in _battery_volume_calculate was not already doing.
//
// Consequence worth knowing: there is no resync path any more. If the curve is wrong the gauge
// converges at 1 point per BATT_VOLUME_UPDATE_DURATION (2%/min) rather than snapping. That is
// the trade for never bouncing, and it is why the curve needs to be fitted to loaded voltage.
//
// The deadband is what stops the gauge nibbling itself down (the original SLAMTEC behaviour):
// a strict "step whenever the curve reads lower" ratchets away a point on every transient sag,
// and monotonicity means it never comes back. Requiring the curve to disagree by BATT_SOC_
// DEADBAND points before stepping ignores sag-sized noise while still tracking a real decline.
// Cost: the gauge lags the curve by up to DEADBAND points, hence the endpoint escapes below -
// without them it would stall short of 0% and 100% and never arrive.
#define BATT_SOC_DEADBAND 3

static void _battery_volume_update(void)
{
    _u8 currentChargeStatus = charge_detect_getstatus();
    _s32 percent = _battery_volume_calculate();
    _s32 shown = (_s32)batteryElectricityPercentage;

    if (currentChargeStatus == ISCHARGE_CHRG) {
        if (shown < 100 && (percent >= shown + BATT_SOC_DEADBAND || percent == 100)) {
            batteryElectricityPercentage++;
        }
    } else {
        if (shown > 0 && (percent <= shown - BATT_SOC_DEADBAND || percent == 0)) {
            batteryElectricityPercentage--;
        }
    }
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
            // Self-zero: the robot is never on the charger at power-up, so whatever the
            // sensor reads now is 0A. This trims out the ACS712's part-to-part offset.
            chargeDetectZero = _chargeCachedCurrentADCVal;
            batteryFrequency = getms();
            DBG_OUT("Charge sense settled, zero = %d/4095 or %dmv, current = %dma.\r\n",
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
        // %dmv is the raw averaged terminal reading, untouched by the gauge smoothing - this
        // is the column to capture for a discharge-curve fit.
        DBG_OUT("%d: Battery voltage %d%%, %dmv ema=%d%s.\r\n", getms(),
          batteryElectricityPercentage, get_electricity(), (int)filteredVoltage,
          isCharging ? " [Charging]" : "");
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
        // raw ADC included so a "no change on connect" can be attributed: if raw does not
        // move when the charger contacts, no current is flowing through the sensor and the
        // cause is physical (full pack / no contact), not the scaling math.
//        DBG_OUT("Charge raw=%d (zero=%d) %dmv -> %dma.\r\n",
//                _chargeCachedCurrentADCVal, chargeDetectZero,
//                _chargeCachedCurrentADCVal * HOCHARGE_DETECT_ADC_REF / 4095,
//                get_charge_current());
      chargeFrequency = getms();

      _s32 current = get_charge_current();

      // Absolute level with hysteresis, not a delta. The sensor sees charger current
      // only, so charging reads high and disconnected reads ~0. Keying off the level
      // rather than an edge also means the state settles correctly when the robot powers
      // up already sitting on the charger.
      if (!isChargeInserted) {
        if (current > CHARGE_ATTACH_THRESHOLD_MA) {
          if (++chargeDebounceCnt >= CHARGE_DEBOUNCE_SAMPLES) {
            isChargeInserted = 1;
            chargeDebounceCnt = 0;
            beep_beeper(5000, 80, 2);
            DBG_OUT("ATTACH to charger detected, current %dma.\r\n", current);
          }
        } else {
          chargeDebounceCnt = 0;
        }
      } else {
        if (current < CHARGE_DETACH_THRESHOLD_MA) {
          if (++chargeDebounceCnt >= CHARGE_DEBOUNCE_SAMPLES) {
            isChargeInserted = 0;
            chargeDebounceCnt = 0;
            // 2000Hz, deliberately distinct: 3000Hz is the bumper and 4000Hz is both the
            // motor-stall and on-ground beeps, so a docking event stays identifiable by ear.
            beep_beeper(2000, 80, 2);
            DBG_OUT("DETACH from charger detected, current %dma.\r\n", current);
          }
        } else {
          chargeDebounceCnt = 0;
        }
      }
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
