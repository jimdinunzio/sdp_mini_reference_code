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
#include "common/common.h"


#define DCCHARGE_DETECT         GPIO_Pin_11
#define BATT_FAULT              GPIO_Pin_4
#define BATT_CHRG               GPIO_Pin_3
#define BATT_READY              GPIO_Pin_2

#define BATT_AND_CHARGE_DETECT_PORT        GPIOA

// The ACS712 divider is physically wired to PC3
#define HOCHARGE_DETECT_PIN     GPIO_Pin_3 //PC3 ADC123_IN13
#define HOCHARGE_DETECT_PORT    GPIOC
#define HOCHARGE_ADC            1
#define HOCHARGE_DETECT_ADC_CHN 13
#define HOCHARGE_DETECT_ADC_REF 2495

#define CHARGE_CURRENT_CALIBRATING_DURATION 3000
// tick x CHARGE_DEBOUNCE_SAMPLES(2) worst-case detection
#define HOCHARGE_DETECT_UPDATE_DURATION 100

// The battery divider is physically wired to PA0. PA6 was used while PA0 carried the
// charge-current sense; PA0 was freed when that moved to PC3, so the sense is back on the
// pin the divider actually lands on.
#define BATT_DETECT_PIN         GPIO_Pin_0 //PA0 ADC123_IN0
#define BATT_DETECT_ADC         1
#define BATT_DETECT_ADC_CHN     0
// PROVISIONAL - this is NOT a validated calibration. 6.40 came from a single pair of DMM
// readings (12.20V at the battery terminals, 1.905V at PA0), and a later second reading
// disproved it: at a pack voltage of 11.0V the pin read 1.93V. The pin rose as the pack
// fell, and a passive divider off the battery cannot have a negative slope - so PA0 is fed
// from the regulated 12V rail, not the raw pack, and 6.40 was a coincidence of the pack
// sitting near rail voltage that day. No ratio value can fix a wiring problem. Recalibrate
// against a meter once a tap is added to the raw battery screw terminals; until then
// get_electricity() reports a near-constant ~12.4V and the 15% low-battery beep in
// heartbeat_battery() will never fire.
#define BATT_DETECT_ADC_RATIO   6.40f
#define BATT_DETECT_ADC_REF     2495

#define ISCHARGE_FAULT            0x0
#define ISCHARGE_CHRG             0x1
#define ISCHARGE_NOCHRG           0x2
#define ISCHARGE_COMPLETE         0x3

#define BATT_VOLUME_CALIBRATING_DURATION 5000   /* Volume calibrating duration, in ms. */
#define BATT_SAMPLE_DURATION 30000 / 10
#define BATT_VOLUME_UPDATE_DURATION      30000  /* Volume updating duration, in ms. */
// TalentCell PB120B1, 3S4P 18650 Li-ion. Spec range 9.0V - 12.6V (3.00 - 4.20V/cell).
// These must stay pinned to the end points of BATT_SOC_CURVE in battery.c.
#define BATTERY_VOLTAGE_FULL    ((int)(12.6 * 1000)) //mV
#define BATTERY_VOLTAGE_EMPTY   ((int)(9.0 * 1000)) //mV

void init_battery(void);
_u32 get_electricity(void);
_u8 get_electricitypercentage(void);
_u8 charge_detect_getstatus(void);
_s8 get_dc_charge_status(void);
_s8 get_home_charge_status(void);
void heartbeat_battery(void);

