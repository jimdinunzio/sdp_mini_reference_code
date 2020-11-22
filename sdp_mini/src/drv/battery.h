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


#define HOCHARGE_DETECT         GPIO_Pin_10
#define DCCHARGE_DETECT         GPIO_Pin_11
#define BATT_FAULT              GPIO_Pin_4
#define BATT_CHRG               GPIO_Pin_3
#define BATT_READY              GPIO_Pin_2

#define BATT_DETECT_PORT        GPIOA
#define BATT_DETECT_PIN         GPIO_Pin_6
#define BATT_DETECT_ADC         1
#define BATT_DETECT_ADC_CHN     6
#define BATT_DETECT_ADC_RATIO   11.0f
#define BATT_DETECT_ADC_REF     2495

#define ISCHARGE_FAULT            0x0
#define ISCHARGE_CHRG             0x1
#define ISCHARGE_NOCHRG           0x2
#define ISCHARGE_COMPLETE         0x3

#define BATT_VOLUME_CALIBRATING_DURATION 5000   /* Volume calibrating duration, in ms. */
#define BATT_VOLUME_UPDATE_DURATION      30000  /* Volume updating duration, in ms. */

// the voltage scale factor to transform the voltage on the ADC pin to the actual battery voltage
// it is controlled by the resistor network, please refer to the ref design schematic for details
#define BATTERY_VOLTAGE_FULL    ((int)(12.0 * 1000)) //mV
#define BATTERY_VOLTAGE_EMPTY   ((int)(7.0 * 1000)) //mV

void init_battery(void);
_u32 get_electricity(void);
_u8 get_electricitypercentage(void);
_u8 charge_detect_getstatus(void);
_s8 get_dc_charge_status(void);
_s8 get_home_charge_status(void);
void heartbeat_battery(void);

