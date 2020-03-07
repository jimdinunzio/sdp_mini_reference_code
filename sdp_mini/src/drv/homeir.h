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
#ifndef _HOMEIR_H__
#define _HOMEIR_H__

#include "common/common.h"

#define HOME_IR_R1 GPIO_Pin_12
#define HOME_IR_R2 GPIO_Pin_13
#define HOME_IR_R3 GPIO_Pin_14

#define HOME_IR_START_TIME 4500
#define HOME_IR_START_ERROR 300
#define HOME_IR_BIT_HIGH_TIME 1680
#define HOME_IR_BIT_HIGH_ERROR 300
#define HOME_IR_BIT_LOW_TIME 560
#define HOME_IR_BIT_LOW_ERROR 300

#define HOME_IR_REMOTE_CONTROL 0x00//whit device to connect me.
#define HOME_IR_CHARGING_DOCK 0x80

#define HOME_KEY        18
#define CLEAN_KEY       84
#define UP_KEY          54
#define DOWN_KEY        152
#define LEFT_KEY        118
#define RIGHT_KEY       140
#define TIMER_KEY       170
#define FOCUS_KEY       190
#define EDGE_KEY        220

enum {
  HOME_IR_MAIN = 0,
  HOME_IR_LEFT = 1,
  HOME_IR_RIGHT = 2,
};


void homeir_Init(void);
uint8_t homeir_GetData(void);
void homeir_heartbeat(void);

uint8_t homeir_getmaindata(void);
uint8_t homeir_getleftdata(void);
uint8_t homeir_getrightdata(void);

#define init_homeir()       homeir_Init()
#define heartbeat_homeir()  homeir_heartbeat()

#endif /* _HOMEIR_H__ */
