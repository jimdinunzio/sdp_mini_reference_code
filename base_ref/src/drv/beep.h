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

#define BEEP_GPIO       GPIOA
#define BEEP_PIN        GPIO_Pin_15
#define BEEP_PWM        TIM2

#define TIME_PERIOD 255
#define TIME_PRESCALER (TIME_PERIOD + 1)
#define BEEP_INIT_HZ 1568

#define BEEP_OPEN() (GPIO_SetBits(GPIOC, GPIO_Pin_12))
#define BEEP_CLOSE() (GPIO_ResetBits(GPIOC, GPIO_Pin_12))
enum {
    BEEP_START = 1,
    BEEP_STOP = 2,
    BEEP_WORKING = 3,
};
enum {
    BEEP_POWERON = 1,
    BEEP_POWEROFF = 2,
    BEEP_POWERLOW = 3,
    BEEP_POWERCHARGE = 4,
    BEEP_PLAYWORKING = 5,
};
void init_beep(void);
void play_music(void);
void play_poweron(void);
void beep_beeper(_u32 frequency, _u32 delay, _u8 sound);
void heartbeat_beep(void);
