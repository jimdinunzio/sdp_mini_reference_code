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


//#define _DEBUG
//#define _PROTOTYPE_BOARD

#include "rpstm32.h"

#ifdef _DEBUG
#include <stdio.h>
#define DBG_OUT(...) printf(__VA_ARGS__)
#else
#define DBG_OUT(...)
#endif

#define FIRMWARE_VERSION_MAJOR        (0)
#define FIRMWARE_VERSION_MINOR        (90)
#define HARDWARE_VERSION_MAJOR        (1)

#define SSID_LEN    40
#define IP_LEN      20

#if defined(__ICCARM__) || defined(__CC_ARM) || defined(_WIN32)
#pragma pack(1)
#endif

typedef struct _baseInfo {
    _u8 model[12];
    _u16 firmwareVersion;
    _u16 hardwareVersion;
    _u32 serialNumber[3];
} __attribute__ ((packed)) baseInfo_t;

#if defined(__ICCARM__) || defined(__CC_ARM) || defined(_WIN32)
#pragma pack()
#endif

extern baseInfo_t baseInfo;

_s32 init_board();
