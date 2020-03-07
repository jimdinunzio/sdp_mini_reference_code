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

baseInfo_t baseInfo;
static _u8 currentNcmd;
/*
 * ctrlbus nCMD脚初始化函数
 * 填充baseInfo，slamcore会要
 */
void init_drv_ctrlbus()
{
    pinMode(CTRLBUS_CBUSY_PORT, CTRLBUS_CBUSY_PIN, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
    pinMode(CTRLBUS_CCMD_PORT, CTRLBUS_CCMD_PIN, GPIO_Mode_Out_OD, GPIO_Speed_50MHz);
    PIN_SET(CTRLBUS_CCMD_PORT, CTRLBUS_CCMD_PIN, HIGH);

    memset(&baseInfo, 0, sizeof(baseInfo_t));
    baseInfo.firmwareVersion = (FIRMWARE_VERSION_MAJOR << 8) | FIRMWARE_VERSION_MINOR;
    baseInfo.hardwareVersion = HARDWARE_VERSION_MAJOR;
    memcpy(&baseInfo.model, "ref base\0", 9);
    for (_s32 pos = 0; pos < STM32_SELFID_LEN; ++pos) {
        baseInfo.serialNumber[pos] = get_self_id()[pos];
    }
    currentNcmd = -1;
}
/*
 * ctrlbus nCMD脚拉低函数
 * 预留
 */
void set_drv_ctrlbus_ncmd(_u8 ncmd)
{
    PIN_SET(CTRLBUS_CCMD_PORT, CTRLBUS_CCMD_PIN, LOW);
    currentNcmd = ncmd;
}
/*
 * ctrlbus nCMD脚拉高函数
 */
void clear_drv_ctrlbus_ncmd(void)
{
    pinMode(CTRLBUS_CCMD_PORT, CTRLBUS_CCMD_PIN, GPIO_Mode_Out_OD, GPIO_Speed_50MHz);
    PIN_SET(CTRLBUS_CCMD_PORT, CTRLBUS_CCMD_PIN, HIGH);
    currentNcmd = -1;
}
/*
 * 获取ctrlbus nCMD脚状态函数
 * 预留
 */
bool is_drv_ctrlbus_ncmd(void)
{
    return !(currentNcmd == (_u8) - 1);
}
/*
 * 获取ctrlbus nCMD函数
 * 预留
 */
_u8 get_drv_ctrlbus_ncmd(void)
{
    return currentNcmd;
}
/*
 * 获取ctrlbus nCMD脚状态忙函数
 * 预留
 */
bool is_drv_ctrlbus_busy(void)
{
    pinMode(CTRLBUS_CBUSY_PORT, CTRLBUS_CBUSY_PIN, GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz);
    return PIN_READ(CTRLBUS_CBUSY_PORT, CTRLBUS_CBUSY_PIN);
}
