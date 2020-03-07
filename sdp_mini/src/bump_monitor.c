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
#include "bump_monitor.h"
#include "drv/bump.h"
#include "drv/motor.h"

#define CONF_STALL_AUTO_CANCELLATION_DURATION  1000     //ms

enum {
    STALLMODE_IDLE = 0,
    STALLMODE_NO_FORWARD = 1,
    STALLMODE_NO_BACKWARD = 2,
};

static _u32 _lastHitTs;
static _u8 _stallMode;
/*
 * 离开阻塞状态函数
 */
static void leave_stall_mode(void)
{
    _stallMode = STALLMODE_IDLE;
}
/*
 * 进入阻塞状态函数
 */
static void enter_stall_mode(int mode)
{
    _stallMode = mode;
    _lastHitTs = getms();

    set_walkingmotor_speed(0, 0);       //进入阻塞时，急停
    brake_walkingmotor();
}
/*
 * 阻塞状态初始化函数
 */
_s32 init_bumpermonitor(void)
{
    _lastHitTs = 0;
    _stallMode = STALLMODE_IDLE;
    return 1;
}
void shutdown_bumpermonitor(void)
{
}
/*
 * 碰撞检测处理函数
 */
_s32 heartbeat_bumpermonitor(void)
{
    if (is_bumped()) {
        if (_stallMode == STALLMODE_IDLE) {
            enter_stall_mode(STALLMODE_NO_FORWARD);
        }
    } else {
        leave_stall_mode();
    }
    return 1;
}
/*
 * 设定速度的合法性判断函数
 * 具体是根据当前的碰撞状态
 */
_s32 bumpermonitor_filter_motorcmd(_s32 spdLeft, _s32 spdRight)
{
    switch (_stallMode) {
    case STALLMODE_IDLE:
        break;
    case STALLMODE_NO_FORWARD:
        if (spdLeft > 0 || spdRight > 0) {
            return 1;
        }
        break;
    case STALLMODE_NO_BACKWARD:
        if (spdLeft < 0 || spdRight < 0) {
            return 1;
        }
        break;
    }
    return 0;
}

float bumpermonitor_clamp_motorcmd(float motorSpd)
{
  switch (_stallMode) {
    case STALLMODE_IDLE:
        break;
    case STALLMODE_NO_FORWARD:
         // robot cannot move forward 
        if(motorSpd > 0)
        {
          return 0;
        }
        break;
    case STALLMODE_NO_BACKWARD:
         // robot cannot move forward
        if(motorSpd < 0)
        {
          return 0;
        }
        break;  
    }
  return motorSpd;
}