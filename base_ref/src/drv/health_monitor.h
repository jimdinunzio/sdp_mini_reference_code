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

#include <stdio.h>
#include "common/common.h"
#include "drv/ctrl_bus_cmd.h"

#define CONFIG_HEALTH_MONITOR_TICKS     100

#define CONFIG_HEALTH_MONITOR_NUM       10

#define HEALTH_MONITOR_EID_NONE         0

#define CONFIG_HEALTH_DESC_SIZE         32

/* health status strcture. */
typedef struct _health_status {
    _u8 flag;           /**< Health status flag. */
    _u8 count;          /**< Health error count. */
} health_status_t;

/**
 Health callback function.

 The return code could be:
    SLAMWARECORECB_HEALTH_FLAG_OK
    SLAMWARECORECB_HEALTH_FLAG_WARN
    SLAMWARECORECB_HEALTH_FLAG_ERROR
    SLAMWARECORECB_HEALTH_FLAG_FATAL
 */
typedef _u8 (*health_cb_t)(void);

/* health description structure. */
typedef struct _health_desc {
    _u32         eid;                           /**< Error id. */
    char         desc[CONFIG_HEALTH_DESC_SIZE]; /**< Error description. */
    health_cb_t  hcb;                           /**< Health callback function. */
} health_desc_t;

/* health monitor public functions. */
void health_monitor_init(void);
void health_monitor_exit(void);
bool health_monitor_register(_u32 eid, const char *desc, health_cb_t hcb);
bool health_monitor_deregister(_u32 eid);
void health_monitor_heartbeat(void);
bool health_monitor_get_status(_u8 *status, _u8 *count);
_u32 health_monitor_get_error(_u8 id);
const char *health_monitor_get_desc(_u8 id);
bool health_monitor_clear(_u32 eid);
