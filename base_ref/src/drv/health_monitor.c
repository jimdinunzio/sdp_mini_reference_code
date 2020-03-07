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
#include "health_monitor.h"

static health_status_t g_health_status; /**< Global health status. */
static _u32 g_health_ticks = 0;         /**< Health monitor ticks. */

static health_desc_t g_health_desc[CONFIG_HEALTH_MONITOR_NUM];  /**< Health description table. */
static _u8 g_health_map[CONFIG_HEALTH_MONITOR_NUM];     /**< Health id hash table. */

#define eid_mask_level(eid)     ((eid) & ~SLAMWARECORE_HEALTH_ERROR_LEVEL_MASK)

/*
 * health monitor?????
 */
void health_monitor_init(void)
{
    memset(g_health_map, 0, sizeof(g_health_map));
    memset(g_health_desc, 0, sizeof(g_health_desc));
    memset(&g_health_status, 0, sizeof(health_status_t));
    return ;
}

/*
 * health monitor????
 */
void health_monitor_exit(void)
{
    memset(g_health_map, 0, sizeof(g_health_map));
    memset(g_health_desc, 0, sizeof(g_health_desc));
    memset(&g_health_status, 0, sizeof(health_status_t));
    return ;
}

/*
 * ??health monitor????
 */
bool health_monitor_register(_u32 eid, const char *desc, health_cb_t hcb)
{
    _u8 i;

    eid = eid_mask_level(eid);
    if (eid == HEALTH_MONITOR_EID_NONE) {
        return false;
    }
    if (hcb == NULL || desc == NULL) {
        return false;
    }

    for (i = 0; i < CONFIG_HEALTH_MONITOR_NUM; i++) {
        /* Skip used slot. */
        if (eid_mask_level(g_health_desc[i].eid) != HEALTH_MONITOR_EID_NONE) {
            continue;
        }
        /* Empty slot, fill it. */
        g_health_desc[i].eid = eid;
        g_health_desc[i].hcb = hcb;
        memset(g_health_desc[i].desc, 0, sizeof(g_health_desc[i].desc));
        strncpy(g_health_desc[i].desc, desc, sizeof(g_health_desc[i].desc));
        return true;
    }
    return false;
}

/*
 * ??health monitor????
 */
bool health_monitor_deregister(_u32 eid)
{
    _u8 i;

    eid = eid_mask_level(eid);
    if (eid == HEALTH_MONITOR_EID_NONE) {
        return false;
    }

    for (i = 0; i < CONFIG_HEALTH_MONITOR_NUM; i++) {
        if (eid == eid_mask_level(g_health_desc[i].eid)) {
            g_health_desc[i].eid  = HEALTH_MONITOR_EID_NONE;
            g_health_desc[i].hcb  = NULL;
            memset(g_health_desc[i].desc, 0, sizeof(g_health_desc[i].desc));
            return true;
        }
    }
    return false;
}

/*
 * health monitor????
 */
void health_monitor_heartbeat(void)
{
    _u8  i;
    _u8  error;

    if (getms() - g_health_ticks < CONFIG_HEALTH_MONITOR_TICKS) {
        return ;
    }

    /* Poll all health monitor objects for current health status. */
    g_health_status.count = 0;
    g_health_status.flag  = 0;
    for (i = 0; i < CONFIG_HEALTH_MONITOR_NUM; i++) {
        if (eid_mask_level(g_health_desc[i].eid) == HEALTH_MONITOR_EID_NONE) {
            continue;
        }
        if (g_health_desc[i].hcb == NULL) {
            continue;
        }
        error = g_health_desc[i].hcb();
        if (error) {
            g_health_map[g_health_status.count++] = i;
        }
        g_health_desc[i].eid  = eid_mask_level(g_health_desc[i].eid);
        g_health_desc[i].eid |= error << 24;
        g_health_status.flag |= error;
    }

    return ;
}

/*
 * ????health????
 */
bool health_monitor_get_status(_u8 *status, _u8 *count)
{
    if (status != NULL) {
        *status = g_health_status.flag;
    }
    if (count != NULL) {
        *count = g_health_status.count;
    }
    return true;
}

/*
 * ???????,??health????
 */
_u32 health_monitor_get_error(_u8 id)
{
    _u8 i;

    if (id >= g_health_status.count) {
        return HEALTH_MONITOR_EID_NONE;
    }
    /* Look up health descriptor table index from hash table. */
    i = g_health_map[id];
    if (i >= CONFIG_HEALTH_MONITOR_NUM) {
        return HEALTH_MONITOR_EID_NONE;
    }
    return g_health_desc[i].eid;
}

/*
 * ???????,??health??????
 */
const char *health_monitor_get_desc(_u8 id)
{
    _u8 i;

    if (id >= g_health_status.count) {
        return NULL;
    }
    /* Look up health descriptor table index from hash table. */
    i = g_health_map[id];
    if (i >= CONFIG_HEALTH_MONITOR_NUM) {
        return NULL;
    }
    return g_health_desc[i].desc;
}

/*
 * ?????,??health????
 */
bool health_monitor_clear(_u32 eid)
{
    _u8 i;

    eid = eid_mask_level(eid);
    if (eid == HEALTH_MONITOR_EID_NONE) {
        return false;
    }

    for (i = 0; i < CONFIG_HEALTH_MONITOR_NUM; i++) {
        if (eid == eid_mask_level(g_health_desc[i].eid)) {
            g_health_desc[i].eid = eid;
            return true;
        }
    }
    return false;
}
