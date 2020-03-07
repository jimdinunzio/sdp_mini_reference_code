/*
 * SlamTec Infra Runtime Public
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

#ifndef __TIMER_H
#define __TIMER_H

/**
 @brief Timer function defintions.
 */
typedef void (*tim_oc_init_t)(TIM_TypeDef*, TIM_OCInitTypeDef*);
typedef void (*tim_oc_preloadcfg)(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
typedef void (*tim_set_compare)(TIM_TypeDef* TIMx, uint16_t Compare3);

/**
 @brief timer function strcture.
 */
typedef struct tim_func {
    tim_oc_init_t     oc_init;          /**< Timer oc init function. */
    tim_oc_preloadcfg oc_preloadcfg;    /**< Timer oc preload configure function. */
    tim_set_compare   set_compare;      /**< Timer set compare status function. */
} tim_func_t;

extern const tim_func_t g_tim_func[];   /**< Global timer function array. */

#define CONFIG_TIME_CHANNEL_NUM         4   /**< Maximum timer channel. */

/**
 @brief Timer functions wrapper.
 */
#define tim_oc_init(tim, ch, oc)                      \
    if (ch > 0 && ch <= CONFIG_TIME_CHANNEL_NUM) {    \
        g_tim_func[ch-1].oc_init(tim, oc);            \
    }

#define tim_oc_preloadcfg(tim, ch, preload)           \
    if (ch > 0 && ch <= CONFIG_TIME_CHANNEL_NUM) {    \
        g_tim_func[ch-1].oc_preloadcfg(tim, preload); \
    }

#define tim_set_compare(tim, ch, duty)                \
    if (ch > 0 && ch <= CONFIG_TIME_CHANNEL_NUM) {    \
        g_tim_func[ch-1].set_compare(tim, duty);      \
    }

#endif
