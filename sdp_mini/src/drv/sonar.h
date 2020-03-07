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

#include <stdio.h>
#include <stdint.h>
#include "common/common.h"

/**
 @addtogroup sonar
 @{
 */

#ifndef _SONAR_H
#define _SONAR_H

/**
 @brief ultrasonic speed coefficient is different by temperature.

    Tempearture  -30  -20 -10  0   10  20  30 100
    Speed(m/s)   313 319  325 332 338 344 349 386

    V = 331.45 + 0.607 * T
    S = V * t / 2
  FIXME: use tempearture sensor to select speed coefficient.
 */
#define CONFIG_SONAR_COE_A              331.45  /*< Sonar distance coefficient A. */
#define CONFIG_SONAR_COE_B              0.607   /*< Sonar distance coefficient B. */
#define CONFIG_SONAR_COE_T              20      /*< Sonar distance coefficient T at 20. */

#define CONFIG_SONAR_CHANNEL_NUM        4       /**< Sonar channel number. */
#define CONFIG_SONAR_SAMPLE_SIZE        4       /**< Sonar sample buffer size. */
#define CONFIG_SONAR_TIMEOUT_MS         50      /**< Sonar detection timeout in ms. */
#define CONFIG_SONAR_TICKS              100     /**< Sonar detection timeout in ms. */

#define CONFIG_SONAR_BLOCK_DIST         100     /**< Sonar block distance in mm. */

//#define CONFIG_SONAR_DISTANCE_Q16       1     /**< Q16 formatted distance. */

/**
 @brief Decimal to fix pointer number converter.
 */
#define FP_Q16(x)       ((x) * (1<<16))
#define FP_Q8(x)        ((x) * (1<<8))

/**
 @brief sonar channel info structure.
 */
typedef struct _sonar_channel {
    uint8_t  state;                             /**< Sonar sensor state. */
    uint8_t  id;                                /**< Sonar smaple buffer id. */
    uint8_t  cnt;                               /**< Sonar smaple count. */
    uint16_t sample[CONFIG_SONAR_SAMPLE_SIZE];  /**< Sonar sample buffer. */
    uint32_t ticks;                             /**< Sonar sample ticks. */
    uint32_t distance;                          /**< Sonar sampel result. */
} sonar_channel_t;

/**
 @brief sonar channel configuration.
 */
typedef struct _sonar_cfg {
    GPIO_TypeDef *trig_port;        /**< Sonar channel trigger IO port. */
    uint16_t      trig_pin;         /**< Sonar channel tirgger IO pin. */
    GPIO_TypeDef *echo_port;        /**< Sonar channel echo input IO port. */
    uint16_t      echo_pin;         /**< Sonar channel echo input IO pin. */
    uint32_t      exti_line;        /**< Sonar channel echo input interrupt line. */
} sonar_cfg_t;

/**
 @brief Sonar sensor states.
 */
enum _sonar_state {
    SONAR_IDLE = 0,
    SONAR_INIT,
    SONAR_MEASURE,
    SONAR_DONE,
    SONAR_EXIT,
};

void sonar_init(void);
void sonar_exit(void);
void sonar_heartbeat(void);
uint32_t sonar_get(uint8_t ch);

#define init_sonar()    sonar_init()
#define heartbeat_sonar()   sonar_heartbeat()

#endif

/** @} */
