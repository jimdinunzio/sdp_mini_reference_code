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
#ifndef __EXTI_H
#define __EXTI_H

#include "common/common.h"
#include <stdio.h>

#define CONFIG_EXTI_MAX_NUM             16  /**< Maximum external line channels. */
#define CONFIG_EXIT_IRQ_NUM             7   /**< External irq number, irq EXTI0~EXTI4, EXTI9-5, EXTI15-10 */

/**
 @brief get gpio port source by port id.
 @param port - gpio port id.
 @return return gpio source number.
 */
static inline _u8 GPIO_PortSource(GPIO_TypeDef *port)
{
    if (port == NULL) {
        return 0;
    }
    if (port == GPIOA) {
        return GPIO_PortSourceGPIOA;
    } else if (port == GPIOB) {
        return GPIO_PortSourceGPIOB;
    } else if (port == GPIOC) {
        return GPIO_PortSourceGPIOC;
    } else if (port == GPIOD) {
        return GPIO_PortSourceGPIOD;
    } else if (port == GPIOE) {
        return GPIO_PortSourceGPIOE;
    } else if (port == GPIOF) {
        return GPIO_PortSourceGPIOF;
    } else if (port == GPIOG) {
        return GPIO_PortSourceGPIOG;
    } else {
        return 0;
    }
}

#define EXTI_LINE(l)        (EXTI_Line0 << (l))

/**
 @brief External line interrupt call back function.
 */
typedef void (*exti_cb_t)(void);

bool exti_reg_callback(_u8 exti, EXTITrigger_TypeDef type, exti_cb_t cb);
void exti_unreg_callback(_u8 exti);

#endif
