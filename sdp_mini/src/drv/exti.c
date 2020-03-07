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
#include "exti.h"

/**
 @brief External line interrupt call back function.
 */
static exti_cb_t g_exti_cb[CONFIG_EXTI_MAX_NUM];
static _u8 g_irq_flag[CONFIG_EXIT_IRQ_NUM];

/**
 @brief External line 0 interrupt.
 @param none.
 @return none.
 */
void EXTI0_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_Line0) != (uint32_t)RESET) {
        EXTI_ClearITPendingBit(EXTI_Line0);
        if (g_exti_cb[0] != NULL) {
            g_exti_cb[0]();
        }
    }
}

/**
 @brief External line 1 interrupt.
 @param none.
 @return none.
 */
void EXTI1_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_Line1) != (uint32_t)RESET) {
        EXTI_ClearITPendingBit(EXTI_Line1);
        if (g_exti_cb[1] != NULL) {
            g_exti_cb[1]();
        }
    }
}

/**
 @brief External line 2 interrupt.
 @param none.
 @return none.
 */
void EXTI2_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_Line2) != (uint32_t)RESET) {
        EXTI_ClearITPendingBit(EXTI_Line2);
        if (g_exti_cb[2] != NULL) {
            g_exti_cb[2]();
        }
    }
}

/**
 @brief External line 3 interrupt.
 @param none.
 @return none.
 */
void EXTI3_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_Line3) != (uint32_t)RESET) {
        EXTI_ClearITPendingBit(EXTI_Line3);
        if (g_exti_cb[3] != NULL) {
            g_exti_cb[3]();
        }
    }
}

/**
 @brief External line 4 interrupt.
 @param none.
 @return none.
 */
void EXTI4_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_Line4) != (uint32_t)RESET) {
        EXTI_ClearITPendingBit(EXTI_Line4);
        if (g_exti_cb[4] != NULL) {
            g_exti_cb[4]();
        }
    }
}

/**
 @brief External line 5~9 interrupt.
 @param none.
 @return none.
 */
void EXTI9_5_IRQHandler(void)
{
    _u8 i;

    for (i = 5; i <= 9; i++) {
        if ((EXTI->PR & EXTI_LINE(i)) != (uint32_t)RESET) {
            EXTI_ClearITPendingBit(EXTI_LINE(i));
            if (g_exti_cb[i] != NULL) {
                g_exti_cb[i]();
            }
        }
    }
}

/**
 @brief External line 10~15 interrupt.
 @param none.
 @return none.
 */
void EXTI15_10_IRQHandler(void)
{
    _u8 i;

    for (i = 10; i <= 15; i++) {
        if ((EXTI->PR & EXTI_LINE(i)) != (uint32_t)RESET) {
            EXTI_ClearITPendingBit(EXTI_LINE(i));
            if (g_exti_cb[i] != NULL) {
                g_exti_cb[i]();
            }
        }
    }
}

/**
 @brief Register external line interrupt callback.
 @param exti - external line interrupt number.
 @param type - interrupt trigger type.
 @param cb   - call back function.
 @return return true if success, or false if not.

 This fuction will return false if the call back has already been registered.
 */
bool exti_reg_callback(_u8 exti, EXTITrigger_TypeDef type, exti_cb_t cb)
{
    _u8              irq;
    _u8              pri;
    EXTI_InitTypeDef exti_cfg;
    NVIC_InitTypeDef nvic_cfg;

    if (exti >= CONFIG_EXTI_MAX_NUM || cb == NULL) {
        return false;
    }
    /* Check if the call back has been registered already. */
    if (g_exti_cb[exti] != NULL) {
        return false;
    }

    EXTI_ClearITPendingBit(EXTI_LINE(exti));

    exti_cfg.EXTI_Line    = EXTI_LINE(exti);
    exti_cfg.EXTI_Mode    = EXTI_Mode_Interrupt;	
    exti_cfg.EXTI_Trigger = type;
    exti_cfg.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_cfg);

    if (exti < 5) {
        irq = EXTI0_IRQn + exti;
        pri = exti;
        g_irq_flag[pri] = 1;
    } else if (exti < 10) {
        irq = EXTI9_5_IRQn;
        pri = 5;
        g_irq_flag[pri] |= (1 << (exti-5));
    } else {
        irq = EXTI15_10_IRQn;
        pri = 6;
        g_irq_flag[pri] |= (1 << (exti-10));
    }

    nvic_cfg.NVIC_IRQChannel = irq;
    nvic_cfg.NVIC_IRQChannelPreemptionPriority = 1;
    nvic_cfg.NVIC_IRQChannelSubPriority = pri;
    nvic_cfg.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_cfg);

    g_exti_cb[exti] = cb;
    return true;
}

/**
 @brief Un-register external line interrupt callback.
 @param exti - external line interrupt number.
 @return none.
 */
void exti_unreg_callback(_u8 exti)
{
    _u8              irq;
    _u8              pri;
    EXTI_InitTypeDef exti_cfg;
    NVIC_InitTypeDef nvic_cfg;

    if (exti >= CONFIG_EXTI_MAX_NUM) {
        return ;
    }
    /* Check if the call back has been registered already. */
    if (g_exti_cb[exti] == NULL) {
        return ;
    }

    EXTI_ClearITPendingBit(EXTI_LINE(exti));

    exti_cfg.EXTI_Line    = EXTI_LINE(exti);
    exti_cfg.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti_cfg.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti_cfg.EXTI_LineCmd = DISABLE;
    EXTI_Init(&exti_cfg);

    if (exti < 5) {
        irq = EXTI0_IRQn + exti;
        pri = exti;
        g_irq_flag[pri] = 0;
    } else if (exti < 10) {
        irq = EXTI9_5_IRQn;
        pri = 5;
        g_irq_flag[pri] &= ~(1 << (exti-5));
    } else {
        irq = EXTI15_10_IRQn;
        pri = 6;
        g_irq_flag[pri] &= ~(1 << (exti-10));
    }

    if (!g_irq_flag[pri]) {
        nvic_cfg.NVIC_IRQChannel = irq;
        nvic_cfg.NVIC_IRQChannelPreemptionPriority = 1;
        nvic_cfg.NVIC_IRQChannelSubPriority = pri;
        nvic_cfg.NVIC_IRQChannelCmd = DISABLE;
        NVIC_Init(&nvic_cfg);
    }

    g_exti_cb[exti] = NULL;
    return ;
}
