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
#include "drv/led.h"

/**
 @defgroup led led driver
 @addtogroup drivers
 @{
 */

#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV == 2)
static int led_r, led_g, led_b;

/**
 @brief LED initialize.
 @param none.
 @return none.
 */
void drv_led_init(void)
{
    pinMode(LED_RED_PORT,   LED_RED_PIN,   GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinMode(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    pinMode(LED_BLUE_PORT,  LED_BLUE_PIN,  GPIO_Mode_Out_PP, GPIO_Speed_50MHz);

    PIN_SET(LED_RED_PORT,   LED_RED_PIN, 0);
    PIN_SET(LED_GREEN_PORT, LED_GREEN_PIN, 0);
    PIN_SET(LED_BLUE_PORT,  LED_BLUE_PIN, 0);
    led_r = led_g = led_b = 0;
}

/**
 @brief LED shutdown.
 @param none.
 @return none.
 */
void drv_led_shutdown(void)
{
    pinMode(LED_RED_PORT,   LED_RED_PIN,   GPIO_Mode_Out_OD, GPIO_Speed_50MHz);
    pinMode(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_Mode_Out_OD, GPIO_Speed_50MHz);
    pinMode(LED_BLUE_PORT,  LED_BLUE_PIN,  GPIO_Mode_Out_OD, GPIO_Speed_50MHz);

    PIN_SET(LED_RED_PORT,   LED_RED_PIN,   0);
    PIN_SET(LED_GREEN_PORT, LED_GREEN_PIN, 0);
    PIN_SET(LED_BLUE_PORT,  LED_BLUE_PIN,  0);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, DISABLE);
    led_r = led_g = led_b = 0;
}

/**
 @brief LED color set.
 @param r - red color, 0/1.
 @param g - green color, 0/1.
 @param b - blue color, 0/1.
 @return none.
 */
void drv_led_set(int r, int g, int b)
{
    if (led_r != r) {
        led_r = r;
            
        if (r) {
            PIN_SET(LED_RED_PORT, LED_RED_PIN, 1);
        } else {
            PIN_SET(LED_RED_PORT, LED_RED_PIN, 0);
        }
    }

    if (led_g != g) {
        led_g = g;
            
        if (g) {
            PIN_SET(LED_GREEN_PORT, LED_GREEN_PIN, 1);
        } else {
            PIN_SET(LED_GREEN_PORT, LED_GREEN_PIN, 0);
        }
    }

    if (led_b != b) {
        led_b = b;
            
        if (b) {
            PIN_SET(LED_BLUE_PORT, LED_BLUE_PIN, 1);
        } else {
            PIN_SET(LED_BLUE_PORT, LED_BLUE_PIN, 0);
        }
    }
}

#elif defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
/**
 @brief INK1002 color led driver.
 */

static int led_r, led_g, led_b;

/**
 @brief 250ns delay macro.

 the delay should be within 250ns+-75ns.
 The real delay time may be a little bit different by OS level.
 The delay in OS size, is about 250ns.
 The delay in OS none, is about 260ns.
 */
#define delay_250ns()  do {    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
} while (0)

/**
 @brief Color led bit command.
 @param bit - bit value, 0 or 1.
 @return none.

  Bit 0 command pulse.
    ____
   |    |________________|
   0.25us      1.0us

  Bit 1 command pulse
    ________________
   |                |____|
        1.0us       0.25us

  RESET command
   |_____________________|
        >= 24us
 */
#define drv_led_bit(bit)    do {    \
    if (bit) {  \
        PIN_SET(LED_COLOR_PORT, LED_COLOR_PIN, 1);  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
        PIN_SET(LED_COLOR_PORT, LED_COLOR_PIN, 0);  \
        delay_250ns();  \
    } else {    \
        PIN_SET(LED_COLOR_PORT, LED_COLOR_PIN, 1);  \
        delay_250ns();  \
        PIN_SET(LED_COLOR_PORT, LED_COLOR_PIN, 0);  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
        delay_250ns();  \
    }   \
} while (0)

/**
 @brief Color led byte command send.
 @param cmd - byte command.
 @return none.
 */
static void drv_led_cmd(uint8_t cmd)
{
    drv_led_bit(cmd & 0x80);
    drv_led_bit(cmd & 0x40);
    drv_led_bit(cmd & 0x20);
    drv_led_bit(cmd & 0x10);
    drv_led_bit(cmd & 0x08);
    drv_led_bit(cmd & 0x04);
    drv_led_bit(cmd & 0x02);
    drv_led_bit(cmd & 0x01);
}

/**
 @brief LED reset.
 @param none.
 @return none.

 Just wait for the active of new settings.
 */
static void drv_led_reset(void)
{
    _delay_us(20);
}

/**
 @brief LED color set.
 @param r - red color, 0 ~ 255.
 @param g - green color, 0 ~ 255.
 @param b - blue color, 0 ~ 255.
 @return none.
 */
void drv_led_set(int r, int g, int b)
{
    uint8_t reset = false;
    r *= 0x10;
    g *= 0x10;
    b *= 0x10;

    if (r < 0 || r > 0xFF) {
        return ;
    }
    if (g < 0 || g > 0xFF) {
        return ;
    }
    if (b < 0 || b > 0xFF) {
        return ;
    }

    if (led_r != r) {
        led_r = r;
        reset = true;
    }

    if (led_g != g) {
        led_g = g;
        reset = true;
    }

    if (led_b != b) {
        led_b = b;
        reset = true;
    }

    if (reset) {
        drv_led_cmd(led_g);
        drv_led_cmd(led_r);
        drv_led_cmd(led_b);
        /* FIXME: to make the settings active, should wait reset.
         * But it'll waste a little time. So let caller do this.
         */
//      drv_led_reset();
    }
}

/**
 @brief Color LED initialize.
 @param none.
 @return none.
 */
void drv_led_init(void)
{
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 5)
    pinMode(LED_COLOR_PORT, LED_COLOR_PIN, GPIO_Mode_Out_OD, GPIO_Speed_50MHz);
#else
    pinMode(LED_COLOR_PORT, LED_COLOR_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
#endif
    PIN_SET(LED_COLOR_PORT, LED_COLOR_PIN, 0);
    _delay_us(100);

    led_r = led_g = led_b = -1;
    drv_led_set(0, 0, 0);
    drv_led_reset();
}

/**
 @brief Color LED shutdonw.
 @param none.
 @return none.
 */
void drv_led_shutdown(void)
{
    pinMode(LED_COLOR_PORT, LED_COLOR_PIN, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);
    PIN_SET(LED_COLOR_PORT, LED_COLOR_PIN, 0);
    drv_led_set(0, 0, 0);
    drv_led_reset();
    led_r = led_g = led_b = 0;
}

/**
 @brief Color LED test.
 @param none.
 @return none.
 */
void drv_led_test(void)
{
    int r, g, b;

    while (1) {
        r = g = b = 0;
        while (r != 0xFF) {
            drv_led_set(r++, 0, 0);
            _delay_ms(10);
        }
        while (g != 0xFF) {
            drv_led_set(0, g++, 0);
            _delay_ms(10);
        }
        while (b != 0xFF) {
            drv_led_set(0, 0, b++);
            _delay_ms(10);
        }
        r = g = b = 0;
        while (r != 0xFF) {
            drv_led_set(r++, g++, 0);
            _delay_ms(10);
        }
        r = g = b = 0;
        while (r != 0xFF) {
            drv_led_set(r++, 0, b++);
            _delay_ms(10);
        }
        r = g = b = 0;
        while (g != 0xFF) {
            drv_led_set(0, g++, b++);
            _delay_ms(10);
        }
        r = g = b = 0;
        while (r != 0xFF) {
            drv_led_set(r++, g++, b++);
            _delay_ms(10);
        }
    }
}

#endif

/** @} */
