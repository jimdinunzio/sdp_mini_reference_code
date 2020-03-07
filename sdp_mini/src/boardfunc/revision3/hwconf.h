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

//MCU主频
#define CPU_FREQ 72000000L
//串口发送缓存区大小
#define USART_MAX_TX_COUNT   512

//flash存储相关
#define STORAGE_SLOT 6
#define STORAGE_DATA_START_PG_NUM 62
#define STORAGE_DATA_PAGE1 (FLASH_ADDR_BASE + FLASH_PAGE_SIZE*STORAGE_DATA_START_PG_NUM)
#define STORAGE_DATA_PAGE2 (FLASH_ADDR_BASE + FLASH_PAGE_SIZE*(STORAGE_DATA_START_PG_NUM+1))

//USART相关
#define USART1_PORT             GPIOA
#define USART1_ID               1
#define USART1_TX_PIN           GPIO_Pin_9
#define USART1_RX_PIN           GPIO_Pin_10
#define USART2_PORT             GPIOA
#define USART2_ID               2
#define USART2_TX_PIN           GPIO_Pin_2
#define USART2_RX_PIN           GPIO_Pin_3

# define USART3_PORT            GPIOC
# define USART3_ID              3
# define USART3_TX_PIN          GPIO_Pin_10
# define USART3_RX_PIN          GPIO_Pin_11

# define DBG_USART_PORT         USART3_PORT
# define DBG_USART_ID           USART3_ID
# define DBG_USART_TX_PIN       USART3_TX_PIN
# define DBG_USART_RX_PIN       USART3_RX_PIN
# define DBG_IS_SHARED          1
# define PRINTF_USART_PORT      USART3_ID

// CTRL BUS相关
#define DATAOUT_USART_ID        2

#define USART_CTRLBUS_ID        USART1_ID
#define USART_CTRLBUS_PORT      USART1_PORT
#define USART_CTRLBUS_TX_PIN    USART1_TX_PIN
#define USART_CTRLBUS_RX_PIN    USART1_RX_PIN

#define CTRLBUS_CCMD_PORT       GPIOC
#define CTRLBUS_CCMD_PIN        GPIO_Pin_9
#define CTRLBUS_CBUSY_PORT      GPIOA
#define CTRLBUS_CBUSY_PIN       GPIO_Pin_12

// USB相关
#define USB_PORT                GPIOA
#define USB_DP                  GPIO_Pin_12
#define USB_DN                  GPIO_Pin_11

// IIC相关
#define I2C1_PORT               GPIOB
#define I2C1_SCL                GPIO_Pin_6
#define I2C1_SDA                GPIO_Pin_7
#define I2C2_PORT               GPIOB
#define I2C2_SCL                GPIO_Pin_10
#define I2C2_SDA                GPIO_Pin_11
#define I2C_CONF_USE_I2C1
#define I2C_CONF_USE_I2C2
#define I2C1_CONF_BAUDRATE      350000
#define I2C1_CONF_SELF_ADDR     0
#define I2C2_CONF_BAUDRATE      350000
#define I2C2_CONF_SELF_ADDR     0

// SPI相关
#define SPI1_PORT               GPIOA
#define SPI1_MISO               GPIO_Pin_6
#define SPI1_MOSI               GPIO_Pin_7
#define SPI1_SCK                GPIO_Pin_5
#define SPI1_NSS                GPIO_Pin_4
#define SPI2_PORT               GPIOB
#define SPI2_MISO               GPIO_Pin_14
#define SPI2_MOSI               GPIO_Pin_15
#define SPI2_SCK                GPIO_Pin_13
#define SPI1_BAUDRATE_BASE 72000
#define SPI2_BAUDRATE_BASE 36000
#define SPI1_BAUDRATE_PRESCALER      32 //2.25Mhz
#define SPI2_BAUDRATE_PRESCALER      8  //4.5Mhz
#define SPI1_CONF_CPHA 0
#define SPI1_CONF_CPOL 0
#define SPI2_CONF_CPHA 1
#define SPI2_CONF_CPOL 1

// ADC相关
#define ADC_REF_VOLT 2.495

//IO端口重映射
# define PERFORM_IO_REMAP() GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);\
                            GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);\
                            GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE)

//APB2总线上的外设

# define LED_COLOR_PORT             GPIOB
# define LED_COLOR_PIN              GPIO_Pin_12

# define LED_RED_PORT               GPIOB
# define LED_RED_PIN                GPIO_Pin_12
# define LED_GREEN_PORT             GPIOB
# define LED_GREEN_PIN              GPIO_Pin_14
# define LED_BLUE_PORT              GPIOD
# define LED_BLUE_PIN               GPIO_Pin_11

# define SONAR_TRIG1_PORT           GPIOE
# define SONAR_TRIG1_PIN            GPIO_Pin_10

# define SONAR_ECHO1_PORT           GPIOE
# define SONAR_ECHO1_PIN            GPIO_Pin_5

# define SONAR_TRIG2_PORT           GPIOE
# define SONAR_TRIG2_PIN            GPIO_Pin_11

# define SONAR_ECHO2_PORT           GPIOE
# define SONAR_ECHO2_PIN            GPIO_Pin_7

# define SONAR_TRIG3_PORT           GPIOE
# define SONAR_TRIG3_PIN            GPIO_Pin_12

# define SONAR_ECHO3_PORT           GPIOE
# define SONAR_ECHO3_PIN            GPIO_Pin_8

# define SONAR_TRIG4_PORT           GPIOE
# define SONAR_TRIG4_PIN            GPIO_Pin_15

# define SONAR_ECHO4_PORT           GPIOE
# define SONAR_ECHO4_PIN            GPIO_Pin_9

#define WIFI_RESET_PORT             GPIOA
#define WIFI_RESET_PIN              GPIO_Pin_3

#define APB2PERIPH_INIT_LIST  \
  ( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC \
  | RCC_APB2Periph_AFIO \
  | RCC_APB2Periph_USART1 \
  | RCC_APB2Periph_ADC1 \
  | RCC_APB2Periph_ADC2 \
  | GET_ADC_PERIPH(ADC_INNER_TEMPER_PORT) \
  )
//APB1总线上的外设
#define APB1PERIPH_INIT_LIST  \
  ( RCC_APB1Periph_USART2 \
  | RCC_APB1Periph_TIM3 \
  )
