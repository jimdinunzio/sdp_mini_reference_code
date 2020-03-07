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

#pragma once

#ifndef RPSTM32_HEADER_H
#error "You must include the rpstm32.h beform including this file."
#endif


#ifndef USART_DISABLE_USART1
  #define USART1_BUF_ID 0
#else
  #define USART1_BUF_ID -1
#endif

#ifndef USART_DISABLE_USART2
  #define USART2_BUF_ID (USART1_BUF_ID+1)
#else
  #define USART2_BUF_ID USART1_BUF_ID
#endif


#ifdef  USART_ENABLE_USART3
  #define USART3_BUF_ID (USART2_BUF_ID+1)
#endif


#define USART1_DMA_CH 4
#define USART2_DMA_CH 7
#define USART3_DMA_CH 2


#ifndef USART_MAX_TX_COUNT
#define USART_MAX_TX_COUNT   256
#endif

#ifndef USART_RX_BUFFER_SIZE
#define USART_RX_BUFFER_SIZE 128
#endif

#define _GET_USART_DMA_CH(ch)  DMA1_Channel##ch
#define _GET_USART_DMA_FLG(ch)  DMA1_FLAG_TC##ch

#define GET_USART_DMA_CH(id) EXPAND_WRAPPER(_GET_USART_DMA_CH, USART##id##_DMA_CH)
#define GET_USART_DMA_FLG(id) EXPAND_WRAPPER(_GET_USART_DMA_FLG, USART##id##_DMA_CH)

typedef void (*proc_on_rx_t)(int id, uint32_t data);

int usart_begin(USART_TypeDef* USARTx, uint32_t baud);
void usart_shutdown(USART_TypeDef* USARTx);

proc_on_rx_t usart_getrecv_func(USART_TypeDef* USARTx);
void usart_setrecv_func(USART_TypeDef* USARTx, proc_on_rx_t proc);
void usart_recv_flush(USART_TypeDef* USARTx);
uint32_t usart_recv_avail(USART_TypeDef* USARTx);
int usart_recv_getc(USART_TypeDef* USARTx);
size_t usart_recv_gets(USART_TypeDef* USARTx, unsigned char * buf, size_t size);
int usart_tx(USART_TypeDef* USARTx, const void * data, uint32_t size);
void usart_tx_wait(USART_TypeDef* USARTx);
int usart_tx_putc(USART_TypeDef* USARTx, int c);
int usart_txbuffer_set(USART_TypeDef* USARTx, const void * src, uint32_t size, uint32_t pos);
void usart_txbuffer_flush(USART_TypeDef* USARTx, uint32_t size);

static inline void usart_setrecv_default(USART_TypeDef* USARTx)
{
  usart_setrecv_func(USARTx, NULL);
}

#define usart_cmd USART_Cmd
#define USART_BEGIN(id, baud)  usart_begin( GET_USART(id), baud)
