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

#include "common/common.h"

static void _def_on_rx(int id, uint32_t data);
/*
 * 串口发送缓存区定义
 */
static unsigned char _tx_buf[
    0
#ifndef USART_DISABLE_USART1
    + USART_MAX_TX_COUNT
#endif
#ifndef USART_DISABLE_USART2
    + USART_MAX_TX_COUNT
#endif
#ifdef  USART_ENABLE_USART3
    + USART_MAX_TX_COUNT
#endif
];
/*
 * 串口接收回调函数定义
 */
static proc_on_rx_t  _rx_proc[] = {
#ifndef USART_DISABLE_USART1
    _def_on_rx,
#endif
#ifndef USART_DISABLE_USART2
    _def_on_rx,
#endif
#ifdef  USART_ENABLE_USART3
    _def_on_rx,
#endif
};
/*
 * 串口接收环形缓存区定义
 */
typedef struct ring_buffer {
  volatile  uint8_t buffer[USART_RX_BUFFER_SIZE];
  int head;
  int tail;
} ring_buffer_t;
static ring_buffer_t _rx_ringbuf[
    0
#ifndef USART_DISABLE_USART1
    + 1
#endif
#ifndef USART_DISABLE_USART2
    + 1
#endif
#ifdef  USART_ENABLE_USART3
    + 1
#endif
];


/*
 * 获取串口号函数
 */
static int _getPortID(USART_TypeDef* USARTx)
{

#ifndef USART_DISABLE_USART1
    if (USARTx == USART1) return 1;
#endif
#ifndef USART_DISABLE_USART2
    if (USARTx == USART2) return 2;
#endif
#ifdef  USART_ENABLE_USART3
    if (USARTx == USART3) return 3;
#endif
    return -1;
}
/*
 * 获取串口缓存区索引函数
 */
static int _getBufID(int id)
{
    switch (id) {
#ifndef USART_DISABLE_USART1
    case 1:
        return USART1_BUF_ID;
#endif
#ifndef USART_DISABLE_USART2
    case 2:
        return USART2_BUF_ID;
#endif
#ifdef  USART_ENABLE_USART3
    case 3:
        return USART3_BUF_ID;
#endif
    default:
        return -1;
    }
}

#define _GET_TX_BUF(id)  (_tx_buf + _getBufID(id)*USART_MAX_TX_COUNT)
/*
 * 获取串口DMA通道函数
 */
static DMA_Channel_TypeDef* _getDMACh(int id)
{
  switch (id)
  {
#ifndef USART_DISABLE_USART1
  case 1:
    return GET_USART_DMA_CH(1);
#endif
#ifndef USART_DISABLE_USART2
  case 2:
    return GET_USART_DMA_CH(2);
#endif
#ifdef  USART_ENABLE_USART3
  case 3:
    return GET_USART_DMA_CH(3);
#endif
  default:
        return NULL;
  }
}
/*
 * 串口发送等待内联函数
 */
static inline void _usart_tx_wait(DMA_Channel_TypeDef *chn)
{
  while(chn->CNDTR);
}

static uint32_t _getDMATCFlg(int id)
{
  switch (id)
  {
#ifndef USART_DISABLE_USART1
  case 1:
    return GET_USART_DMA_FLG(1);
#endif
#ifndef USART_DISABLE_USART2
  case 2:
    return GET_USART_DMA_FLG(2);
#endif
#ifdef  USART_ENABLE_USART3
  case 3:
    return GET_USART_DMA_FLG(3);
#endif
  default:
    return 0;
  }
}


#define DEF_INIT_DMA_FOR_ID(x) \
  case x:  \
    { \
      DMA_DeInit(GET_USART_DMA_CH(x)); \
      DMA_Init(GET_USART_DMA_CH(x), &DMA_InitStructure); \
    } \
    break

/*
 * 串口关闭函数
 * 释放串口和绑定的DMA通道
 */
void usart_shutdown(USART_TypeDef* USARTx)
{
    DMA_Channel_TypeDef * dma_chn;
    int portid;

    // shutdown the dma...
    USART_DMACmd(USARTx, USART_DMAReq_Tx, DISABLE);
    USART_DeInit(USARTx);

    portid = _getPortID(USARTx);
    if (portid == -1) return;


    dma_chn = _getDMACh(portid);
    DMA_DeInit(dma_chn);
}
/*
 * 串口打开函数
 * 绑定DMA通道用于发送
 */
int usart_begin(USART_TypeDef* USARTx, uint32_t baud)
{
  USART_InitTypeDef desc;

  int portid = _getPortID(USARTx);

  if (portid == -1) return 0;

  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 0;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USARTx->DR;


  switch (portid)
  {
#ifndef USART_DISABLE_USART1
    DEF_INIT_DMA_FOR_ID(1);
#endif
#ifndef USART_DISABLE_USART2
    DEF_INIT_DMA_FOR_ID(2);
#endif
#ifdef  USART_ENABLE_USART3
    DEF_INIT_DMA_FOR_ID(3);
#endif
  }

  USART_StructInit(&desc);
  desc.USART_BaudRate = baud;

  USART_Init(USARTx, &desc);

  USARTx->SR = 0;
  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
  USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
  USART_Cmd(USARTx, ENABLE);
  ring_buffer_t *rx_buffer = _rx_ringbuf+_getBufID(portid);
  rx_buffer->head = rx_buffer->tail = 0;

  return 1;
}

/*
 * 串口发送等待函数
 */
void usart_tx_wait(USART_TypeDef* USARTx)
{
   int id = _getPortID(USARTx);
   if (id == -1) return;

   _usart_tx_wait(_getDMACh(id));
   while (DMA_GetFlagStatus(_getDMATCFlg(id)) == RESET);
}
/*
 * 串口发送数据设置函数
 */
int usart_txbuffer_set(USART_TypeDef* USARTx, const void * src, uint32_t size, uint32_t pos)
{
  DMA_Channel_TypeDef * dma_chn;
  int portid;

  if (pos>=USART_MAX_TX_COUNT) return 0;
  if (size>USART_MAX_TX_COUNT-pos) size = USART_MAX_TX_COUNT-pos;

  portid = _getPortID(USARTx);
  if (portid == -1) return 0;

  dma_chn = _getDMACh(portid);
  _usart_tx_wait(dma_chn);

  memcpy(_GET_TX_BUF(portid) + pos, src, size);

  return size;
}

/*
 * 串口发送数据函数
 * 将串口发送缓存区中数据全发出去
 */
void usart_txbuffer_flush(USART_TypeDef* USARTx, uint32_t size)
{
  DMA_Channel_TypeDef * dma_chn;
  int portid;
  portid = _getPortID(USARTx);
  if (portid == -1) return;

  dma_chn = _getDMACh(portid);

  DMA_Cmd(dma_chn, DISABLE);
  DMA1->IFCR = _getDMATCFlg(portid);
  dma_chn->CNDTR = size;
  dma_chn->CMAR = (uint32_t)_GET_TX_BUF(portid);

  DMA_Cmd(dma_chn, ENABLE);
}
/*
 * 串口发送函数
 * 1、设置发送数据
 * 2、将串口发送缓存区中数据全发出去
 */
int usart_tx(USART_TypeDef* USARTx, const void * data, uint32_t size)
{
  if (size==0) size = strlen((char *)data);
  size = usart_txbuffer_set(USARTx, data, size, 0);
  usart_txbuffer_flush(USARTx, size);
  return size;
}
/*
 * 串口发送单个字符函数
 */
int usart_tx_putc(USART_TypeDef* USARTx, int c)
{
  return usart_tx(USARTx, &c, 1);
}
/*
 * 串口清空接收缓存区函数
 */
void usart_recv_flush(USART_TypeDef* USARTx)
{
  int portid = _getPortID(USARTx);
  if (portid == -1) return;

  ring_buffer_t *rx_buffer = _rx_ringbuf+_getBufID(portid);
  rx_buffer->head = rx_buffer->tail = 0;
  USARTx->SR = 0;
}
/*
 * 串口是否有接收数据判定函数
 */
uint32_t usart_recv_avail(USART_TypeDef* USARTx)
{
  int portid = _getPortID(USARTx);
  if (portid == -1) return 0;

  ring_buffer_t *rx_buffer = _rx_ringbuf+_getBufID(portid);
  return (USART_RX_BUFFER_SIZE +
          rx_buffer->head - rx_buffer->tail) % USART_RX_BUFFER_SIZE;
}
/*
 * 串口接收获取数据函数
 */
size_t usart_recv_gets(USART_TypeDef* USARTx, unsigned char * buf, size_t size)
{
    size_t actual_get = 0;
    while (size--) {
        int current =  usart_recv_getc( USARTx);
        if (current == -1) break;
        buf[actual_get++] = (unsigned char)current;
    }

    return actual_get;
}
/*
 * 串口接收获取单个字符函数
 */
int usart_recv_getc(USART_TypeDef* USARTx)
{
  int portid = _getPortID(USARTx);
  if (portid == -1) return -1;

  ring_buffer_t *rx_buffer = _rx_ringbuf+_getBufID(portid);
  if (rx_buffer->head == rx_buffer->tail) {
    return -1;
  } else {
    unsigned char c = rx_buffer->buffer[rx_buffer->tail];
    rx_buffer->tail = (rx_buffer->tail + 1) % USART_RX_BUFFER_SIZE;
    return c;
  }
}
/*
 * 设置串口中断接收回调函数
 */
void usart_setrecv_func(USART_TypeDef* USARTx, proc_on_rx_t proc)
{
  int portid = _getPortID(USARTx);
  if (portid == -1) return;

  if (proc==NULL) proc =_def_on_rx;
  _rx_proc[_getBufID(portid)] = proc;
}
/*
 * 获取串口中断接收回调函数
 */
proc_on_rx_t usart_getrecv_func(USART_TypeDef* USARTx)
{
  int portid = _getPortID(USARTx);
  if (portid == -1) return NULL;
  return _rx_proc[_getBufID(portid)];
}
//------------------------------


#define DEF_RX_HANDLER_FOR_ID(id) \
void USART##id##_IRQHandler(void)  \
{  \
  if(USART_GetITStatus(USART##id, USART_IT_RXNE) != RESET)  \
  { \
    /* Read one byte from the receive data register */  \
      int c = USART##id->DR & 0xFF; \
      _rx_proc[USART##id##_BUF_ID](id, c); \
  } \
  if(USART_GetFlagStatus(USART##id,USART_FLAG_ORE)==SET) \
    /* Overflow? */ \
  { \
      USART##id->SR = (uint16_t)~USART_FLAG_ORE; \
      int c = USART##id->DR; \
  } \
}

#ifndef USART_DISABLE_USART1
// USART1 Int handler
DEF_RX_HANDLER_FOR_ID(1)
#endif
#ifndef USART_DISABLE_USART2
// USART2 Int handler
DEF_RX_HANDLER_FOR_ID(2)
#endif
#ifdef  USART_ENABLE_USART3
// USART2 Int handler
DEF_RX_HANDLER_FOR_ID(3)
#endif

/*
 * 默认串口中断接收回调函数
 */
static void _def_on_rx(int id, uint32_t data)
{
  ring_buffer_t *rx_buffer = _rx_ringbuf+_getBufID(id);
  int i = (rx_buffer->head + 1) % USART_RX_BUFFER_SIZE;

  if (i != rx_buffer->tail) {
    rx_buffer->buffer[rx_buffer->head] = data;
    rx_buffer->head = i;
  }
}
