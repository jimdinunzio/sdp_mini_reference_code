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
#include "drv/serial_channel.h"

static infra_channel_desc_t _channel;
static interchip_rx_proc_t _rx_proc;

//串口通道的接收缓存区
static _u8 _serialRxBuffer[CONFIG_DRV_SERIALCHANNEL_RXBUFFERSIZE];

/*
 * 串口中断回调函数
 * 串口接收中断里会调用该函数
 */
static void _serial_on_rx_t(int id, _u32 data)
{
    if (_rx_proc) {
        _u8 data_u8 = data;
        _rx_proc(&_channel, &data_u8, 1);
    }

}
/*
 * 注册上层协议函数
 * 指定了该串口通道下对应的上层协议，如interchip
 */
static void _register_rx_callback(infra_channel_desc_t * handle, interchip_rx_proc_t proc)
{
    _rx_proc = proc;
}
/*
 * 串口通道设置数据到发送缓存区函数
 * 调用更底层的环形发送缓存区来缓存
 */
static _s32 _set_tx_buffer(infra_channel_desc_t * handle, const void *data, size_t size, size_t offset)
{
    return usart_txbuffer_set((USART_TypeDef *) _channel._medialayer_data, data, size, offset);
}
/*
 * 串口通道发送函数
 * 调用等底层的发送函数将环形缓存区的数据全部发送出去
 */
static void _flush_tx(infra_channel_desc_t * handle, size_t size)
{
    usart_txbuffer_flush((USART_TypeDef *) _channel._medialayer_data, size);
}
/*
 * 串口初始化函数
 * 指定了接收缓存区，接收中断的回调函数等
 */
_s32 drv_serialchannel_init(USART_TypeDef * USARTx, _u32 baud)
{
    if (!usart_begin(USARTx, baud))
        return 0;

    _rx_proc = NULL;
    _channel.rxbuffer = _serialRxBuffer;
    _channel._medialayer_data = USARTx;
    _channel.register_rx_callback = &_register_rx_callback;
    _channel.tx_buffer_set = &_set_tx_buffer;
    _channel.tx_flush = &_flush_tx;

    _channel.rxbuffersize = sizeof(_serialRxBuffer);
    _channel.txbuffersize = CONFIG_DRV_SERIALCHANNEL_TXBUFFERSIZE;

    usart_setrecv_func(USARTx, _serial_on_rx_t);

    return 1;
}
/*
 * 串口通道关闭函数
 * 取消绑定的中断接收回调函数和该串口通道对应的uart
 */
void drv_serialchannel_shutdown(USART_TypeDef * USARTx)
{
    usart_setrecv_func(USARTx, NULL);
    usart_shutdown(USARTx);
}
/*
 * 获取当前串口通道函数
 */
infra_channel_desc_t *drv_serialchannel_getchannel()
{
    return &_channel;
}
