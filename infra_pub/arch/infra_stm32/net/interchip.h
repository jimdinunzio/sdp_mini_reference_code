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

// infra code
#include "net/common.h"
#include "net/basic_net.h"

#define  INTERCHIP_RX_ENABLE_LONGFRAME
#define  INTERCHIP_TX_ENABLE_LONGFRAME

typedef struct net_pkt_desc
{
    _u16 size;
    _u8  checksum;
} net_pkt_desc_t;

typedef struct _infra_channel_desc infra_channel_desc_t;
typedef void (*interchip_rx_proc_t)(infra_channel_desc_t * handle, void * data, size_t size);

struct _infra_channel_desc {
    _u8 * rxbuffer;
    void * _medialayer_data;

    void (* register_rx_callback)(infra_channel_desc_t * handle, interchip_rx_proc_t proc);
    int  (* tx_buffer_set) ( infra_channel_desc_t * handle, const void * data, size_t size, size_t  offset);
    void (* tx_flush)(infra_channel_desc_t * handle, size_t size);

    _u16 rxbuffersize;
    _u16 txbuffersize;
    volatile _u32  rx_state;
    _u16 rx_pos;
    _u8  rx_checksum;
};


#ifdef INTERCHIP_RX_ENABLE_LONGFRAME
_u8   net_get_request_cmd(infra_channel_desc_t * channel);
_u16  net_get_request_size(infra_channel_desc_t * channel);
_u8 * net_get_request_data(infra_channel_desc_t * channel);
#else
#define net_get_request_cmd(channel)  ((lit_pkt_header_t *)channel->rxbuffer)->_cmd
#define net_get_request_size(channel) ((lit_pkt_header_t *)channel->rxbuffer)->_len
#define net_get_request_data(channel) ((_u8 *)channel->rxbuffer + sizeof(lit_pkt_header_t))
#endif

static inline _u16 net_get_payload_size(infra_channel_desc_t * channel) {
    return    net_get_request_size(channel) -1;
}

#define _request_cmd         net_get_request_cmd(channel)
#define _request_len         net_get_request_size(channel)
#define _request_data        net_get_request_data(channel)

int net_bind(infra_channel_desc_t* channel);
int net_send_pkt(infra_channel_desc_t* channel ,_u8 cmd, const void * data, _u16 size);
int net_send_errorcode(infra_channel_desc_t* channel , _u16 code);
int net_send_ans(infra_channel_desc_t* channel , const void * data, _u16 size);
bool net_poll_request(infra_channel_desc_t* channel);
void net_prepare_ans(infra_channel_desc_t* channel ,net_pkt_desc_t * pkt_desc);
void net_prepare_pkt(infra_channel_desc_t* channel ,_u8 cmd, net_pkt_desc_t * pkt_desc);
int  net_pkt_pushdata(infra_channel_desc_t* channel ,net_pkt_desc_t * pkt_desc, const void * data, _u16 size);
int  net_flush_pkt(infra_channel_desc_t* channel ,net_pkt_desc_t * pkt_desc);

static inline size_t net_get_max_payload_size(size_t framesize)
{
    return framesize - sizeof(long_pkt_header_t) - 1;
}