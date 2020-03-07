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
#include "net/interchip.h"

#define PKT_STATE_NULL  0x00
#define PKT_STATE_READY 0x80
#define PKT_STATE_CHECKSUM_FAIL PKT_ERRORCODE_CHECKSUM_FAIL
#define PKT_STATE_SIZE_OVERFLOW PKT_ERRORCODE_SIZE_OVERFLOW
#define PKT_STATE_NOT_SYNC_PKT  PKT_ERRORCODE_NOT_SYNC_PKT

#define PKT_STATE_WAIT_LEN     (1)
#define PKT_STATE_WAIT_LEN_EX  (2)
#define PKT_STATE_WAIT_DATA_AND_CHK    (3)
#define PKT_STATE_WAIT_DATA    (4)

static void _interchip_on_rx(infra_channel_desc_t * handle, void * data_arr, size_t size)
{
    infra_channel_desc_t * desc = (infra_channel_desc_t *)handle;

    while (size -- ) {
        _u8  data = *((_u8 *)data_arr);
        data_arr = ((_u8 *)data_arr) + 1;

        _u32 cached_state = desc->rx_state >>16;

        if ( cached_state & PKT_STATE_READY)
        {
            desc->rx_state |= PKT_STATE_NOT_SYNC_PKT;
            alert();
            return ;
        }

        if ( desc->rx_pos == 0 )
        {
            if ( data == LIT_PKT_CMD_FLAG
    #ifdef INTERCHIP_RX_ENABLE_LONGFRAME
                ||  data == LONG_PKT_CMD_FLAG
    #endif
                )
            {
                desc->rx_checksum = 0;
                cached_state = PKT_STATE_WAIT_LEN;
            } else {
                desc->rx_state |= PKT_STATE_NOT_SYNC_PKT;
                alert();
                return;
            }
        }
        else
        {
            switch (cached_state) {

            case PKT_STATE_WAIT_LEN:
    #ifdef INTERCHIP_RX_ENABLE_LONGFRAME
            case PKT_STATE_WAIT_LEN_EX:
                if ( desc->rxbuffer[0] == LONG_PKT_CMD_FLAG && cached_state==PKT_STATE_WAIT_LEN) {
                    ++cached_state;
                    break;
                }
    #endif
                cached_state = PKT_STATE_WAIT_DATA_AND_CHK;
                break;
            case PKT_STATE_WAIT_DATA_AND_CHK:
                if ( net_get_request_size(desc) + desc->rx_pos > desc->rxbuffersize)
                {
                   desc->rx_state |= PKT_STATE_SIZE_OVERFLOW;
                   desc->rx_pos = 0;
                   alert();
                   return;
                }
                ++ cached_state;
            case PKT_STATE_WAIT_DATA:

                if (desc->rx_pos == net_get_request_size(desc) + ( (desc->rxbuffer[0] == LONG_PKT_CMD_FLAG)?3:2))
                {
                    if (data != desc->rx_checksum)
                    {
                        desc->rx_state |= PKT_STATE_CHECKSUM_FAIL;
                    }
                    desc->rx_state |= PKT_STATE_READY;
                    desc->rx_pos = 0;
                    alert();
                    return;
                }
                break;
            default :
                desc->rx_pos = 0;
                return;
            }
        }
        desc->rx_checksum ^=(uint8_t)(data & 0xFF);
        desc->rxbuffer[desc->rx_pos++] = data;
        desc->rx_state = (cached_state<<16) | (desc->rx_state & 0xFFFF);
    }
}

int net_bind(infra_channel_desc_t * channel)
{
    if (!channel->rxbuffer || !channel->rxbuffersize
        || !channel->register_rx_callback
        || !channel->tx_buffer_set
        || !channel->tx_flush) return 0;

    channel->rx_state = 0;
    channel->rx_pos = 0;
    channel->rx_checksum = 0;

    channel->register_rx_callback(channel, &_interchip_on_rx);
    return 1;
}

void net_prepare_pkt(infra_channel_desc_t * channel ,_u8 cmd, net_pkt_desc_t * pkt_desc)
{
    _u8 txheader[4];

#ifdef INTERCHIP_TX_ENABLE_LONGFRAME
    txheader[0] = LONG_PKT_CMD_FLAG;
    pkt_desc->size = 1 + 2 + 1; // flag + 16bit len + command
#else
    txheader[0] = LIT_PKT_CMD_FLAG;
    pkt_desc->size = 1 + 1 + 1; // flag + 8bit len + command
#endif
    pkt_desc->checksum = txheader[0] ^ cmd;
    txheader[pkt_desc->size-1] = cmd;
    channel->tx_buffer_set(channel, &txheader, pkt_desc->size, 0);
}

int  net_pkt_pushdata(infra_channel_desc_t * channel, net_pkt_desc_t * pkt_desc, const void * data, _u16 size)
{
    if (pkt_desc->size+size > channel->txbuffersize -1) {
        size = channel->txbuffersize -1 - pkt_desc->size;
    }

    channel->tx_buffer_set(channel, data, size, pkt_desc->size);
    pkt_desc->size += size;

    for (_u16 pos = 0; pos < size; ++pos) {
        pkt_desc->checksum ^= ((_u8 *)data)[pos];
    }
    return size;
}

int  net_flush_pkt(infra_channel_desc_t * channel, net_pkt_desc_t * pkt_desc)
{
    _u16 length_field;
#ifdef INTERCHIP_TX_ENABLE_LONGFRAME
    length_field = pkt_desc->size - 3;
    channel->tx_buffer_set(channel, &length_field, 2, 1);

    pkt_desc->checksum ^= ((_u8)(length_field & 0xFF)) ^ ((_u8)(length_field >> 8));
#else
    length_field = pkt_desc->size - 2;
    channel->tx_buffer_set(channel, &length_field, 1, 1);
    pkt_desc->checksum ^= (_u8)(length_field);
#endif

    channel->tx_buffer_set(channel, &pkt_desc->checksum, 1, pkt_desc->size);
    channel->tx_flush(channel, pkt_desc->size+1);

    return pkt_desc->size+1;
}

int net_send_pkt(infra_channel_desc_t * channel, _u8 cmd, const void * data, _u16 size)
{
    net_pkt_desc_t pkt;
    net_prepare_pkt(channel, cmd, &pkt);
    net_pkt_pushdata(channel, &pkt, data, size);
    return net_flush_pkt(channel, &pkt);
}

int net_send_errorcode(infra_channel_desc_t * channel, _u16 code)
{
    channel->rx_state = PKT_STATE_NULL;
    return net_send_pkt(channel, STATUS_CODE_ANS_RXERR, (unsigned char *)&code, sizeof(code));
}

int net_send_ans(infra_channel_desc_t * channel, const void * data, _u16 size)
{
    channel->rx_state = PKT_STATE_NULL;
    return net_send_pkt(channel, STATUS_CODE_ANS, (unsigned char *)data, size);
}

void net_prepare_ans(infra_channel_desc_t * channel, net_pkt_desc_t * pkt_desc)
{
    channel->rx_state = PKT_STATE_NULL;
    net_prepare_pkt(channel, STATUS_CODE_ANS, pkt_desc);
}

bool net_poll_request(infra_channel_desc_t* channel)
{
    clear_alert();
    if (channel->rx_state & (PKT_STATE_NOT_SYNC_PKT | PKT_STATE_CHECKSUM_FAIL | PKT_STATE_SIZE_OVERFLOW) )
    {
        net_send_errorcode(channel , channel->rx_state & 0xFFFF);
    }
    else
    {
        if (channel->rx_state & PKT_STATE_READY)
        {
            _u8 cmd = net_get_request_cmd(channel);
            if (cmd == STATUS_CODE_SYNC
                || cmd == STATUS_CODE_ECHO)
            {
                _u8 old_checksum = channel->rx_checksum;

                channel->rx_state = PKT_STATE_NULL;

                _u32 pkt_size = net_get_request_size(channel) + ((channel->rxbuffer[0] == LONG_PKT_CMD_FLAG )?3:2);

                channel->tx_buffer_set(channel, (_u8 *)channel->rxbuffer, pkt_size, 0);
                channel->tx_buffer_set(channel, &old_checksum, 1, pkt_size);
                channel->tx_flush(channel, pkt_size+1 );

            }else
            {
                return true;
            }
        }
    }
    return false;
}


#ifdef INTERCHIP_RX_ENABLE_LONGFRAME

_u8   net_get_request_cmd(infra_channel_desc_t * channel)
{
    if (channel->rxbuffer[0] == LONG_PKT_CMD_FLAG ) {
        return  ((long_pkt_header_t *)channel->rxbuffer)->_cmd;
    } else {
        return  ((lit_pkt_header_t *)channel->rxbuffer)->_cmd;
    }
}

_u16  net_get_request_size(infra_channel_desc_t * channel)
{
    if (channel->rxbuffer[0] == LONG_PKT_CMD_FLAG ) {
        return  ((long_pkt_header_t *)channel->rxbuffer)->_len16;
    } else {
        return  ((lit_pkt_header_t *)channel->rxbuffer)->_len;
    }
}

_u8 * net_get_request_data(infra_channel_desc_t * channel)
{
    if (channel->rxbuffer[0] == LONG_PKT_CMD_FLAG ) {
        return  (_u8 *)channel->rxbuffer + sizeof(long_pkt_header_t);
    } else {
        return  (_u8 *)channel->rxbuffer + sizeof(lit_pkt_header_t);
    }
}

#endif