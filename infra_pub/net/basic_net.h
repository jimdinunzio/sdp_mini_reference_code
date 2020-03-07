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

#ifndef _INFRA_BASIC_NET_H
#define _INFRA_BASIC_NET_H

#include "net/common.h"

#define MAX_LIT_PKT_SIZE   (256 + 4)
#define MAX_LIT_CMDPKT_SIZE 128
#define MAX_STD_CMDPKT_SIZE 255

#if defined(_WIN32) || defined(__ICCARM__) || defined(__CC_ARM)
#pragma pack(1)
#endif

//checksum enabled
#define LIT_PKT_CMD_FLAG   0x10 
#define LONG_PKT_CMD_FLAG  0x50 

/////////////////////////////////////////////////////////////////
//Pre-defined Packets

typedef struct lit_pkt_header
{
  frame_flag_t  _flag;
  _u8           _len;
  _u8           _cmd;
}  __attribute__((packed)) lit_pkt_header_t;


typedef struct long_pkt_header
{
  frame_flag_t  _flag;
  _u16          _len16;
  _u8           _cmd;
}  __attribute__((packed)) long_pkt_header_t;


typedef union LitFrame
{
    struct rawData
    {
        unsigned char buf[MAX_LIT_CMDPKT_SIZE];
    } __attribute__((packed)) _rawdata;

    struct LitPkt
    {
        lit_pkt_header_t _header;
        unsigned char _data[MAX_LIT_CMDPKT_SIZE - sizeof(lit_pkt_header_t)];
    } __attribute__((packed)) _litpkt;

} LitFrame_t;

typedef union StdFrame
{
    struct rawData_std
    {
        unsigned char buf[MAX_STD_CMDPKT_SIZE];
    } __attribute__((packed)) _rawdata;

    struct StdPkt
    {
        lit_pkt_header_t _header;
        unsigned char _data[MAX_STD_CMDPKT_SIZE - sizeof(lit_pkt_header_t)];
    } __attribute__((packed)) _stdpkt;
    
} StdFrame_t;

#if defined(_WIN32) || defined(__ICCARM__) || defined(__CC_ARM)
#pragma pack()
#endif

static inline _u8 litframe_get_payload_len ( LitFrame_t * frame)
{
    return frame->_litpkt._header._len - 1;
}

#endif
