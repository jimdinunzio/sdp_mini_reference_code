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
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)

#include "common/common.h"
#include "irdecoder.h"
     
enum decode_states_t {
    IR_DECODE_STATE_WAIT_LEADING_HIGH = 0,
    IR_DECODE_STATE_WAIT_LEADING_LOW  = 1,
    IR_DECODE_STATE_WAIT_BIT_HIGH     = 2,
    IR_DECODE_STATE_WAIT_BIT_LOW      = 3,
};

#define IRDECODE_DURATION_TOLERANT_uS     200
#define IRDECODE_DURATION_LEADING_HIGH    8900
#define IRDECODE_DURATION_LEADING_LOW     4600
#define IRDECODE_DURATION_BIT_HIGH        560
#define IRDECODE_DURATION_BIT_0_LOW       560
#define IRDECODE_DURATION_BIT_1_LOW       1650

#define IRDECODE_PAYLOAD_BIT_COUNT        7

static void _irdecoder_on_data_ready(irdecoder_context_t * context, _u32 data, _u32 ts)
{
    proc_on_decode_ready_t proc = (proc_on_decode_ready_t) context->callback_proc;
    if (proc) proc(context, data, ts);
}

void irdecoder_init(irdecoder_context_t * context, proc_on_decode_ready_t callback )
{
    irdecoder_reset(context);
    context->callback_proc = (void *)callback;
}

void irdecoder_reset(irdecoder_context_t * context)
{
    context->state = 0;
    context->decodePos = 0;
    context->decodeBuffer = 0;
    context->lastTS = 0;
}

static int _withinDuration(_u32 src, _u32 target, _u32 error)
{
    if ( (src >= target - error) && (src <= target + error) ) {
        return 1;
    } else {
        return 0;
    }
}

static inline void _switchToState(irdecoder_context_t * context, int newState)
{
    if (newState == IR_DECODE_STATE_WAIT_LEADING_LOW) {
        // clear pending buffer..
        context->decodeBuffer = 0;
        context->decodePos = 0;
    }
    context->state = newState;
}

void irdecoder_on_signal(irdecoder_context_t * context, int currentLvl)
{
    _u32 currentMicrosec = getus();    
    _u32 lastTS = context->lastTS;
    
    
    switch (context->state) {
    case IR_DECODE_STATE_WAIT_LEADING_HIGH:
        if (currentLvl) {
            _switchToState(context, IR_DECODE_STATE_WAIT_LEADING_LOW);
        }
        break;
    
    case IR_DECODE_STATE_WAIT_LEADING_LOW:
        if (currentLvl == 0) {
            if (_withinDuration(currentMicrosec - lastTS, IRDECODE_DURATION_LEADING_HIGH, IRDECODE_DURATION_TOLERANT_uS))
            {
                _switchToState(context, IR_DECODE_STATE_WAIT_BIT_HIGH);
                break;
            }
        } 
        // state mismatch, revert
        _switchToState(context, IR_DECODE_STATE_WAIT_LEADING_HIGH);
        
        break;
        
    case IR_DECODE_STATE_WAIT_BIT_HIGH:
        if (currentLvl) {
            
            if (context->decodePos == 0) {
                // leading bit...
                if (_withinDuration(currentMicrosec - lastTS, IRDECODE_DURATION_LEADING_LOW, IRDECODE_DURATION_TOLERANT_uS))
                {
                    _switchToState(context, IR_DECODE_STATE_WAIT_BIT_LOW);
                    break; 
                } else {
                }
            } else {
                // data bit
                
                if (_withinDuration(currentMicrosec - lastTS, IRDECODE_DURATION_BIT_0_LOW, IRDECODE_DURATION_TOLERANT_uS))
                {
                    context->decodeBuffer <<= 1;
                    _switchToState(context, IR_DECODE_STATE_WAIT_BIT_LOW);
                    break;
                } 
                
                if (_withinDuration(currentMicrosec - lastTS, IRDECODE_DURATION_BIT_1_LOW, IRDECODE_DURATION_TOLERANT_uS))
                {
                    context->decodeBuffer <<= 1;
                    context->decodeBuffer |= 0x1;
                    _switchToState(context, IR_DECODE_STATE_WAIT_BIT_LOW);
                    break;
                }              
            }
        } 
        
        // state mismatch, revert
        _switchToState(context, IR_DECODE_STATE_WAIT_LEADING_HIGH);
        break;
        
    case IR_DECODE_STATE_WAIT_BIT_LOW:
        if (currentLvl == 0) {

            if (_withinDuration(currentMicrosec - lastTS, IRDECODE_DURATION_BIT_HIGH, IRDECODE_DURATION_TOLERANT_uS))
            {
                
                if ( context->decodePos++ >= IRDECODE_PAYLOAD_BIT_COUNT ) 
                {
                    // data received
                    _irdecoder_on_data_ready(context, context->decodeBuffer, currentMicrosec);
                    
                    // ready for recv a next frame...
                    _switchToState(context, IR_DECODE_STATE_WAIT_LEADING_HIGH);
                } else {
                    
                    _switchToState(context, IR_DECODE_STATE_WAIT_BIT_HIGH);
                }
               
                break; 
            } else {
                
                // is that a leading frame?
                if (_withinDuration(currentMicrosec - lastTS, IRDECODE_DURATION_LEADING_HIGH, IRDECODE_DURATION_TOLERANT_uS))
                {
                    // may be it is...
                    context->decodePos = 0;
                    _switchToState(context, IR_DECODE_STATE_WAIT_BIT_HIGH);                   
                    break;
                }
           
            }
        }
        // state mismatch, revert
        _switchToState(context, IR_DECODE_STATE_WAIT_LEADING_HIGH);
        break;

    default:
        _switchToState(context, IR_DECODE_STATE_WAIT_LEADING_HIGH);
    }
    
    context->lastTS = currentMicrosec;
}

void irdecoder_on_idle_tick(irdecoder_context_t * context)
{
    
}
#endif
