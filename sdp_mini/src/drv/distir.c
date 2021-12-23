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
#include "distir.h"

#define IR_CARRIER_FREQ  400000                                 //单位：hz
#define IR_CARRIER_EMIT_WIDTH 20                                //单位：ms
#define IR_SENSOR_DROP_THRESHOLD  100                           //跌落判定阈值
#define IR_SENSOR_MAX_ADC_DIFF    3500
#define IRPWM_TIMER_PERIOD (CPU_FREQ/IR_CARRIER_FREQ -1)        //PWM周期
#define IRPWM_TIMER_FRONT_PW      (IRPWM_TIMER_PERIOD/10)       //前红外占空比
#define IRPWM_TIMER_BOTTOM_PW     (IRPWM_TIMER_PERIOD/10)       //底下红外占空比

#define IR_FRONT_BLOCK_DISTANCE  30                             //单位：mm

enum IR_WORKING_STATUS_T {
    STATUS_ADC_IDLE = 0,
    STATUS_ADC_PENDING = 1,
    STATUS_WAIT = 2,
};

enum IR_SAMPLE_STAGE_T {
    STATUS_IR_SAMPLE_BACKGROUND = 0,
    STATUS_IR_SAMPLE_LOADED = 1,
};

typedef struct _adcSamplePair {
    _u16 bitPos;
    _u16 adcPin;
    _s16 lastAdcval;
    _s16 lastAdcdiff;
} adcSamplePair_t;

#define IRSENSOR_FRONT_R1_BIT    (0x1<<IRSENSOR_FRONT_R1_ID)
#define IRSENSOR_FRONT_R2_BIT    (0x1<<IRSENSOR_FRONT_R2_ID)
#define IRSENSOR_FRONT_R3_BIT    (0x1<<IRSENSOR_FRONT_R3_ID)
#define IRSENSOR_FRONT_R4_BIT    (0x1<<IRSENSOR_FRONT_R4_ID)

#define IRSENSOR_BOTTOM_BIT_BASE 8

#define IRSENSOR_BOTTOM_R1_BIT   (0x1<<(IRSENSOR_BOTTOM_R1_ID+IRSENSOR_BOTTOM_BIT_BASE))
#define IRSENSOR_BOTTOM_R2_BIT   (0x1<<(IRSENSOR_BOTTOM_R2_ID+IRSENSOR_BOTTOM_BIT_BASE))
#define IRSENSOR_BOTTOM_R3_BIT   (0x1<<(IRSENSOR_BOTTOM_R3_ID+IRSENSOR_BOTTOM_BIT_BASE))
#define IRSENSOR_BOTTOM_R4_BIT   (0x1<<(IRSENSOR_BOTTOM_R4_ID+IRSENSOR_BOTTOM_BIT_BASE))

static adcSamplePair_t _adcSamplePairs[] = {
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
#else
    {IRSENSOR_FRONT_R1_BIT, FRONT_IR_R1_ADC_CH},
    {IRSENSOR_FRONT_R2_BIT, FRONT_IR_R2_ADC_CH},
    {IRSENSOR_FRONT_R3_BIT, FRONT_IR_R3_ADC_CH},
    {IRSENSOR_FRONT_R4_BIT, FRONT_IR_R4_ADC_CH},
#endif
    {IRSENSOR_BOTTOM_R1_BIT, BOTTOM_IR_R1_ADC_CH},
    {IRSENSOR_BOTTOM_R2_BIT, BOTTOM_IR_R2_ADC_CH},
    {IRSENSOR_BOTTOM_R3_BIT, BOTTOM_IR_R3_ADC_CH},
    {IRSENSOR_BOTTOM_R4_BIT, BOTTOM_IR_R4_ADC_CH},
};

static _u8 _currentSamplePos;
static irDistance_t _cachedIrDistance;
static _u32 _cachedIrresult;

static _u8 _irWorkingStatus;
static _u8 _irSampleStage;
static _u32 _irLastSampleTime;

static _s32 init_dev_irdetector();
static void heartbeat_dev_irdetector();
static _u32 read_dev_irdetector();
/*
 * 红外测距初始化函数
 */
void init_distir(void)
{
    init_dev_irdetector();
    memset(&_cachedIrDistance, 0, sizeof(_cachedIrDistance));
    _cachedIrDistance.bottomSensorBitmap = (_u32) (-1);
}
/*
 * 红外测距处理函数
 */
void heartbeat_distir(void)
{
    heartbeat_dev_irdetector();
    _u32 currentBitmap = read_dev_irdetector();
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
#else
    for (size_t pos = 0; pos <= _countof(_cachedIrDistance.frontSensorDistMmQ16); ++pos) {
        int isBlocked = !(currentBitmap & (0x1 << pos));
        if (isBlocked) {
            int relativeDistEst = (int) _adcSamplePairs[pos].lastAdcdiff * IR_FRONT_BLOCK_DISTANCE / IR_SENSOR_MAX_ADC_DIFF;
            if (relativeDistEst >= IR_FRONT_BLOCK_DISTANCE)
                relativeDistEst = IR_FRONT_BLOCK_DISTANCE - 1;
            _cachedIrDistance.frontSensorDistMmQ16[pos] = ((IR_FRONT_BLOCK_DISTANCE - relativeDistEst) << 16);
        } else {
            _cachedIrDistance.frontSensorDistMmQ16[pos] = 0;
        }
    }
#endif
    _cachedIrDistance.bottomSensorBitmap = 0xFF;
    for (size_t pos = IRSENSOR_BOTTOM_R1_ID; pos <= IRSENSOR_BOTTOM_R4_ID; ++pos) {
        int isDropDetected = (currentBitmap & (0x1 << (pos + IRSENSOR_BOTTOM_BIT_BASE)));
        if (isDropDetected) {
            _cachedIrDistance.bottomSensorBitmap &= ~(0x1 << pos);
        }
    }
}
/*
 * 获得红外测距数据函数
 */
const irDistance_t *get_distir_value()
{
    return &_cachedIrDistance;
}
/*
 * 红外测距PWM发送初始化函数
 */
static int init_dev_irdetector()
{

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    memset(&TIM_TimeBaseStructure, 0, sizeof(TIM_TimeBaseStructure));
    memset(&TIM_OCInitStructure, 0, sizeof(TIM_OCInitStructure));

    TIM_TimeBaseStructure.TIM_Period = IRPWM_TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_DeInit(GET_TIM(IR_EMITTER_TIMER_ID));
    TIM_TimeBaseInit(GET_TIM(IR_EMITTER_TIMER_ID), &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0;

#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
#else
    INIT_OC_FOR(FRONT_IR_E_TIMER_CH, IR_EMITTER_TIMER_ID);
#endif
    INIT_OC_FOR(BOTTOM_IR_E_TIMER_CH, IR_EMITTER_TIMER_ID);

    TIM_Cmd(GET_TIM(IR_EMITTER_TIMER_ID), ENABLE);

#if   IR_EMITTER_TIMER_ID==1
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
#endif

#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
#else
    pinMode(FRONT_IR_E_PORT, FRONT_IR_E_PIN, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    pinMode(FRONT_IR_R1_PORT, FRONT_IR_R1_PIN, GPIO_Mode_AIN, GPIO_Speed_50MHz);
    pinMode(FRONT_IR_R2_PORT, FRONT_IR_R2_PIN, GPIO_Mode_AIN, GPIO_Speed_50MHz);
    pinMode(FRONT_IR_R3_PORT, FRONT_IR_R3_PIN, GPIO_Mode_AIN, GPIO_Speed_50MHz);
    pinMode(FRONT_IR_R4_PORT, FRONT_IR_R4_PIN, GPIO_Mode_AIN, GPIO_Speed_50MHz);
#endif

    pinMode(BOTTOM_IR_E_PORT, BOTTOM_IR_E_PIN, GPIO_Mode_AF_PP, GPIO_Speed_50MHz);
    pinMode(BOTTOM_IR_R1_PORT, BOTTOM_IR_R1_PIN, GPIO_Mode_AIN, GPIO_Speed_50MHz);
    pinMode(BOTTOM_IR_R2_PORT, BOTTOM_IR_R2_PIN, GPIO_Mode_AIN, GPIO_Speed_50MHz);
    pinMode(BOTTOM_IR_R3_PORT, BOTTOM_IR_R3_PIN, GPIO_Mode_AIN, GPIO_Speed_50MHz);
    pinMode(BOTTOM_IR_R4_PORT, BOTTOM_IR_R4_PIN, GPIO_Mode_AIN, GPIO_Speed_50MHz);
    _irWorkingStatus = STATUS_ADC_IDLE;
    _irSampleStage = STATUS_IR_SAMPLE_BACKGROUND;
    _cachedIrresult = 0;
    _irLastSampleTime = 0;
    _currentSamplePos = 0;
    return 1;
}
/*
 * Infrared ranging ADC training sampling function 
 */
static void heartbeat_dev_irdetector()
{
    switch (_irWorkingStatus) {
    case STATUS_WAIT:                                                           //Wait for the infrared PWM sending time before sampling. Generally, PWM is issued after the background value is collected 
        {
            if (getms() - _irLastSampleTime > IR_CARRIER_EMIT_WIDTH) {
                _irWorkingStatus = STATUS_ADC_IDLE;
            }
        }
        break;
    case STATUS_ADC_IDLE:                                                       //When sampling is possible, the ADC sampling channels are switched in turn, and each channel is the infrared receiving point 
        {
            _u16 adcPin = _adcSamplePairs[_currentSamplePos].adcPin;
            adc_read_start(GET_ADC(IR_SENSOR_SAMPLE_ADC), adcPin);
            _irWorkingStatus = STATUS_ADC_PENDING;
        }
        break;
    case STATUS_ADC_PENDING:
        {
            if (adc_read_is_ready(GET_ADC(IR_SENSOR_SAMPLE_ADC))) {             //After sampling, save the sampled value 
                int result = adc_read_final(GET_ADC(IR_SENSOR_SAMPLE_ADC));

                if (_irSampleStage == STATUS_IR_SAMPLE_BACKGROUND) {            //Sampling and storing background ADC values 
                    _adcSamplePairs[_currentSamplePos].lastAdcval = result;

                } else {                                                        //Sampling and storing ranging values 
                    _u16 relatedBit = _adcSamplePairs[_currentSamplePos].bitPos;
                    int adcDiff = _adcSamplePairs[_currentSamplePos].lastAdcval - result;
                    if (adcDiff > IR_SENSOR_DROP_THRESHOLD) {
                        _cachedIrresult &= ~relatedBit;
                        _adcSamplePairs[_currentSamplePos].lastAdcdiff = adcDiff;

                    } else {
                        _cachedIrresult |= relatedBit;
                    }
                }
                if (++_currentSamplePos >= _countof(_adcSamplePairs)) {
                    _currentSamplePos = 0;
                    if (_irSampleStage == STATUS_IR_SAMPLE_BACKGROUND) {        //一Round sampling is over, if it is the end of background sampling, infrared PWM will be emitted for ranging 
#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
#else
                        RAW_PWM_SET(FRONT_IR_E_TIMER_CH, IR_EMITTER_TIMER_ID, IRPWM_TIMER_FRONT_PW);
#endif
                        RAW_PWM_SET(BOTTOM_IR_E_TIMER_CH, IR_EMITTER_TIMER_ID, IRPWM_TIMER_BOTTOM_PW);
                        _irSampleStage = STATUS_IR_SAMPLE_LOADED;

                    } else {                                                    //一Round sampling is over, if it is the end of ranging sampling, stop sending infrared PWM and wait for the next round of background sampling 

#if defined(CONFIG_BREAKOUT_REV) && (CONFIG_BREAKOUT_REV >= 3)
#else
                        RAW_PWM_SET(FRONT_IR_E_TIMER_CH, IR_EMITTER_TIMER_ID, 0);
#endif
                        RAW_PWM_SET(BOTTOM_IR_E_TIMER_CH, IR_EMITTER_TIMER_ID, 0);

                        _irSampleStage = STATUS_IR_SAMPLE_BACKGROUND;
                    }
                    _irWorkingStatus = STATUS_WAIT;
                    _irLastSampleTime = getms();
                } else {
                    _irWorkingStatus = STATUS_ADC_IDLE;
                }

            }

        }
        break;
    }
}
/*
 * 获得红外测距的碰撞状态函数
 */
static _u32 read_dev_irdetector()
{
    return _cachedIrresult;
}
