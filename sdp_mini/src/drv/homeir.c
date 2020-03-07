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
#include "homeir.h"
#include "irdecoder.h"

#define TOWERID_MAIN_BEACON      0
#define TOWERID_LEFT_BEACON      1
#define TOWERID_RIGHT_BEACON     2

#define TOWERID_MAIN_PROBE       0
#define TOWERID_LEFT_PROBE       1
#define TOWERID_RIGHT_PROBE      2

#define TOWERLOCATOR_BEACON_MAIN    12
#define TOWERLOCATOR_BEACON_LEFT    80
#define TOWERLOCATOR_BEACON_RIGHT   2

#define TOWERLOCATOR_BEACON_EPOCH_DURATION  (130000UL*2) //us

#define IR_SENSOR_TOWER_INT_PORT GPIO_PortSourceGPIOD
#define IR_SENSOR_TOWER1_INTLINE 14
#define IR_SENSOR_TOWER2_INTLINE 13
#define IR_SENSOR_TOWER3_INTLINE 12

#define IR_SENSOR_TOWER1_PORT     GPIOD
#define IR_SENSOR_TOWER2_PORT     GPIOD
#define IR_SENSOR_TOWER3_PORT     GPIOD

#define IR_SENSOR_TOWER1_PIN      GPIO_Pin_14
#define IR_SENSOR_TOWER2_PIN      GPIO_Pin_13
#define IR_SENSOR_TOWER3_PIN      GPIO_Pin_12

#define IR_SENSOR_TOWER_IRQn    EXTI15_10_IRQn

typedef struct _irdecoder_data_cache_t {
    _u8  beaconbit;
    _u32 beaconLastTs[3];
} irdecoder_data_cache_t;

static  irdecoder_context_t    decoder_ctx[3];
static  irdecoder_data_cache_t beacon_cache[3];


#ifdef _IR_TOWER_RAW_DEBUG
static _u32 _dumpBuffer[64];
static _u8  _dumpBits[64];
static _u8 _dumpInitial;
static _u8 _dumpState;
static _u8 _dumpPos;
#endif

static void _on_beacon_decode_ready (irdecoder_context_t * context, _u32 data, _u32 ts);

int drv_towerlocator_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);                          //外部中断，需要使能AFIO时钟

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);                         //使能PORTA,PORTC时钟
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;//P
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                                 

    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    EXTI_InitTypeDef EXTI_InitStructure;
    
    GPIO_EXTILineConfig(IR_SENSOR_TOWER_INT_PORT, GET_EXTINT_PIN(IR_SENSOR_TOWER1_INTLINE));
    GPIO_EXTILineConfig(IR_SENSOR_TOWER_INT_PORT, GET_EXTINT_PIN(IR_SENSOR_TOWER2_INTLINE));
    GPIO_EXTILineConfig(IR_SENSOR_TOWER_INT_PORT, GET_EXTINT_PIN(IR_SENSOR_TOWER3_INTLINE));
    
    // Configure EXTI lines
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    // for precision, EXTI_Trigger_Rising_Falling can be used. But it brings more
    //  overheads
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    
    EXTI_InitStructure.EXTI_Line = GET_EXTINT_LINE(IR_SENSOR_TOWER1_INTLINE);
    EXTI_Init(&EXTI_InitStructure);
    
    EXTI_InitStructure.EXTI_Line = GET_EXTINT_LINE(IR_SENSOR_TOWER2_INTLINE);
    EXTI_Init(&EXTI_InitStructure);
    
    EXTI_InitStructure.EXTI_Line = GET_EXTINT_LINE(IR_SENSOR_TOWER3_INTLINE);
    EXTI_Init(&EXTI_InitStructure);   
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = IR_SENSOR_TOWER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);  
  
    
    memset(beacon_cache, 0, sizeof(beacon_cache));
           
    for (size_t pos = 0; pos < _countof(decoder_ctx); ++pos)
    {
        decoder_ctx[pos].userData = pos;
        irdecoder_init(&decoder_ctx[pos], _on_beacon_decode_ready);
    }

    return 1;
}



void drv_towerlocator_shutdown(void)
{
    // disable EXIT
    EXTI_InitTypeDef EXTI_InitStructure;
    // Configure EXTI lines
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    // for precision, EXTI_Trigger_Rising_Falling can be used. But it brings more
    //  overheads
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    
    EXTI_InitStructure.EXTI_Line = GET_EXTINT_LINE(IR_SENSOR_TOWER1_INTLINE);
    EXTI_Init(&EXTI_InitStructure);
    
    EXTI_InitStructure.EXTI_Line = GET_EXTINT_LINE(IR_SENSOR_TOWER2_INTLINE);
    EXTI_Init(&EXTI_InitStructure);
    
    EXTI_InitStructure.EXTI_Line = GET_EXTINT_LINE(IR_SENSOR_TOWER3_INTLINE);
    EXTI_Init(&EXTI_InitStructure);    
    
}


static void _update_beacon_epoch(int probeID, _u32 current_uS)
{
    _u32 irqsave = enter_critical_section();
    for (size_t pos = 0; pos < _countof(beacon_cache[probeID].beaconLastTs); ++pos)
    {
        _u8 currentbit = (0x1<<pos);
        if (beacon_cache[probeID].beaconbit & currentbit) {
            if ( (beacon_cache[probeID].beaconLastTs[pos] < current_uS) && (current_uS - beacon_cache[probeID].beaconLastTs[pos] >= TOWERLOCATOR_BEACON_EPOCH_DURATION))
            {
                beacon_cache[probeID].beaconbit &= ~currentbit;
                beacon_cache[probeID].beaconLastTs[pos] = current_uS;       
            }
        }
    }  
    leave_critical_section(irqsave);
}

void drv_towerlocator_heartbeat(void)
{
    // timeout check...

    for (size_t pos = 0; pos < _countof(decoder_ctx); ++pos)
    {
        irdecoder_on_idle_tick(&decoder_ctx[pos]);
        
    }
    
    _u32 current_uS = getus();
    
    for (size_t pos = 0; pos < _countof(beacon_cache); ++pos) {
        _update_beacon_epoch(pos, current_uS);
    }

    
#ifdef _IR_TOWER_BEACON_DEBUG
    static _u32 lastts = 0;
    
    if (getms() - lastts > 300) {
        lastts = getms();
        
        printf("%02x %02x %02x\r\n",  (_u32)beacon_cache[0].beaconbit,  (_u32)beacon_cache[1].beaconbit,  (_u32)beacon_cache[2].beaconbit);
    }
    
#endif
    
#ifdef _IR_TOWER_RAW_DEBUG    
    if (_dumpState) {
        printf("Initial %d\r\n", (int)_dumpInitial);
        
        for (size_t pos = 0; pos < _countof(_dumpBuffer); ++pos)
        {
            drv_watchdog_mark();
            printf("#%d %lu %lu %d\r\n", pos, _dumpBuffer[pos], pos?(_dumpBuffer[pos] - _dumpBuffer[pos-1]):0, (int)_dumpBits[pos]);
        }
        _dumpState = 0;
    }
#endif
}

_u32 drv_towerlocator_get_beacon(int id)
{
    return beacon_cache[id].beaconbit;
}

_u32 drv_towerlocator_get_beacon_timestamp(int id, int beacon)
{
    if (id < _countof(beacon_cache) && beacon < _countof(beacon_cache[id].beaconLastTs)) {
        return beacon_cache[id].beaconLastTs[beacon];
    }
    return 0;
}

static void _on_signal_swap(int id, int currentLvl)
{
#ifdef _IR_TOWER_RAW_DEBUG    
    if (id == 0) {
        if (_dumpState == 0) {
            if (_dumpPos == 0) {
                _dumpInitial = currentLvl;
            }
            _dumpBits[_dumpPos] = currentLvl?1:0;
            _dumpBuffer[_dumpPos++] = getus();
            
            if (_dumpPos == _countof(_dumpBuffer)) {
                _dumpPos = 0;
                _dumpState = 1;
            }
        }
    }
#endif
    
    irdecoder_on_signal(&decoder_ctx[id], currentLvl?0:1); //received signal level is inversed
}

// interrupt handler

#if IR_SENSOR_TOWER_IRQn!=EXTI15_10_IRQn
#error "IR_SENSOR_TOWER_IRQn!=EXTI15_10_IRQn"
#endif


void EXTI15_10_IRQHandler(void)
{
    //check which pin has signaled the interrupt
    
    if (EXTI_GetITStatus(GET_EXTINT_LINE(IR_SENSOR_TOWER1_INTLINE))!= RESET) {
        EXTI_ClearITPendingBit(GET_EXTINT_LINE(IR_SENSOR_TOWER1_INTLINE));
        _on_signal_swap(TOWERID_MAIN_PROBE, PIN_READ(IR_SENSOR_TOWER1_PORT, IR_SENSOR_TOWER1_PIN));
    }
    
    if (EXTI_GetITStatus(GET_EXTINT_LINE(IR_SENSOR_TOWER2_INTLINE))!= RESET) {
        EXTI_ClearITPendingBit(GET_EXTINT_LINE(IR_SENSOR_TOWER2_INTLINE));
        _on_signal_swap(TOWERID_LEFT_PROBE, PIN_READ(IR_SENSOR_TOWER2_PORT, IR_SENSOR_TOWER2_PIN));
    }

    if (EXTI_GetITStatus(GET_EXTINT_LINE(IR_SENSOR_TOWER3_INTLINE))!= RESET) {
        EXTI_ClearITPendingBit(GET_EXTINT_LINE(IR_SENSOR_TOWER3_INTLINE));
        _on_signal_swap(TOWERID_RIGHT_PROBE, PIN_READ(IR_SENSOR_TOWER3_PORT, IR_SENSOR_TOWER3_PIN));
    }    
}


static void _on_beacon_decode_ready (irdecoder_context_t * context, _u32 data, _u32 ts)
{   
    switch (data) {
    case TOWERLOCATOR_BEACON_MAIN:
        beacon_cache[context->userData].beaconbit |= (0x1<<TOWERID_MAIN_BEACON);
        beacon_cache[context->userData].beaconLastTs[TOWERID_MAIN_BEACON] = ts;
        
        break;
    case TOWERLOCATOR_BEACON_LEFT:
        beacon_cache[context->userData].beaconbit |= (0x1<<TOWERID_LEFT_BEACON);
        beacon_cache[context->userData].beaconLastTs[TOWERID_LEFT_BEACON] = ts;        
        break;
        
    case TOWERLOCATOR_BEACON_RIGHT:
        beacon_cache[context->userData].beaconbit |= (0x1<<TOWERID_RIGHT_BEACON);
        beacon_cache[context->userData].beaconLastTs[TOWERID_RIGHT_BEACON] = ts;
        break;
        
    }
    
#if 0
    if (context->userData == 0) {
        char buffer[10];
        sprintf(buffer, "%d\r\n", data);
        usart_tx(GET_USART(1), buffer, strlen(buffer));
    }
#endif
}


void homeir_Init(void)
{
  drv_towerlocator_init();
}
void homeir_heartbeat(void)
{
  drv_towerlocator_heartbeat();
}

uint8_t homeir_getmaindata(void)
{
  return drv_towerlocator_get_beacon(TOWERID_MAIN_PROBE);
}

uint8_t homeir_getleftdata(void)
{
  return drv_towerlocator_get_beacon(TOWERID_LEFT_PROBE);
}

uint8_t homeir_getrightdata(void)
{
  return drv_towerlocator_get_beacon(TOWERID_RIGHT_PROBE);
}


#else

#include "homeir.h"

uint8_t homeir_data[3] = {0};

void homeir_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;  

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能PORTB时钟 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//TIM5 时钟使能 
  
  GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		//上拉输入 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);


  TIM_TimeBaseStructure.TIM_Period = 60000; //设定计数器自动重装值 最大10ms溢出  
  TIM_TimeBaseStructure.TIM_Prescaler =(72-1); 	//预分频器,1M的计数频率,1us加1.	   
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;  // 选择输入端 IC3映射到TI4上
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM_ICInitStructure.TIM_ICFilter = 0x03;//IC4F=0011 配置输入滤波器 8个定时器时钟周期滤波
  TIM_ICInit(TIM4, &TIM_ICInitStructure);//初始化定时器输入捕获通道
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);

  TIM_Cmd(TIM4, ENABLE ); 	//使能定时器

  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
  NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器	

  TIM_ITConfig(TIM4, TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3, ENABLE);//允许更新中断 ,允许CC2IE捕获中断
  //TIM_ITConfig(TIM4, TIM_IT_Update|TIM_IT_CC1, ENABLE);
}

uint8_t HOME_IR_R1_Data = 0;
uint8_t HOME_IR_R1_KeyVal = 0;
uint8_t HOME_IR_R1_Flag = 0;
uint32_t TIMOC1_val = 0;
uint32_t TIMOC1_Data = 0;
uint8_t TIMOC1_sta = 0;

uint8_t HOME_IR_R2_Data = 0;
uint8_t HOME_IR_R2_KeyVal = 0;
uint8_t HOME_IR_R2_Flag = 0;
uint32_t TIMOC2_val = 0;
uint32_t TIMOC2_Data = 0;
uint8_t TIMOC2_sta = 0;

uint8_t HOME_IR_R3_Data = 0;
uint8_t HOME_IR_R3_KeyVal = 0;
uint8_t HOME_IR_R3_Flag = 0;
uint32_t TIMOC3_val = 0;
uint32_t TIMOC3_Data = 0;
uint8_t TIMOC3_sta = 0;
uint8_t TIMUpdateFlag = 0;
void TIM4_IRQHandler(void)
{	    	 
  if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    TIM_ClearFlag(TIM4, TIM_IT_Update);
    TIMUpdateFlag = 1;
    //GPIOE->ODR ^= ((1 << 1));

  }
  /**********************************************************************************/
  if(TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
  {
    TIM_ClearFlag(TIM4, TIM_IT_CC1);
    
    if(1 == GPIO_ReadInputDataBit(GPIOD, HOME_IR_R1))//上升沿捕获
    {
      TIMUpdateFlag = 0;
      TIMOC1_val = TIM_GetCapture1(TIM4);

      TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获	
      

    }
    else //下降沿捕获
    {
      
      if(1 == TIMUpdateFlag)
      {
        TIMUpdateFlag = 0;
        TIMOC1_Data = TIM_GetCapture1(TIM4) + 60000 - TIMOC1_val;//读取CCR1也可以清CC1IF标志位
      }
      else/**/
      {
        TIMOC1_Data = TIM_GetCapture1(TIM4) - TIMOC1_val;
      }
      TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Rising); //CC4P=0	设置为上升沿捕获
      
      
      if(TIMOC1_Data > (HOME_IR_START_TIME - HOME_IR_START_ERROR) && TIMOC1_Data < (HOME_IR_START_TIME + HOME_IR_START_ERROR))		//4500为标准值4.5ms  成功接收到了引导码
      {//DBG_OUT("TIMOC1_sta = 0x%x \r\n", TIMOC1_sta);
        if(TIMOC1_sta == HOME_IR_CHARGING_DOCK)
        {
            HOME_IR_R1_Flag = 1;
            HOME_IR_R1_KeyVal = HOME_IR_R1_Data;
        }
        TIMOC1_sta = 0x01;
        HOME_IR_R1_Data = 0;
        
        return;
        /**/
      }
      
      if(TIMOC1_sta != 0)
      {
        
        if(TIMOC1_Data > (HOME_IR_BIT_LOW_TIME - HOME_IR_BIT_LOW_ERROR) && TIMOC1_Data < (HOME_IR_BIT_LOW_TIME + HOME_IR_BIT_LOW_ERROR))			//560为标准值,560us
        {
          TIMOC1_sta <<= 1;
          HOME_IR_R1_Data <<= 1;	//左移一位.
          HOME_IR_R1_Data |= 0;	//接收到0	   
        }
        else if(TIMOC1_Data > (HOME_IR_BIT_HIGH_TIME - HOME_IR_BIT_HIGH_ERROR) && TIMOC1_Data < (HOME_IR_BIT_HIGH_TIME + HOME_IR_BIT_HIGH_ERROR))	//1680为标准值,1680us
        {
          TIMOC1_sta <<= 1;
          HOME_IR_R1_Data <<= 1;	//左移一位.
          HOME_IR_R1_Data |= 1;	//接收到1
        }
        else if(TIMOC1_Data > 2200 && TIMOC1_Data < 2600)	//得到按键键值增加的信息 2500为标准值2.5ms
        {		
        }
        else if(TIMOC1_Data > 4200 && TIMOC1_Data < 4700)		//4500为标准值4.5ms
        {//正常情况这里不会得到引导码
          //TIMOC3_sta |= (1 << 0);
          //HOME_IR_R3_Data = 0;
        }
      }
      else
      {
        
      }
      
    }				 		     	    					   
  }  
  /**********************************************************************************/
  if(TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
  {
    TIM_ClearFlag(TIM4, TIM_IT_CC2);
    
    if(1 == GPIO_ReadInputDataBit(GPIOD, HOME_IR_R2))//上升沿捕获
    {
      TIMUpdateFlag = 0;
      TIMOC2_val = TIM_GetCapture2(TIM4);

      TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获	
      

    }
    else //下降沿捕获
    {
      
      if(1 == TIMUpdateFlag)
      {
        TIMUpdateFlag = 0;
        TIMOC2_Data = TIM_GetCapture2(TIM4) + 60000 - TIMOC2_val;//读取CCR1也可以清CC1IF标志位
      }
      else/**/
      {
        TIMOC2_Data = TIM_GetCapture2(TIM4) - TIMOC2_val;
      }
      TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising); //CC4P=0	设置为上升沿捕获
      
      
      if(TIMOC2_Data > (HOME_IR_START_TIME - HOME_IR_START_ERROR) && TIMOC2_Data < (HOME_IR_START_TIME + HOME_IR_START_ERROR))		//4500为标准值4.5ms  成功接收到了引导码
      {//DBG_OUT("TIMOC1_sta = 0x%x \r\n", TIMOC1_sta);
        if(TIMOC2_sta == HOME_IR_CHARGING_DOCK)
        {
            HOME_IR_R2_Flag = 1;
            HOME_IR_R2_KeyVal = HOME_IR_R2_Data;
        }
        TIMOC2_sta = 0x01;
        HOME_IR_R2_Data = 0;
        
        return;
        /**/
      }
      
      if(TIMOC2_sta != 0)
      {
        
        if(TIMOC2_Data > (HOME_IR_BIT_LOW_TIME - HOME_IR_BIT_LOW_ERROR) && TIMOC2_Data < (HOME_IR_BIT_LOW_TIME + HOME_IR_BIT_LOW_ERROR))			//560为标准值,560us
        {
          TIMOC2_sta <<= 1;
          HOME_IR_R2_Data <<= 1;	//左移一位.
          HOME_IR_R2_Data |= 0;	//接收到0	   
        }
        else if(TIMOC2_Data > (HOME_IR_BIT_HIGH_TIME - HOME_IR_BIT_HIGH_ERROR) && TIMOC2_Data < (HOME_IR_BIT_HIGH_TIME + HOME_IR_BIT_HIGH_ERROR))	//1680为标准值,1680us
        {
          TIMOC2_sta <<= 1;
          HOME_IR_R2_Data <<= 1;	//左移一位.
          HOME_IR_R2_Data |= 1;	//接收到1
        }
        else if(TIMOC2_Data > 2200 && TIMOC2_Data < 2600)	//得到按键键值增加的信息 2500为标准值2.5ms
        {		
        }
        else if(TIMOC2_Data > 4200 && TIMOC2_Data < 4700)		//4500为标准值4.5ms
        {//正常情况这里不会得到引导码
          //TIMOC3_sta |= (1 << 0);
          //HOME_IR_R3_Data = 0;
        }
      }
      else
      {
        
      }
      
    }				 		     	    					   
  }   
  /**********************************************************************************/
  if(TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)
  {
    TIM_ClearFlag(TIM4, TIM_IT_CC3);
    
    if(1 == GPIO_ReadInputDataBit(GPIOD, HOME_IR_R3))//上升沿捕获
    {
      TIMUpdateFlag = 0;
      TIMOC3_val = TIM_GetCapture3(TIM4);

      TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获	
      

    }
    else //下降沿捕获
    {
      
      if(1 == TIMUpdateFlag)
      {
        TIMUpdateFlag = 0;
        TIMOC3_Data = TIM_GetCapture3(TIM4) + 60000 - TIMOC3_val;//读取CCR1也可以清CC1IF标志位
      }
      else/**/
      {
        TIMOC3_Data = TIM_GetCapture3(TIM4) - TIMOC3_val;
      }
      TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising); //CC4P=0	设置为上升沿捕获
      
      
      if(TIMOC3_Data > (HOME_IR_START_TIME - HOME_IR_START_ERROR) && TIMOC3_Data < (HOME_IR_START_TIME + HOME_IR_START_ERROR))		//4500为标准值4.5ms  成功接收到了引导码
      {//DBG_OUT("TIMOC1_sta = 0x%x \r\n", TIMOC1_sta);
        if(TIMOC3_sta == HOME_IR_CHARGING_DOCK)
        {
            HOME_IR_R3_Flag = 1;
            HOME_IR_R3_KeyVal = HOME_IR_R3_Data;
        }
        TIMOC3_sta = 0x01;
        HOME_IR_R3_Data = 0;
        
        return;
        /**/
      }
      
      if(TIMOC3_sta != 0)
      {
        
        if(TIMOC3_Data > (HOME_IR_BIT_LOW_TIME - HOME_IR_BIT_LOW_ERROR) && TIMOC3_Data < (HOME_IR_BIT_LOW_TIME + HOME_IR_BIT_LOW_ERROR))			//560为标准值,560us
        {
          TIMOC3_sta <<= 1;
          HOME_IR_R3_Data <<= 1;	//左移一位.
          HOME_IR_R3_Data |= 0;	//接收到0	   
        }
        else if(TIMOC3_Data > (HOME_IR_BIT_HIGH_TIME - HOME_IR_BIT_HIGH_ERROR) && TIMOC3_Data < (HOME_IR_BIT_HIGH_TIME + HOME_IR_BIT_HIGH_ERROR))	//1680为标准值,1680us
        {
          TIMOC3_sta <<= 1;
          HOME_IR_R3_Data <<= 1;	//左移一位.
          HOME_IR_R3_Data |= 1;	//接收到1
        }
        else if(TIMOC3_Data > 2200 && TIMOC3_Data < 2600)	//得到按键键值增加的信息 2500为标准值2.5ms
        {		
        }
        else if(TIMOC3_Data > 4200 && TIMOC3_Data < 4700)		//4500为标准值4.5ms
        {//正常情况这里不会得到引导码
          //TIMOC3_sta |= (1 << 0);
          //HOME_IR_R3_Data = 0;
        }
      }
      else
      {
        
      }
      
    }	
  }   
 
  /**********************************************************************************/
}

/*
void TIM4_IRQHandler(void)
{ 		    	 
  if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    TIM_ClearFlag(TIM4, TIM_IT_Update);
    TIMUpdateFlag = 1;
    GPIOE->ODR ^= ((1 << 1));
  }
  
  if(TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)
  {
    TIM_ClearFlag(TIM4, TIM_IT_CC3);
    
    if(1 == GPIO_ReadInputDataBit(GPIOD, HOME_IR_R3))//上升沿捕获
    {
      TIMOC3_val = TIM_GetCapture3(TIM4);

      TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获	
      

    }
    else //下降沿捕获
    {
      
      if(1 == TIMUpdateFlag)
      {
        TIMUpdateFlag = 0;
        TIMOC3_Data = TIM_GetCapture3(TIM4) + 60000 - TIMOC3_val;//读取CCR1也可以清CC1IF标志位
        
      }
      else
      {
        TIMOC3_Data = TIM_GetCapture3(TIM4) - TIMOC3_val;
      }
      TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising); //CC4P=0	设置为上升沿捕获
      

      if(TIMOC3_sta != 0)		//4500为标准值4.5ms  成功接收到了引导码
      {
        
        TIMUpdateFlag = 0;
        TIMOC3_sta <<= 1;
        if(TIMOC3_Data > 300 && TIMOC3_Data < 800)			//560为标准值,560us
        {
          HOME_IR_R3_Data <<= 1;	//左移一位.
          HOME_IR_R3_Data |= 0;	//接收到0	   
        }
        else if(TIMOC3_Data > 1400 && TIMOC3_Data < 1800)	//1680为标准值,1680us
        {
          HOME_IR_R3_Data <<= 1;	//左移一位.
          HOME_IR_R3_Data |= 1;	//接收到1
        }
        else if(TIMOC3_Data > 2200 && TIMOC3_Data < 2600)	//得到按键键值增加的信息 2500为标准值2.5ms
        {		
        }
        else if(TIMOC3_Data > 4200 && TIMOC3_Data < 4700)		//4500为标准值4.5ms
        {
          HOME_IR_R3_Data = 0;
          return;
        }
        
      }
      else if(TIMOC3_Data > 4200 && TIMOC3_Data < 4700)		//4500为标准值4.5ms  成功接收到了引导码
      {
        GPIOE->ODR |= ((1 << 0));
        
        TIMOC3_sta |= (1 << 0);
        HOME_IR_R3_Flag = 1;
        HOME_IR_R3_KeyVal = HOME_IR_R3_Data;
        HOME_IR_R3_Data = 0;

        GPIOE->ODR &= ~((1 << 0));
      }
      
    }				 		     	    					   
  }   
}*/

uint8_t homeir_getmaindata(void)
{
    return homeir_data[HOME_IR_MAIN];
}
uint8_t homeir_getleftdata(void)
{
    return homeir_data[HOME_IR_LEFT];
}
uint8_t homeir_getrightdata(void)
{
    return homeir_data[HOME_IR_RIGHT];
}

uint8_t homeir_GetData(void)
{ 
    if(HOME_IR_R1_Flag == 1)
    {
        HOME_IR_R1_Flag = 0;
        //DBG_OUT("HOME_IR_R1 = 0x%x \r\n", HOME_IR_R1_KeyVal);
        homeir_data[HOME_IR_RIGHT] = HOME_IR_R1_KeyVal;
    }

    
    if(HOME_IR_R2_Flag == 1)
    {
        HOME_IR_R2_Flag = 0;
        //DBG_OUT("HOME_IR_R2 = 0x%x \r\n", HOME_IR_R2_KeyVal);
        homeir_data[HOME_IR_LEFT] = HOME_IR_R2_KeyVal;
    }

    
    if(HOME_IR_R3_Flag == 1)
    {
        HOME_IR_R3_Flag = 0;
        //DBG_OUT("HOME_IR_R3 = 0x%x \r\n", HOME_IR_R3_KeyVal);
        homeir_data[HOME_IR_MAIN] = HOME_IR_R3_KeyVal;
    }

  
  return HOME_IR_R3_Data;
}

uint32_t homeir_heartbeat_frequency = 0;
void homeir_heartbeat(void)
{
    if(HOME_IR_R1_Flag == 1)
    {
        HOME_IR_R1_Flag = 0;
        //DBG_OUT("HOME_IR_R1 = 0x%x \r\n", HOME_IR_R1_KeyVal);
        homeir_data[HOME_IR_RIGHT] = HOME_IR_R1_KeyVal;
    }

    
    if(HOME_IR_R2_Flag == 1)
    {
        HOME_IR_R2_Flag = 0;
        //DBG_OUT("HOME_IR_R2 = 0x%x \r\n", HOME_IR_R2_KeyVal);
        homeir_data[HOME_IR_LEFT] = HOME_IR_R2_KeyVal;
    }

    
    if(HOME_IR_R3_Flag == 1)
    {
        HOME_IR_R3_Flag = 0;
        //DBG_OUT("HOME_IR_R3 = 0x%x \r\n", HOME_IR_R3_KeyVal);
        homeir_data[HOME_IR_MAIN] = HOME_IR_R3_KeyVal;
    }
    
    if((getms() - homeir_heartbeat_frequency) >= 100)
    {
        homeir_heartbeat_frequency = getms();
        
        homeir_data[HOME_IR_MAIN] = 0;
        homeir_data[HOME_IR_LEFT] = 0;
        homeir_data[HOME_IR_RIGHT] = 0;
    }
    
        
    //DBG_OUT("%x  ", (uint8_t)homeir_getmaindata());
    //DBG_OUT("%x  ", (uint8_t)homeir_getleftdata());
    //DBG_OUT("%x  \r\n", (uint8_t)homeir_getrightdata());
}

#endif
