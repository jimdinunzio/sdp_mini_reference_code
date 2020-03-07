#include "time.h"

//uint8_t isConnect = 0;


void TIM6_Int_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);        //时钟使能

    TIM_TimeBaseStructure.TIM_Period = arr;     //设置在下一个更新事件装入活动的自动重装载寄存器周期的值         计数到5000为500ms
    TIM_TimeBaseStructure.TIM_Prescaler = psc;  //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;        //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);     //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    TIM_ITConfig(               //使能或者失能指定的TIM中断
                    TIM6,       //TIM2
                    TIM_IT_Update, ENABLE       //使能
        );
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;     //TIM3中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);     //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

    TIM_Cmd(TIM6, ENABLE);      //使能TIMx外设

}

//int32_t speedL = 0;
//int32_t speedR = 0;

uint8_t timecnt = 0;
uint8_t flag1ms = 0;
uint8_t flag100ms = 0;

/*
void TIM6_IRQHandler(void)   //TIM6中断
{
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)                            //检查指定的TIM中断发生与否:TIM 中断源 
  {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update  );                               //清除TIMx的中断待处理位:TIM 中断源 

    flag1ms = 1;
    if(timecnt ++ >= 77)
    {
      timecnt = 0;
      flag100ms = 1;
    }
  }
}
*/
void perform_heartbeat(void)
{
    if (1 == flag100ms)         //50ms 速度控制
    {
        flag100ms = 0;

        walkingmotor_SpeedDetect();     //获取当前速度

        /*if((isOntheground()) || (0 == isConnect))
           {
           speedL = walkingmotor_GetL_SpeedSet();
           speedR = walkingmotor_GetR_SpeedSet();
           walkingmotor_Brake();
           walkingmotor_SpeedSet(speedL, speedR);
           }
           else */
        {
            walkingmotor_SpeedControl();
        }

        battery_heartbeat();
    }

    if (1 == flag1ms) {
        flag1ms = 0;
        //GPIOE->ODR |= ((1 << 0));

        //GPIOB->ODR ^= ((1 << 10) | (1 << 11));//BOTTOM_IR_E FRONT_IR_E 距离红外传感器 输出 500hzPWM
        //GPIOB->ODR ^= ((1 << 10));
        _delay_us(100);
        IR_Get_distance();
        //GPIOE->ODR &= ~((1 << 0));
    }
/**/}
