#include "time.h"

//uint8_t isConnect = 0;


void TIM6_Int_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);        //ʱ��ʹ��

    TIM_TimeBaseStructure.TIM_Period = arr;     //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ         ������5000Ϊ500ms
    TIM_TimeBaseStructure.TIM_Prescaler = psc;  //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;        //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);     //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    TIM_ITConfig(               //ʹ�ܻ���ʧ��ָ����TIM�ж�
                    TIM6,       //TIM2
                    TIM_IT_Update, ENABLE       //ʹ��
        );
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;     //TIM3�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //��ռ���ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�3��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);     //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

    TIM_Cmd(TIM6, ENABLE);      //ʹ��TIMx����

}

//int32_t speedL = 0;
//int32_t speedR = 0;

uint8_t timecnt = 0;
uint8_t flag1ms = 0;
uint8_t flag100ms = 0;

/*
void TIM6_IRQHandler(void)   //TIM6�ж�
{
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)                            //���ָ����TIM�жϷ������:TIM �ж�Դ 
  {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update  );                               //���TIMx���жϴ�����λ:TIM �ж�Դ 

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
    if (1 == flag100ms)         //50ms �ٶȿ���
    {
        flag100ms = 0;

        walkingmotor_SpeedDetect();     //��ȡ��ǰ�ٶ�

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

        //GPIOB->ODR ^= ((1 << 10) | (1 << 11));//BOTTOM_IR_E FRONT_IR_E ������⴫���� ��� 500hzPWM
        //GPIOB->ODR ^= ((1 << 10));
        _delay_us(100);
        IR_Get_distance();
        //GPIOE->ODR &= ~((1 << 0));
    }
/**/}
