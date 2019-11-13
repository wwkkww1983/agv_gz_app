
#include "tim.h"

static void basic_tim_nvic_init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		
    NVIC_InitStructure.NVIC_IRQChannel = BASIC_TIM_IRQ ;	
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 14;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void basic_tim_init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	basic_tim_nvic_init();
	
	BASIC_TIM_APBxClock_FUN(BASIC_TIM_CLK, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = BASIC_TIM_Period;
	TIM_TimeBaseStructure.TIM_Prescaler = BASIC_TIM_Prescaler;
	TIM_TimeBaseInit(BASIC_TIM, &TIM_TimeBaseStructure);
	
	TIM_ClearFlag(BASIC_TIM, TIM_FLAG_Update);
	TIM_ITConfig(BASIC_TIM,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(BASIC_TIM, ENABLE);
}
