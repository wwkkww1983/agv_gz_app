#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "stm32f10x.h"

#define            BASIC_TIM                   TIM6
#define            BASIC_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            BASIC_TIM_CLK               RCC_APB1Periph_TIM6
#define            BASIC_TIM_Period            (10000-1)   // 1/10000 s x 10000次 = 1s
#define            BASIC_TIM_Prescaler         (7200 - 1)  // 72000000/7200 = 10000hz
#define            BASIC_TIM_IRQ               TIM6_IRQn
#define            BASIC_TIM_IRQHandler        TIM6_IRQHandler


void basic_tim_init(void);

#endif

