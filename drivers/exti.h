#ifndef __EXTI_H
#define	__EXTI_H


#include "stm32f10x.h"
#include "gpio.h"


//引脚定义
#define SPI_EXT_1_EXTI_PORTSOURCE   GPIO_PortSourceGPIOA
#define SPI_EXT_1_EXTI_PINSOURCE    GPIO_PinSource8
#define SPI_EXT_1_INT_EXTI_LINE     EXTI_Line8
#define SPI_EXT_1_INT_EXTI_IRQ_CHL  EXTI9_5_IRQn
#define SPI_EXT_1_IRQHandler        EXTI9_5_IRQHandler

#define SPI_EXT_2_EXTI_PORTSOURCE   GPIO_PortSourceGPIOD
#define SPI_EXT_2_EXTI_PINSOURCE    GPIO_PinSource10
#define SPI_EXT_2_INT_EXTI_LINE     EXTI_Line10
#define SPI_EXT_2_INT_EXTI_IRQ_CHL  EXTI15_10_IRQn
#define SPI_EXT_2_IRQHandler        EXTI15_10_IRQHandler



void exti_init(void);//外部中断初始化		 
void Exti_Enable(void);
void Exti_Disable(void);


#endif /* __EXTI_H */


