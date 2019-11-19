#include "exti.h"
//#include "led.h"
//#include "delay.h"
//#include "usart.h"
#include "stm32f10x_exti.h"

EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

//
void Exti_Enable(void)
{
	NVIC_InitStructure.NVIC_IRQChannel = SPI_EXT_1_INT_EXTI_IRQ_CHL;			
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;	
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = SPI_EXT_2_INT_EXTI_IRQ_CHL;			
  	NVIC_Init(&NVIC_InitStructure);
}

void Exti_Disable(void)
{
	NVIC_InitStructure.NVIC_IRQChannel = SPI_EXT_1_INT_EXTI_IRQ_CHL;			
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;	
  	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = SPI_EXT_2_INT_EXTI_IRQ_CHL;			
  	NVIC_Init(&NVIC_InitStructure);
}

static void NVIC_Configuration(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	// spi-1 irq
	NVIC_InitStructure.NVIC_IRQChannel = SPI_EXT_1_INT_EXTI_IRQ_CHL;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// spi-2 irq
	NVIC_InitStructure.NVIC_IRQChannel = SPI_EXT_2_INT_EXTI_IRQ_CHL;
	NVIC_Init(&NVIC_InitStructure);
}

void exti_init(void)
{
	/* ���� NVIC �ж�*/
	NVIC_Configuration();

	// spi1�ӵ�wk2168��irq�ź�����
	/* ѡ��EXTI���ź�Դ */
	GPIO_EXTILineConfig(SPI_EXT_1_EXTI_PORTSOURCE, SPI_EXT_1_EXTI_PINSOURCE);

	/* �����ж� */
	EXTI_InitStructure.EXTI_Line = SPI_EXT_1_INT_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// spi2�ӵ�wk2168��irq�ź�����
	/* ѡ��EXTI���ź�Դ */
	GPIO_EXTILineConfig(SPI_EXT_2_EXTI_PORTSOURCE, SPI_EXT_2_EXTI_PINSOURCE);

	/* �����ж� */
	EXTI_InitStructure.EXTI_Line = SPI_EXT_2_INT_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}
/*********************************************END OF FILE**********************/

