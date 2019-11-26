/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "FreeRTOS.h"
#include "uart.h"
#include "service.h"
#include "applications.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

extern TaskHandle_t h_ext_wifi_entry;
void USART_1_IRQHandler(void)
{
	uint8_t data;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	uint32_t ulReturn;
	ulReturn = taskENTER_CRITICAL_FROM_ISR();	
	
	if( USART_GetITStatus( usart1->def.port_uart, USART_IT_RXNE )!= RESET ) 
	{
		data = USART_ReceiveData( usart1->def.port_uart );
		uart_save_from_isr( usart1, data );	
	}
	else if ( USART_GetITStatus( usart1->def.port_uart, USART_IT_IDLE ) != RESET )
	{
		data = usart1->def.port_uart->SR; 
		data = usart1->def.port_uart->DR;

		xTaskNotifyFromISR( h_ext_wifi_entry, RX_IDLE_BIT, eSetBits, &xHigherPriorityTaskWoken );
	}
	
	if ( xHigherPriorityTaskWoken == pdTRUE )
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	taskEXIT_CRITICAL_FROM_ISR( ulReturn );
}

extern TaskHandle_t h_cmd_entry;
void UART4_IRQHandler()
{
    uint8_t data;
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// 关中断
	uint32_t ulReturn;
	ulReturn = taskENTER_CRITICAL_FROM_ISR();
	
	if( USART_GetITStatus( UART4, USART_IT_RXNE )!=RESET ) 
	{
		data = USART_ReceiveData(UART4);
		uart_save_from_isr( uart4, data );
	}
	else if ( USART_GetITStatus( uart4->def.port_uart, USART_IT_IDLE ) != RESET )
	{
		data = uart4->def.port_uart->SR; // 清除idle位，先读SR，再读DR
		data = uart4->def.port_uart->DR;
		
		xTaskNotifyFromISR( h_cmd_entry, RX_IDLE_BIT, eSetBits, &xHigherPriorityTaskWoken );
	}
	
	if ( xHigherPriorityTaskWoken == pdTRUE )
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	
	// 开中断
	taskEXIT_CRITICAL_FROM_ISR( ulReturn );
}

extern TaskHandle_t h_touch_screen_entry;
void USART2_IRQHandler(void)
{
	uint8_t data;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// ¹ØÖÐ¶Ï
	uint32_t ulReturn;
	ulReturn = taskENTER_CRITICAL_FROM_ISR();	
	
	if( USART_GetITStatus( usart2->def.port_uart, USART_IT_RXNE )!= RESET ) 
	{
		data = USART_ReceiveData( usart2->def.port_uart );
		uart_save_from_isr( usart2, data );	
	}
	else if ( USART_GetITStatus( usart2->def.port_uart, USART_IT_IDLE ) != RESET )
	{
		data = usart2->def.port_uart->SR; // Çå³ýidleÎ»£¬ÏÈ¶ÁSR£¬ÔÙ¶ÁDR
		data = usart2->def.port_uart->DR;
		
		xTaskNotifyFromISR( h_touch_screen_entry, RX_IDLE_BIT, eSetBits, &xHigherPriorityTaskWoken );
	}
	
	if ( xHigherPriorityTaskWoken == pdTRUE )
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	
	// ¿ªÖÐ¶Ï
	taskEXIT_CRITICAL_FROM_ISR( ulReturn );
}

#include "exti.h"
#include "SPI_WK2168.h"

// spi1的wk2168接收中断
void SPI_EXT_1_IRQHandler(void)
{
	uint8_t gifr, rxlen;
	//确保是否产生了EXTI Line中断
	if(EXTI_GetITStatus(SPI_EXT_1_INT_EXTI_LINE) != RESET) 
	{
		//清除中断标志位
		EXTI_ClearITPendingBit(SPI_EXT_1_INT_EXTI_LINE);     
// 关闭中断
Exti_Disable();
//__disable_irq();
//		printf("\r\n\r\nin spi1 exti....");
		gifr=WkReadGReg(SPI1, WK2XXX_GIFR);/**/
		do{
			if(gifr&WK2XXX_UT1INT)//判断子串口1是否有中断
			{ /*数据处理*/
						/*数据接收*/
//						rxlen=wk_RxChars(SPI1, 1,rxbuf);//一次接收的数据不会超过256Byte
						rxlen=cache_bms_rx( SPI1, 1 );
//						printf("\r\nspi1, chl1, rx_len:%d", rxlen);
//						/*数据发送*/
//						//把接收的数据发送出去
//						wk_TxChars(1,rxlen,rxbuf);

			}
			
			if(gifr&WK2XXX_UT2INT)//判断子串口2是否有中断
			{
				/*数据接收*/
//						rxlen=wk_RxChars(SPI1, 2,rxbuf);//一次接收的数据不会超过256Byte
						rxlen=cache_bms_rx( SPI1, 2 );
//						printf("\r\nspi1, chl2, rx_len:%d", rxlen);
						/*数据发送*/
//						//把接收的数据发送出去
//						wk_TxChars(2,rxlen,rxbuf);
			
			  }
			if(gifr&WK2XXX_UT3INT)//判断子串口3是否有中断
			{
				/*数据接收*/
//						rxlen=wk_RxChars(SPI1, 3,rxbuf);//一次接收的数据不会超过256Byte
						rxlen=cache_bms_rx( SPI1, 3 );
//						printf("\r\nspi1, chl3, rx_len:%d", rxlen);
						/*数据发送*/
						//把接收的数据发送出去
//						wk_TxChars(3,rxlen,rxbuf);
				   // printf("port!!!!\n");
			}
			if(gifr&WK2XXX_UT4INT)//判断子串口4是否有中断
			{
				/*数据接收*/
//						rxlen=wk_RxChars(SPI1, 4,rxbuf);//一次接收的数据不会超过256Byte
						rxlen=cache_bms_rx( SPI1, 4 );
//						printf("\r\nspi1, chl4, rx_len:%d", rxlen);
						/*数据发送*/
						//把接收的数据发送出去
				   //wk_TxChars(4,rxlen,rxbuf);
			}
			
		gifr=WkReadGReg(SPI1, WK2XXX_GIFR);
		//printf("IN EXTI2_IRQ GIFR:0X%X !!!\n",gifr);
		}while(gifr&0x0f);
// 打开中断
Exti_Enable();
//__enable_irq();		
//		printf("\r\n...spi1 exti over! !!!\n");
	} 
}

// spi2的wk2168接收中断
void SPI_EXT_2_IRQHandler(void)
{
	uint8_t gifr, rxlen;
	//确保是否产生了EXTI Line中断
	if(EXTI_GetITStatus(SPI_EXT_2_INT_EXTI_LINE) != RESET) 
	{
		//清除中断标志位
		EXTI_ClearITPendingBit(SPI_EXT_2_INT_EXTI_LINE);     
// 关闭中断
Exti_Disable();

//		printf("\r\n\r\nin spi2 exti....");
		gifr=WkReadGReg(SPI2, WK2XXX_GIFR);/**/
		do{
			if(gifr&WK2XXX_UT1INT)//判断子串口1是否有中断
			{ /*数据处理*/
						/*数据接收*/
//						rxlen=wk_RxChars(SPI2, 1,rxbuf);//一次接收的数据不会超过256Byte
						rxlen=cache_bms_rx( SPI2, 1 );
//						printf("\r\nspi2, chl1, rx_len:%d", rxlen);
//						/*数据发送*/
//						//把接收的数据发送出去
//						wk_TxChars(1,rxlen,rxbuf);

			}
			
			if(gifr&WK2XXX_UT2INT)//判断子串口2是否有中断
			{
				/*数据接收*/
//						rxlen=wk_RxChars(SPI2, 2,rxbuf);//一次接收的数据不会超过256Byte
						rxlen=cache_bms_rx( SPI2, 2 );
//						printf("\r\nspi2, chl2, rx_len:%d", rxlen);
						/*数据发送*/
//						//把接收的数据发送出去
//						wk_TxChars(2,rxlen,rxbuf);
			
			  }
			if(gifr&WK2XXX_UT3INT)//判断子串口3是否有中断
			{
				/*数据接收*/
//						rxlen=wk_RxChars(SPI2, 3,rxbuf);//一次接收的数据不会超过256Byte
						rxlen=cache_bms_rx( SPI2, 3 );
//						printf("\r\nspi2, chl3, rx_len:%d", rxlen);
						/*数据发送*/
						//把接收的数据发送出去
//						wk_TxChars(3,rxlen,rxbuf);
				   // printf("port!!!!\n");
			}
			if(gifr&WK2XXX_UT4INT)//判断子串口4是否有中断
			{
				/*数据接收*/
//						rxlen=wk_RxChars(SPI2, 4,rxbuf);//一次接收的数据不会超过256Byte
						rxlen=cache_bms_rx( SPI2, 4 );
//						printf("\r\nspi2, chl4, rx_len:%d", rxlen);
						/*数据发送*/
						//把接收的数据发送出去
				   //wk_TxChars(4,rxlen,rxbuf);
			}
			
		gifr=WkReadGReg(SPI2, WK2XXX_GIFR);
		//printf("IN EXTI2_IRQ GIFR:0X%X !!!\n",gifr);
		}while(gifr&0x0f);

// 打开中断
Exti_Enable();
//		printf("\r\n...spi2 exti over! !!!\n");
	} 
}

#include "can.h"

void CAN_RX_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	// ¹ØÖÐ¶Ï
	uint32_t ulReturn;
	ulReturn = taskENTER_CRITICAL_FROM_ISR();
	
    if(CAN_GetITStatus(CAN1,CAN_IT_FF0) != RESET) 
	{ 
		CAN_ClearITPendingBit(CAN1,CAN_IT_FF0); 
	} 

	else if(CAN_GetITStatus(CAN1,CAN_IT_FF1) != RESET) 
	{ 
		CAN_ClearITPendingBit(CAN1,CAN_IT_FF1); 
	} 
	else 
	{
		sCanRxISR(CAN_FIFO0);
		
		xTaskNotifyFromISR( h_can_comm_entry, CAN_RX_BIT, eSetBits, &xHigherPriorityTaskWoken );
		
		if ( xHigherPriorityTaskWoken == pdTRUE )
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
	
	taskEXIT_CRITICAL_FROM_ISR( ulReturn );	
}



/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
