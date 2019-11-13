#ifndef __CAN_H
#define	__CAN_H

//#include "main.h"
//#include "applications.h"

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

#define    OS_ENTER_CRITICAL     taskENTER_CRITICAL
#define    OS_EXIT_CRITICAL      taskEXIT_CRITICAL

#define                 CAN_CLK                              RCC_APB1Periph_CAN1
#define                 CAN_TX_GPIO_CLK                      (RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO)
#define                 CAN_TX_GPIO_PORT                     GPIOD
#define                 CAN_TX_GPIO_PIN                      GPIO_Pin_1
#define                 CAN_TX_GPIO_MODE                     GPIO_Mode_AF_PP

#define                 CAN_RX_GPIO_CLK                      (RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO)
#define                 CAN_RX_GPIO_PORT                     GPIOD
#define                 CAN_RX_GPIO_PIN                      GPIO_Pin_0
#define                 CAN_RX_GPIO_MODE                     GPIO_Mode_IPU

#define                 CAN_RX_IRQ							 USB_LP_CAN1_RX0_IRQn
#define                 CAN_RX_IRQHandler					 USB_LP_CAN1_RX0_IRQHandler
#define                 CAN_IRQ_PRIO                         14

#define    CAN_RX_BIT                  0x0001

//macro Define
#define     cNull               0x00
#define     cAll                0x00   //broadcast Data
#define		cCanRxBufSize		50   //Ethan 2015.2.5

#define		cCanRxBufEmpty		0
#define		cCanRxBufRdy		1

/************************************************************************************
************************************************************************************/

//ID config: SourceSend

 
//Comm Kind

//ID config¡mmessage type kind-CmdType 
#define		cDataTransfer   0x1
#define		cCommandSet		  0x2
#define		cMMControl		  0x3

/************************************************************************************
//	message mailbox Data define
************************************************************************************/
typedef union
{
	INT32U	DWORD;
  struct
	{      
		INT16U      Reserve:16;       // 0:15
		INT16U      Reserve1:4;       // 16:19
		INT16U      CmdKind:5;        // 20:24
		INT16U      SourceSend:4;     // 25:28
		INT16U      DIR;             //29
		INT16U      XTD:1;            //30
		INT16U      MSGVAL:1;         //31
	}BIT;
}CANID;

typedef union
{
	INT16U  WORD;
	struct
	{
		INT16U CmdIndex           :4;        //0:3 just for Slave(from 1 to 3), other Reserve 0
		INT16U SubCmmdIndex       :8;        //4:7
		INT16U SourceSend					:4;        //8:11
//		INT16U reserved           :4;        //12:15
	}BIT;
}CANCmdData;

typedef struct
{
	CANCmdData	CanData0;
	INT16U	    CanData1;
	INT16U	    CanData2;
	INT16U	    CanData3;
}CANFRAME;

void CAN_Mode_Config(void);
void CAN_Filter_Config(void);
void CAN_Config(void);
void CAN_SetMsg(void);
void sCanCopy(CANFRAME *pSource,CANFRAME *pDestination);
void sCanHdRead(CANFRAME *pdata,INT16U wMBXnbr);
INT8U	sCanRead(CANFRAME *pdata);
void	sCanRxISR(INT16U wMBXnbr);
void	sCanInitial(void);
void sSetCANDataSend1(INT8U bCmdIndex,INT8U bSubCmmdIndex,INT8U bSourceSend,
                       INT16U wData1,INT16U wData2,INT16U wData3);
void sSetCANDataSend2(INT8U bCmdIndex,INT8U bSubCmmdIndex,INT8U bSourceSend,
                       INT16U wData1,INT16U wData2,INT16U wData3);
void sSetCANDataSend3(INT8U bCmdIndex,INT8U bSubCmmdIndex,INT8U bSourceSend,
                       INT16U wData1,INT16U wData2,INT16U wData3);
#endif
