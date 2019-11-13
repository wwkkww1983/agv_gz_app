/**
  ******************************************************************************
  * @file    CAN.c
  * @author  June_Zu 
  * @version V1.0.0
  * @date    04-March-2017
  * @brief   CAN file
  ******************************************************************************/

#include "CAN.h" 
#include "string.h"

/********************************************************************************
* variables dfinition	*
*********************************************************************************/
//CAN receive buffer
CANFRAME	wCanRxBuf[cCanRxBufSize];
CANFRAME	*pCanRxIn;				
CANFRAME	*pCanRxOut;
INT8U		wCanRxLength;

CanTxMsg TxMessage;			     //发送缓冲区
CanRxMsg RxMessage;				 //接收缓冲区

//INT8U CAN_msg_num[3] = {0};   // 发送邮箱标记

// Can的NVIC配置，使用分组4
static void can_nvic_configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_InitStructure.NVIC_IRQChannel = CAN_RX_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CAN_IRQ_PRIO;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// can的gpio配置
static void can_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// 打开gpio时钟
	RCC_APB2PeriphClockCmd ( CAN_TX_GPIO_CLK | CAN_RX_GPIO_CLK, ENABLE );
	GPIO_PinRemapConfig( GPIO_Remap2_CAN1, ENABLE );  // Remap2 - PD0, PD1

	GPIO_InitStructure.GPIO_Pin = CAN_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = CAN_TX_GPIO_MODE; 
	GPIO_Init(CAN_TX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = CAN_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = CAN_RX_GPIO_MODE; 
	GPIO_Init(CAN_RX_GPIO_PORT, &GPIO_InitStructure);
}

/*
 * 函数名：CAN_Mode_Config
 * 描述  ：CAN的模式 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
void CAN_Mode_Config(void)
{
   	CAN_InitTypeDef        CAN_InitStructure;
	
	// 打开can时钟
	RCC_APB1PeriphClockCmd( CAN_CLK, ENABLE );
	
	/************************CAN通信参数设置**********************************/
	/*CAN寄存器初始化*/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	
	  /*CAN单元初始化*/
  	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
    CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  自动离线管理 
	
    CAN_InitStructure.CAN_AWUM=ENABLE;			   //MCR-AWUM  使用自动唤醒模式
	
    CAN_InitStructure.CAN_NART=ENABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
    CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
    CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常工作模式
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW 重新同步跳跃宽度 2个时间单元
    CAN_InitStructure.CAN_BS1=CAN_BS1_6tq;		   //BTR-TS1 时间段1 占用了6个时间单元
    CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;		   //BTR-TS1 时间段2 占用了3个时间单元
    CAN_InitStructure.CAN_Prescaler =8;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 36/(1+6+2)/12=0.5Mbps
    CAN_Init(CAN1, &CAN_InitStructure);
}

/*
 * 函数名：CAN_Filter_Config
 * 描述  ：CAN的过滤器 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
void CAN_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN过滤器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;						//过滤器组0(103系列共14个)
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在标识符屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//过滤器位宽为单个32位。
	/* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x0000;				//要过滤的ID高位 （具体每一个ID号过滤）
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000;         //要过滤的ID低位 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;		//过滤器高16位每位必须匹配（具体每一个ID号过滤）
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;			//过滤器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//过滤器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能过滤器
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/*CAN通信中断失能*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
	CAN_ITConfig(CAN1, CAN_IT_FF0, DISABLE);
	CAN_ITConfig(CAN1, CAN_IT_FF1, DISABLE);
	
//	/*CAN通信中断使能*/
//	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

INT32U sSetMBOXMSGID(INT8U bCmdKind,INT8U SourceSend)
{
	CANID	    dwCanIdTemp;
	
	dwCanIdTemp.BIT.Reserve = 0;
	dwCanIdTemp.BIT.Reserve1 = 0;
	
	dwCanIdTemp.BIT.SourceSend = SourceSend;
	dwCanIdTemp.BIT.CmdKind = bCmdKind;
	
	dwCanIdTemp.BIT.DIR = 0; //receive
	dwCanIdTemp.BIT.XTD = 1; //29 bits
 	dwCanIdTemp.BIT.MSGVAL = 0;
	
	return(dwCanIdTemp.DWORD);
}
/************************************************************************************
* Name: sSetCANDataSend1
* Function:Send ext data;
* The Function: Send data by the can's communite(Std Frame)
* Author:@Ect_June_Zu
************************************************************************************/
void sSetCANDataSend1(INT8U bCmdIndex,INT8U bSubCmmdIndex,INT8U bSourceSend,
                       INT16U wData1,INT16U wData2,INT16U wData3)
{
	
	CANCmdData bCANCmdData;
	CanTxMsg  sCANMessage;
	INT8U    CANMsgData[8];	
		
	bCANCmdData.BIT.CmdIndex = bCmdIndex;
	bCANCmdData.BIT.SubCmmdIndex = bSubCmmdIndex;	
	bCANCmdData.BIT.SourceSend = bSourceSend;

	CANMsgData[0] = (INT8U)bCANCmdData.WORD;
	CANMsgData[1] = (INT8U)(bCANCmdData.WORD>>8);
	CANMsgData[2] = (INT8U)wData1;
	CANMsgData[3] = (INT8U)(wData1>>8);
	CANMsgData[4] = (INT8U)wData2;
	CANMsgData[5] = (INT8U)(wData2>>8);
	CANMsgData[6] = (INT8U)wData3;
	CANMsgData[7] = (INT8U)(wData3>>8);
	sCANMessage.ExtId = sSetMBOXMSGID(cDataTransfer,cAll); //广播形式
	sCANMessage.RTR = CAN_RTR_DATA;    	        // 传输消息的帧类型为数据帧（还有远程帧）	
	sCANMessage.IDE = CAN_ID_EXT;
	sCANMessage.DLC = 8;                        // size of message is 8
	memcpy(sCANMessage.Data,CANMsgData,8);
	
  CAN_Transmit(CAN1, &sCANMessage);
//	CAN_msg_num[0] = 1; 
//	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);
}

/************************************************************************************
* Name: sSetCANDataSend2
* Function:Send ext data;
* The Function: Send data by the can's communite(Std Frame)
* Author:@Ect_June_Zu
************************************************************************************/
void sSetCANDataSend2(INT8U bCmdIndex,INT8U bSubCmmdIndex,INT8U bSourceSend,
                       INT16U wData1,INT16U wData2,INT16U wData3)
{
	
	CANCmdData bCANCmdData;
	CanTxMsg  sCANMessage;
	INT8U    CANMsgData[8];	
		
	bCANCmdData.BIT.CmdIndex = bCmdIndex;
	bCANCmdData.BIT.SubCmmdIndex = bSubCmmdIndex;	
	bCANCmdData.BIT.SourceSend = bSourceSend;

	CANMsgData[0] = (INT8U)bCANCmdData.WORD;
	CANMsgData[1] = (INT8U)(bCANCmdData.WORD>>8);
	CANMsgData[2] = (INT8U)wData1;
	CANMsgData[3] = (INT8U)(wData1>>8);
	CANMsgData[4] = (INT8U)wData2;
	CANMsgData[5] = (INT8U)(wData2>>8);
	CANMsgData[6] = (INT8U)wData3;
	CANMsgData[7] = (INT8U)(wData3>>8);
	sCANMessage.ExtId = sSetMBOXMSGID(cDataTransfer,cAll); //广播形式
	sCANMessage.RTR = CAN_RTR_DATA;    	        // 传输消息的帧类型为数据帧（还有远程帧）	
	sCANMessage.IDE = CAN_ID_EXT;
	sCANMessage.DLC = 8;                        // size of message is 8
	memcpy(sCANMessage.Data,CANMsgData,8);
	
  CAN_Transmit(CAN1, &sCANMessage);
//  CAN_msg_num[1] = 1;
//	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE); 
}
/************************************************************************************
* Name: sSetCANDataSend3
* Function:Send ext data;
* The Function: Send data by the can's communite(Std Frame)
* Author:@Ect_June_Zu
************************************************************************************/
void sSetCANDataSend3(INT8U bCmdIndex,INT8U bSubCmmdIndex,INT8U bSourceSend,
                       INT16U wData1,INT16U wData2,INT16U wData3)
{
	
	CANCmdData bCANCmdData;
	CanTxMsg  sCANMessage;
	INT8U    CANMsgData[8];	
		
	bCANCmdData.BIT.CmdIndex = bCmdIndex;
	bCANCmdData.BIT.SubCmmdIndex = bSubCmmdIndex;	
	bCANCmdData.BIT.SourceSend = bSourceSend;

	CANMsgData[0] = (INT8U)bCANCmdData.WORD;
	CANMsgData[1] = (INT8U)(bCANCmdData.WORD>>8);
	CANMsgData[2] = (INT8U)wData1;
	CANMsgData[3] = (INT8U)(wData1>>8);
	CANMsgData[4] = (INT8U)wData2;
	CANMsgData[5] = (INT8U)(wData2>>8);
	CANMsgData[6] = (INT8U)wData3;
	CANMsgData[7] = (INT8U)(wData3>>8);
	sCANMessage.ExtId = sSetMBOXMSGID(cDataTransfer,cAll); //广播形式
	sCANMessage.RTR = CAN_RTR_DATA;    	        // 传输消息的帧类型为数据帧（还有远程帧）	
	sCANMessage.IDE = CAN_ID_EXT;
	sCANMessage.DLC = 8;                        // size of message is 8
	memcpy(sCANMessage.Data,CANMsgData,8);
	
  CAN_Transmit(CAN1, &sCANMessage);
//  CAN_msg_num[2] = 1;
//	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE); 
}
/********************************************************************************
*Function name:		sCanRead													*
*Parameters:		pdata:the address where to save the data					*
*Returns:			cCanRxBufEmpty:there is no data in the receive buffer		*
*					cCanRxBufRdy:data is read successdully from buffer			*
*Description:		This fucntion get the data from receiving buffer and save to*
*					user's buffer.it should be executed in task					*
*********************************************************************************/
INT8U	  sCanRead(CANFRAME *pdata)
{
	#if OS_CRITICAL_METHOD == 3          // Allocate storage for CPU status register 
	OS_CPU_SR  cpu_sr;
	#endif
	
	if(wCanRxLength==0)
	{
		return(cCanRxBufEmpty);
	}
	
	OS_ENTER_CRITICAL();
	sCanCopy(pCanRxOut,pdata);
	wCanRxLength--;
	
	if(pCanRxOut==&wCanRxBuf[cCanRxBufSize-1])
	{
		pCanRxOut=wCanRxBuf;
	}
	else
	{
		pCanRxOut++;
	}
	OS_EXIT_CRITICAL();
	return(cCanRxBufRdy);
}
/********************************************************************************
*Function name:		sCanCopy													*
*Parameters:		pSource:pointer to the source data							*
*					pDestination:pointer to the destination data				*
*Description:		This fucntion copy the content of psource to pDestination	*
*********************************************************************************/
void	sCanCopy(CANFRAME *pSource,CANFRAME *pDestination)
{
	pDestination->CanData0.WORD=pSource->CanData0.WORD;
	pDestination->CanData1=pSource->CanData1;
	pDestination->CanData2=pSource->CanData2;
	pDestination->CanData3=pSource->CanData3;
}

/********************************************************************************
*Function name:		sCanRxISR													*
*Parameters:		none														*
*Description:		This fucntion should be executed in the can receiving 		*
*					interrupt,it save the received data to the queue and send	*
*					event to AP													*
*********************************************************************************/

extern TaskHandle_t h_can_comm_entry; 
void	sCanRxISR(INT16U wMBXnbr)
{	
	CAN_ClearITPendingBit(CAN1, wMBXnbr);
	
	if(wCanRxLength == cCanRxBufSize)
	{
		return;
	}
	else
	{
		sCanHdRead(pCanRxIn,wMBXnbr);
		if(pCanRxIn == &wCanRxBuf[cCanRxBufSize - 1])
		{
			pCanRxIn = wCanRxBuf;
		}
		else
		{
			pCanRxIn++;
		}
		wCanRxLength++;
	}

//	xTaskNotifyFromISR( h_can_comm_entry, CAN_RX_BIT, eSetBits, &xHigherPriorityTaskWoken );
//	OSISREventSend(cPrioDataCANTask,eMailboxReceive);
}


void sCanHdRead(CANFRAME *pdata,INT16U wMBXnbr)
{
	  CanRxMsg  sCANMessage;
	  INT8U ucMsgData[8];
	
	  CAN_Receive(CAN1, wMBXnbr, &sCANMessage);
	  memcpy(ucMsgData,sCANMessage.Data,8);
	 
	  pdata->CanData0.WORD = ucMsgData[1]<<8|ucMsgData[0];	
		pdata->CanData1 = ucMsgData[3]<<8|ucMsgData[2];
		pdata->CanData2 = ucMsgData[5]<<8|ucMsgData[4];
		pdata->CanData3 = ucMsgData[7]<<8|ucMsgData[6];		
}

void	sCanInitial(void)
{
	can_gpio_init();
	can_nvic_configuration();
	CAN_Mode_Config();
	CAN_Filter_Config();	
	
	pCanRxIn = wCanRxBuf;			//CAN data
	pCanRxOut = wCanRxBuf;
	wCanRxLength = 0;
}
/***********************END OF FILE************************************/

