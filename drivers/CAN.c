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

CanTxMsg TxMessage;			     //���ͻ�����
CanRxMsg RxMessage;				 //���ջ�����

//INT8U CAN_msg_num[3] = {0};   // ����������

// Can��NVIC���ã�ʹ�÷���4
static void can_nvic_configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	NVIC_InitStructure.NVIC_IRQChannel = CAN_RX_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CAN_IRQ_PRIO;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// can��gpio����
static void can_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// ��gpioʱ��
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
 * ��������CAN_Mode_Config
 * ����  ��CAN��ģʽ ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
void CAN_Mode_Config(void)
{
   	CAN_InitTypeDef        CAN_InitStructure;
	
	// ��canʱ��
	RCC_APB1PeriphClockCmd( CAN_CLK, ENABLE );
	
	/************************CANͨ�Ų�������**********************************/
	/*CAN�Ĵ�����ʼ��*/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	
	  /*CAN��Ԫ��ʼ��*/
  	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ��
    CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  �Զ����߹��� 
	
    CAN_InitStructure.CAN_AWUM=ENABLE;			   //MCR-AWUM  ʹ���Զ�����ģʽ
	
    CAN_InitStructure.CAN_NART=ENABLE;			   //MCR-NART  ��ֹ�����Զ��ش�	  DISABLE-�Զ��ش�
    CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���  
    CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  ����FIFO���ȼ� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ�� 
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //��������ģʽ
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW ����ͬ����Ծ��� 2��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS1=CAN_BS1_6tq;		   //BTR-TS1 ʱ���1 ռ����6��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;		   //BTR-TS1 ʱ���2 ռ����3��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_Prescaler =8;		   ////BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 36/(1+6+2)/12=0.5Mbps
    CAN_Init(CAN1, &CAN_InitStructure);
}

/*
 * ��������CAN_Filter_Config
 * ����  ��CAN�Ĺ����� ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
void CAN_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN��������ʼ��*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;						//��������0(103ϵ�й�14��)
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//�����ڱ�ʶ������λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//������λ��Ϊ����32λ��
	/* ʹ�ܱ��ı�ʾ�����������ձ�ʾ�������ݽ��бȶԹ��ˣ���չID�������µľ����������ǵĻ��������FIFO0�� */

	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x0000;				//Ҫ���˵�ID��λ ������ÿһ��ID�Ź��ˣ�
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000;         //Ҫ���˵�ID��λ 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;		//��������16λÿλ����ƥ�䣨����ÿһ��ID�Ź��ˣ�
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;			//��������16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//��������������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//ʹ�ܹ�����
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/*CANͨ���ж�ʧ��*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
	CAN_ITConfig(CAN1, CAN_IT_FF0, DISABLE);
	CAN_ITConfig(CAN1, CAN_IT_FF1, DISABLE);
	
//	/*CANͨ���ж�ʹ��*/
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
	sCANMessage.ExtId = sSetMBOXMSGID(cDataTransfer,cAll); //�㲥��ʽ
	sCANMessage.RTR = CAN_RTR_DATA;    	        // ������Ϣ��֡����Ϊ����֡������Զ��֡��	
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
	sCANMessage.ExtId = sSetMBOXMSGID(cDataTransfer,cAll); //�㲥��ʽ
	sCANMessage.RTR = CAN_RTR_DATA;    	        // ������Ϣ��֡����Ϊ����֡������Զ��֡��	
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
	sCANMessage.ExtId = sSetMBOXMSGID(cDataTransfer,cAll); //�㲥��ʽ
	sCANMessage.RTR = CAN_RTR_DATA;    	        // ������Ϣ��֡����Ϊ����֡������Զ��֡��	
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

