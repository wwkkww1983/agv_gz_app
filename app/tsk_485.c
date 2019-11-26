#include "applications.h"
#include "SPI_WK2168.h"
#include "service.h"
#include "string.h"
#include "exti.h"

typedef struct
{
	INT8U SendDataFlag;
	INT8U RFlag;    // 可以读取缓冲区数据的标志，TRUE表示可以去读，否则不可以读取
	INT8U SendPort;
	INT8U RWDataBuf[255];  // 文档注明，最大数据长度为1024，实际数据长度根据现场调试确定
	INT16U TxDataLen;
	INT16U RxDataLen;   // 接收数据的长度
}WK2168_Data;


WK2168_Data SPI1_RTxData[4];
WK2168_Data SPI2_RTxData[4];
void sGetSPI1_RS485Data(SPI_TypeDef* SPIx);
void sGetSPI2_RS485Data(SPI_TypeDef* SPIx);
void sSendRS485Data(SPI_TypeDef* SPIx,INT8U PortNum );

static BOOL rs485_data_parse( uint8_t port_num );
static void deal_with_bms_rx(void);
void start_485_test(void);

// 保存测试失败时的测试结果
uint32_t g_u32_test_result_bms = 0;

TaskHandle_t h_ext_bms_comm_entry = NULL;
void bms_comm_entry(void)
{
BaseType_t xResult = pdTRUE;
uint32_t ulNotifiedValue;
const TickType_t xTicksToWait = 1000;
//const TickType_t xTicksToWait = portMAX_DELAY;
uint8_t gena_1 = 0;
uint8_t gena_2 = 0;
	
	for( ;; )
	{
		/* Wait to be notified. */
        xResult = xTaskNotifyWait( (uint32_t)   pdFALSE, /* Don't clear bits on entry. */
                                   (uint32_t)   ULONG_MAX, /* Clear all bits on exit. */
                                   (uint32_t *) &ulNotifiedValue, /* Stores the notified value. */
                                   (TickType_t) xTicksToWait ); /* timeout:1s */
        if( xResult == pdPASS )
        {
            /* A notification was received. See which bits were set. */
			if( ulNotifiedValue & RX_IDLE_BIT )
			{
				deal_with_bms_rx();
			}
			else if (ulNotifiedValue & NOTIFY_TSK_BMS_TEST )
			{
			    start_485_test();
			}
		}
		else
		{
			gena_1 = WkReadGReg( SPI1, WK2XXX_GENA );
			gena_2 = WkReadGReg( SPI2, WK2XXX_GENA );
			if( ( gena_1 != 0x3F ) || ( gena_2 != 0x3F ) )
			{
				LOG_DEBUG_APP("\r\n-----------------gena occur err");
				WK2168_Sys_Init();
				delay_ms(50);
			}
		}
	}
}

void start_485_test(void)
{
	uint32_t idx = 0, port_num = 0;
	char bSendDataBuf[] = "gz_test com 485";
	uint32_t len = strlen( bSendDataBuf );

	// result 清零
	g_u32_test_result_bms = 0;

	// 操作前，需要临时关闭对应的外部中断
	Exti_Disable();
	
    // 因为每个通道有独立的fifo，所以可以同时向八个通道发送查询指令
    for( idx = 0; idx < 4; idx ++ )
    {
    	port_num = idx + 1;

        // send spi1
        if( SPI1_RTxData[idx].SendDataFlag != 0x01 )
        {
        	SPI1_RTxData[idx].SendPort = port_num;
        	SPI1_RTxData[idx].TxDataLen = len;
        	memmove(SPI1_RTxData[idx].RWDataBuf, bSendDataBuf, len);

        	sSendRS485Data( SPI1, port_num);
			
			SPI1_RTxData[idx].RFlag = FALSE;
			SPI1_RTxData[idx].RxDataLen = 0;
			SPI1_RTxData[idx].SendDataFlag = 0;
        }
//		printf("\r\n向通道%d发送data->:%s", port_num, bSendDataBuf);
    }
	
	for( idx = 0; idx < 4; idx ++ )
    {
    	port_num = idx + 1;

        // send spi2
        if( SPI2_RTxData[idx].SendDataFlag != 0x01 )
        {
        	SPI2_RTxData[idx].SendPort = port_num;
        	SPI2_RTxData[idx].TxDataLen = len;
        	memmove(SPI2_RTxData[idx].RWDataBuf, bSendDataBuf, len);

        	sSendRS485Data( SPI2, port_num);
			
			SPI2_RTxData[idx].RFlag = FALSE;
			SPI2_RTxData[idx].RxDataLen = 0;
			SPI2_RTxData[idx].SendDataFlag = 0;
        }
//		printf("\r\n向通道%d发送data->:%s", port_num+4, bSendDataBuf);
    }

	// 操作完成后，打开对应的外部中断
	Exti_Enable();
	
	xTimerStart( xTimersBox_Test[TEST_TIMER_ID_485], 0 );
}

static void deal_with_bms_rx(void)
{
	uint32_t idx;

	for( idx = 0; idx < 4; idx++ )
	{
		if( SPI1_RTxData[idx].RFlag == TRUE )
		{
			printf("\r\nport:%d rec data:%s", idx+1, SPI1_RTxData[idx].RWDataBuf);
		    rs485_data_parse( idx + 1 );
	        // clear
			memset(&SPI1_RTxData[idx].SendDataFlag,0x00,sizeof(WK2168_Data));
			SPI1_RTxData[idx].RxDataLen = 0;
	        SPI1_RTxData[idx].RFlag = FALSE;
		}

		if( SPI2_RTxData[idx].RFlag == TRUE )
		{
			printf("\r\nport:%d rec data:%s", idx+5, SPI2_RTxData[idx].RWDataBuf);
		    rs485_data_parse( idx + 5 );
	        // clear
	        memset(&SPI2_RTxData[idx].SendDataFlag,0x00,sizeof(WK2168_Data));
			SPI2_RTxData[idx].RxDataLen = 0;
	        SPI2_RTxData[idx].RFlag = FALSE;
		}
	}
}

static BOOL rs485_data_parse( uint8_t port_num )
{
	char rec_buff[] = "gz_test com 485";
	char ack_buff[] = "gz_test com ack 485";
	uint8_t *p_data = NULL;
	uint8_t idx = 0, tmp_port = 0;
	uint32_t len;

	if( ( port_num >= 1 ) && ( port_num <= 4 ) )
        p_data = SPI1_RTxData[port_num-1].RWDataBuf;
	else if( ( port_num >4 ) && ( port_num <= 8 ) )
        p_data = SPI2_RTxData[port_num-5].RWDataBuf;
	else
	    return FALSE;

	if( 0 == strcmp( (const char*)p_data, (const char*)rec_buff ) )
	{
		len = sizeof( ack_buff );
		tmp_port = port_num;
	    if( ( port_num >= 1 ) && ( port_num <= 4 ) )
		{
			idx = port_num - 1;
		    // send ack
			kprintf("\r\nport:%d rec gz_board test data, send ack", port_num);
      	    SPI1_RTxData[idx].SendPort = tmp_port;
      	    SPI1_RTxData[idx].TxDataLen = len;
      	    memmove(SPI1_RTxData[idx].RWDataBuf, ack_buff, len);

      	    sSendRS485Data( SPI1, tmp_port);

	    	SPI1_RTxData[idx].RFlag = FALSE;
	    	SPI1_RTxData[idx].RxDataLen = 0;
	    	SPI1_RTxData[idx].SendDataFlag = 0;
		}
	    else if( ( port_num >4 ) && ( port_num <= 8 ) )
		{
			idx = port_num - 5;
		    tmp_port = port_num - 4;
		    // send ack
			kprintf("\r\nport:%d rec gz_board test data, send ack", port_num);
      	    SPI2_RTxData[idx].SendPort = tmp_port;
      	    SPI2_RTxData[idx].TxDataLen = len;
      	    memmove(SPI2_RTxData[idx].RWDataBuf, ack_buff, len);

      	    sSendRS485Data( SPI2, tmp_port);

	    	SPI2_RTxData[idx].RFlag = FALSE;
	    	SPI2_RTxData[idx].RxDataLen = 0;
	    	SPI2_RTxData[idx].SendDataFlag = 0;
		}
	}
	else if( 0 == strcmp( (const char*)p_data, (const char*)ack_buff ) )
	{
		idx = port_num - 1;
		set_bit( g_u32_test_result_bms, idx );
		kprintf("\r\nport:%d rec ack", port_num);
		if( (g_u32_test_result_bms & 0xFF) == 0xFF )
		{
			// stop timeout timer
			xTimerStop( xTimersBox_Test[TEST_TIMER_ID_485], 0 );

			kprintf("\r\n485 test success..........");
			// 485 test success
		    xTaskNotify( h_cmd_entry, NOTIFY_TSK_CMD_485_TEST_SUCCESS, eSetBits );
			g_u32_test_result_bms = 0;
		}
	}
	
	return TRUE;
}


void sSendRS485Data(SPI_TypeDef* SPIx,INT8U PortNum)
{
	WK2168_Data *pRTxData = NULL;
	
	if(SPIx == SPI1)
	{
		pRTxData = SPI1_RTxData;
	}
	else if( SPIx == SPI2 )
	{
		pRTxData = SPI2_RTxData;
	}
	else
	{
		LOG_DEBUG_APP_1("\r\nin send485 data, invalid spi interface");
		return;
	}
	
	if( ( PortNum < 1 ) || ( PortNum > 4 ) )
	{
		LOG_DEBUG_APP_1("\r\nin send485 data, invalid port");
		return;
	}
	
	pRTxData[PortNum-1].SendDataFlag = 1;
	wk_TxChars( SPIx, pRTxData[PortNum-1].SendPort, pRTxData[PortNum-1].TxDataLen, pRTxData[PortNum-1].RWDataBuf );
	memset(pRTxData[PortNum-1].RWDataBuf,0x00,sizeof(pRTxData[PortNum-1].RWDataBuf));	
}


uint16_t cache_bms_rx( SPI_TypeDef* SPIx, u8 port )
{
	WK2168_Data *pRTxData;
	uint8_t *recbuf;
	uint8_t fsr=0,rfcnt=0,sifr=0;
    uint16_t len=0;
	uint16_t offset_buff = port - 1;

	// indicate buffer
	if( SPIx == SPI1 )
		pRTxData = SPI1_RTxData;
	else if( SPIx == SPI2 )
		pRTxData = SPI2_RTxData;

	// read rec data
	sifr=WkReadSReg(SPIx, port,WK2XXX_SIFR);	
	if((sifr&WK2XXX_RFTRIG_INT)||(sifr&WK2XXX_RXOVT_INT))
	{ 
	    fsr = WkReadSReg(SPIx, port,WK2XXX_FSR);
	    rfcnt = WkReadSReg(SPIx, port,WK2XXX_RFCNT);
	    //printf("rfcnt=0x%x.\n",rfcnt);
		if(fsr& WK2XXX_RDAT)
	    { 
	  	    len=(rfcnt==0)?256:rfcnt;
	    }

		// 确定接收数据存储的地址
	    recbuf = pRTxData[offset_buff].RWDataBuf + pRTxData[offset_buff].RxDataLen;
#if 1
        WkReadSFifo(SPIx, port,recbuf,len);
#else
	    for(n=0;n<len;n++)
	        *(recbuf+n)=WkReadSReg(SPIx, port,WK2XXX_FDAT);
#endif	

		pRTxData[offset_buff].RxDataLen += len;
		// 如果是超时中断，说明数据接收完了
		if( sifr & WK2XXX_RXOVT_INT )
		{
//			printf("\r\nrxout_int");
			pRTxData[offset_buff].RFlag = TRUE; // 表示可以读取并解析缓冲区的数据了
		    xTaskNotifyFromISR( h_ext_bms_comm_entry, RX_IDLE_BIT, eSetBits, NULL );
		}

	    return len;
    }
	else
	{
		len=0;
		return len;
	}

}
