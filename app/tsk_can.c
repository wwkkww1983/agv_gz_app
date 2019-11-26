#include "applications.h"
#include "can.h"
#include "service.h"

//ID config: SourceSend 
#define  MasterMach     0x01 
#define  SlaveMach      0x02

// CmdIndex MasterMach: 
#define  SendData     0x01 
#define  SendCmd      0x02
#define  SendAck      0x03
#define  SendNack     0x04

//bSubCmdIndex SetDisData:01 
#define  FrameNum     0x01


#define    CAN_PROTOCAL_HEADER           ( (uint16_t)(0x55AA) )

#define    CAN_PROTOCAL_FUN_QUERY        ( (uint16_t)(0xA55A) )
#define    CAN_PROTOCAL_FUN_CTL_OPEN     ( (uint16_t)(0x5AA5) )
#define    CAN_PROTOCAL_FUN_CTL_CLOSE    ( (uint16_t)(0x5A5A) )

#define    CAN_PROTOCAL_CMD_QUERY_BAT_INFO            ( (uint16_t)(0x0001) )
#define    CAN_PROTOCAL_CMD_QUERY_CHGR_STATION_INFO   ( (uint16_t)(0x0002) )
#define    CAN_PROTOCAL_CMD_QUERY_ONLINE              ( (uint16_t)(0x0003) )
#define    CAN_PROTOCAL_CMD_QUERY_PMBUS               ( (uint16_t)(0x0004) )


// can 一帧发送3个16位的值, 即一帧发送6个字节
#define    CAN_SEND_EACH_FRAME_WITH_BYTE_CNT    ( ( uint16_t )( 6 ))

#define    SIZE_CAN_TX_BUFF    250
#define    SIZE_CAN_RX_BUFF    250

// 发送缓存，把单个充电站信息发送到另一个充电站
static INT16U DisDataBuf[ SIZE_CAN_TX_BUFF ] = {0}; 
// 接收缓存，接收另一个充电站的信息
static INT16U  FrameBuf[ SIZE_CAN_RX_BUFF ] = {0};
// 保护can的发送
SemaphoreHandle_t xMutexCanSend = NULL;

uint8_t g_u16_can_query_timeout = 0;
static void can_send_cmd( uint16_t fun_code, uint16_t cmd );
static void can_process_cmd( uint16_t fun_code, uint16_t cmd );
static void can_send_ack( uint16_t fun_code, uint16_t ack );
static void can_send_nack( uint16_t fun_code, uint16_t ack );

void sCanDealWithBoxRx(void);
void sCANCOMMSendData(uint8_t SourceSend, uint8_t cmd_index, uint16_t frame_cnt );

eCanTestStatusDef g_e_can_test_status = CAN_TEST_CAN_COM;
TaskHandle_t h_can_comm_entry = NULL;
void   can_comm_entry( void * pvParameters )
{
BaseType_t xResult = pdTRUE;
//BaseType_t xReturn = pdPASS;
const TickType_t xTicksToWait = portMAX_DELAY; 
//const TickType_t xTicksToWait = pdMS_TO_TICKS(1000);

uint32_t ulNotifiedValue;
uint16_t cnt = 0;
	
// create can send operation mutex
// 保护can的发送，防止冲突
xMutexCanSend = xSemaphoreCreateMutex();
if( xMutexCanSend == NULL ) { kprintf("\r\ncreate can send mutex failed"); return; }
// 给出互斥量
xSemaphoreGive( xMutexCanSend );

    for(;;)
	{
	    /* Wait to be notified. */
        xResult = xTaskNotifyWait( (uint32_t)   pdFALSE, /* Don't clear bits on entry. */
                                   (uint32_t)   ULONG_MAX, /* Clear all bits on exit. */
                                   (uint32_t *) &ulNotifiedValue, /* Stores the notified value. */
                                   (TickType_t) xTicksToWait );
		// 接收
		if( xResult == pdPASS )
		{
			if( ulNotifiedValue & CAN_RX_BIT )
			{
				sCanDealWithBoxRx();
			}
			else if (ulNotifiedValue & NOTIFY_TSK_CAN_TEST )
			{
				can_send_cmd( CAN_PROTOCAL_FUN_QUERY, CAN_PROTOCAL_CMD_QUERY_ONLINE );
			    xTimerStart( xTimersBox_Test[TEST_TIMER_ID_CAN], 0 );
			}
			else if (ulNotifiedValue & NOTIFY_TSK_CAN_PMBUS_TEST )
			{
				can_send_cmd( CAN_PROTOCAL_FUN_QUERY, CAN_PROTOCAL_CMD_QUERY_PMBUS );
			    xTimerStart( xTimersBox_Test[TEST_TIMER_ID_CAN], 0 );
			}
		}
		else
		{
		}
	}
}

void sCANCOMMSendData(uint8_t SourceSend, uint8_t cmd_index, uint16_t frame_cnt ) 
{
	static INT16U  bMachiSendCnt = 0;
    BaseType_t xReturn = pdPASS;

	// 获取互斥量
	xReturn = xSemaphoreTake( xMutexCanSend, pdMS_TO_TICKS(500) );
    if( pdTRUE != xReturn )
		return;
	
	sSetCANDataSend1(cmd_index, FrameNum+bMachiSendCnt*3,SourceSend,FrameBuf[bMachiSendCnt*3],FrameBuf[bMachiSendCnt*3+1],FrameBuf[bMachiSendCnt*3+2]); 
	
	bMachiSendCnt++;
	if( bMachiSendCnt >= frame_cnt )
	{
		bMachiSendCnt = 0;
	}
	
    // 给出互斥量
	xSemaphoreGive( xMutexCanSend );
}

void sCanDealWithBoxRx(void)
{
    static uint16_t rx_cnt = 0;
    CANFRAME stCanRxData; 
    INT8U bSourceSend=0,bCmdIndex=0,bSubCmdIndex=0; 

	while(sCanRead(&stCanRxData) != cCanRxBufEmpty)
	{
		bSourceSend = stCanRxData.CanData0.BIT.SourceSend; 
		bCmdIndex = stCanRxData.CanData0.BIT.CmdIndex; 
		bSubCmdIndex = stCanRxData.CanData0.BIT.SubCmmdIndex; 

		if(bSourceSend == SlaveMach)
		{
			DisDataBuf[bSubCmdIndex-1] = stCanRxData.CanData1; 
			DisDataBuf[bSubCmdIndex] = stCanRxData.CanData2; 
			DisDataBuf[bSubCmdIndex+1] = stCanRxData.CanData3;
			
			rx_cnt ++;

			switch( bCmdIndex )
			{
				case SendAck:
			        if( rx_cnt >= 1 )
				    {
						DisDataBuf[0] = stCanRxData.CanData1; 
						DisDataBuf[1] = stCanRxData.CanData2; 
						DisDataBuf[2] = stCanRxData.CanData3;
						
						if( DisDataBuf[2] == CAN_PROTOCAL_CMD_QUERY_ONLINE )
						{
							LOG_DEBUG_APP_1("\r\n..........can comm, master rec slave online ack");
							rx_cnt = 0;
							
							// stop timeout timer
							xTimerStop( xTimersBox_Test[TEST_TIMER_ID_CAN], 0 );
							
							// can test success
							xTaskNotify( h_cmd_entry, NOTIFY_TSK_CMD_CAN_TEST_SUCCESS, eSetBits );
						}
						else if( DisDataBuf[2] == CAN_PROTOCAL_CMD_QUERY_PMBUS )
						{
							LOG_DEBUG_APP_1("\r\n..........can comm, master rec pmbus success ack");
							rx_cnt = 0;
							
							// stop timeout timer
							xTimerStop( xTimersBox_Test[TEST_TIMER_ID_CAN], 0 );
							
							// pmbus test success
							xTaskNotify( h_cmd_entry, NOTIFY_TSK_CMD_PMBUS_TEST_SUCCESS, eSetBits );
						}
				    }
					break;
				case SendNack:
					if( rx_cnt >= 1 )
				    {
					    DisDataBuf[0] = stCanRxData.CanData1; 
						DisDataBuf[1] = stCanRxData.CanData2; 
						DisDataBuf[2] = stCanRxData.CanData3;
						if( DisDataBuf[2] == CAN_PROTOCAL_CMD_QUERY_PMBUS )
						{
							LOG_DEBUG_APP_1("\r\n..........can comm, master rec pmbus failed ack");
							rx_cnt = 0;
							
							// stop timeout timer
							xTimerStop( xTimersBox_Test[TEST_TIMER_ID_CAN], 0 );
							
							// pmbus test success
							xTaskNotify( h_cmd_entry, NOTIFY_TSK_CMD_PMBUS_TEST_FAILED, eSetBits );
						}
					}
					break;
				default:
					break;
			}
		}
		else if(bSourceSend == MasterMach)
		{
			switch( bCmdIndex )
			{
                case SendCmd:
			        DisDataBuf[0] = stCanRxData.CanData1; 
			        DisDataBuf[1] = stCanRxData.CanData2; 
			        DisDataBuf[2] = stCanRxData.CanData3;

			        if( DisDataBuf[0] != 0x55AA )
			        {
			        	LOG_DEBUG_APP("\r\nfrom master, header err!");
			        	return;
			        }
                    can_process_cmd( DisDataBuf[1], DisDataBuf[2] );
					break;
				default:
					break;
			}
		}
	}
}

static void can_send_cmd( uint16_t fun_code, uint16_t cmd )
{
    FrameBuf[0] = CAN_PROTOCAL_HEADER;
    FrameBuf[1] = fun_code;
    FrameBuf[2] = cmd;

    sCANCOMMSendData( MasterMach, SendCmd, 1 );
}

static void can_send_ack( uint16_t fun_code, uint16_t cmd )
{
    FrameBuf[0] = CAN_PROTOCAL_HEADER;
    FrameBuf[1] = fun_code;
    FrameBuf[2] = cmd;

    sCANCOMMSendData( SlaveMach, SendAck, 1 );
}

static void can_send_nack( uint16_t fun_code, uint16_t ack )
{
    FrameBuf[0] = CAN_PROTOCAL_HEADER;
    FrameBuf[1] = fun_code;
    FrameBuf[2] = ack;

    sCANCOMMSendData( SlaveMach, SendNack, 1 );
}

#include "bsp_rpb1600_48.h"
extern const uint8_t addr_list[8];
static void can_process_cmd( uint16_t fun_code, uint16_t cmd )
{
	uint16_t cnt = 0;
	uint32_t idx = 0;
	
	switch( fun_code )
	{
		case CAN_PROTOCAL_FUN_QUERY:
			if( cmd == CAN_PROTOCAL_CMD_QUERY_ONLINE )
			{
				can_send_ack( fun_code, cmd );
			}
			else if( cmd == CAN_PROTOCAL_CMD_QUERY_PMBUS )
			{
			    for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
				{
					if( ChkAddrDev( addr_list[ idx ] ) )
					{
						can_send_ack( fun_code, cmd );
						printf("gz_test com ack pmbus");
						return;
					}
					vTaskDelay( pdMS_TO_TICKS( 20 ) );
				}

				can_send_nack( fun_code, cmd );
				printf("gz_test com nack pmbus");
			}
			break;
		default:
			break;
	}
}


