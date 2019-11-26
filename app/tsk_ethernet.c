#include "applications.h"
#include "service.h"
#include <string.h>

#define    WIFI_FUN_CODE_GET    0x04
#define    WIFI_FUN_CODE_SET    0x03
#define    WIFI_REC_MAX_LEN     100    // 20191024,添加产品序号和产品编码功能，所以接收数据的长度需要加上，暂定限制为100


typedef struct
{
	uint8_t header[4];        // 帧头
	uint16_t frame_len;       // 帧数据长度
	uint8_t _empty_1;         // 预留
	uint8_t cmd;              // 命令字
	uint8_t fun_code;         // 功能码
	uint16_t chgr_unit_num;   // 充电站序号
	uint16_t serial_chl_num;  // 输出通道号
	uint16_t crc;             // CRC值
	uint8_t data_len;         // 数据长度
	uint8_t data[WIFI_REC_MAX_LEN-15];        // 数据
}WifiDataDef;


TaskHandle_t h_ext_wifi_entry = NULL;


/********************************************************************************************
// tcp wifi communication 
*********************************************************************************************/
WifiDataDef wifi_rx_data;  

BOOL wifi_data_parse(void);
void wifi_data_process(void);
static void wifi_for_get( void );
static void wifi_for_set( void );
void for_test( void );

static void start_ethernet_test(void);

// for wifi comm
void ext_wifi_entry( void * pvParameters )
{
BaseType_t xResult;
uint32_t ulNotifiedValue;
const TickType_t xTicksToWait = portMAX_DELAY;

    for( ;; )
	{
		/* Wait to be notified of an interrupt. */
        xResult = xTaskNotifyWait( (uint32_t)   pdFALSE, /* Don't clear bits on entry. */
                                   (uint32_t)   ULONG_MAX, /* Clear all bits on exit. */
                                   (uint32_t *) &ulNotifiedValue, /* Stores the notified value. */
                                   (TickType_t) xTicksToWait );
        if( xResult == pdPASS )
        {
            /* A notification was received. See which bits were set. */
            if( ( ulNotifiedValue & TX_IDLE_BIT ) != 0 )
            {
			    /* The TX ISR has set a bit. */
//                prvProcessTx();
            }
            else if( ( ulNotifiedValue & RX_IDLE_BIT ) != 0 )
            {
                /* The RX ISR has set a bit. */
//                prvProcessRx();

                wifi_data_process();
//				for_test();
            }
			else if (ulNotifiedValue & NOTIFY_TSK_WIFI_TEST )
			{
				start_ethernet_test();
			}
	    }
	}
}


static void start_ethernet_test(void)
{
	const char send_buff[] = "gz_test com ethernet";
	uint32_t i = 0;

	for(i = 0; i < strlen(send_buff); i++ )
		com_ext_wifi->write( send_buff[i] );

    com_ext_wifi->send();

	// start test timeout timer
    xTimerStart( xTimersBox_Test[TEST_TIMER_ID_ETHERNET], 0 );
}

void wifi_data_process(void)
{
	if( !wifi_data_parse() )
	{
		LOG_DEBUG_APP("\r\nparse wifi rec data failed, clear rx cache");
		// 清空接收缓存
		com_ext_wifi->reset_rx();

		// 清空数据缓存
		memset(&wifi_rx_data, 0, sizeof( WifiDataDef) );
	    return;
	}

	// 清空数据缓存
	memset(&wifi_rx_data, 0, sizeof( WifiDataDef) );
}

BOOL wifi_data_parse(void)
{
	uint16_t idx = 0;
	uint8_t rec_buff[WIFI_REC_MAX_LEN] = {0};
	uint8_t std_buff[] = "gz_test com ethernet";
	uint8_t ack_buff[] = "gz_test com ack ethernet";
	uint16_t len = 0;
	uint16_t crc_calc = 0;
	uint32_t crc_idx = 0;

// 先读取所有数据
    len = com_ext_wifi->get_rx_cnt();
	LOG_DEBUG_APP("\r\n\r\nwifi rec data len:%d", len);
	// 判断接收的数据是否超出限制值
	if( len >= WIFI_REC_MAX_LEN )
	{
		LOG_DEBUG_APP_1("data length out of limit, clear rx buff, wifi rec max size:%d, actual rec size:%d", (uint32_t)WIFI_REC_MAX_LEN, len);
	    return FALSE;
	}
	else
	{
	    LOG_DEBUG_APP_1("data length correct");
	}
	
	// 读出所有数据
	LOG_DEBUG_APP_1("\r\nrec data: ");
//	printf("\r\nrec data: ");
	for( idx = 0; idx < len; idx ++ )
	{
		rec_buff[idx] = com_ext_wifi->read();
		LOG_DEBUG_APP_1("%02x", rec_buff[idx]);
		printf("%02x", rec_buff[idx]);
	}
	
// 处理相关业务
	if( 0 == strcmp(rec_buff, std_buff) )
	{
		for( idx = 0; idx < strlen(std_buff); idx ++ )
		    com_ext_wifi->write( std_buff[idx] );

		com_ext_wifi->send();
//		printf("\r\nack data");
	}
	else if( 0 == strcmp(rec_buff, ack_buff) )
	{
//		printf("\r\nethernet test success");
		// stop timeout timer
        xTimerStop( xTimersBox_Test[TEST_TIMER_ID_ETHERNET], 0 );

		// ethernet test success
	    xTaskNotify( h_cmd_entry, NOTIFY_TSK_CMD_ETHERNET_TEST_SUCCESS, eSetBits );
	}
    return TRUE;
}




