#include "stm32f10x.h"
#include "stdarg.h"
#include "string.h"
#include "stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "limits.h"

#include "service.h"
#include "applications.h"
#include "uart.h"
#include "bsp_rpb1600_48.h"
#include "adc.h"
#include "gpio.h"


static char *ver_proj  = "agv_hk_20181220.29";									  
static char *ver_dev = "RPB_1600_48";
static char *ver_sw  = VER_SW;
static char *ver_hmi  = VER_HMI;

#define    PRINTF_STACK    256
#define    PRINTF_PRIO     1
TaskHandle_t h_printf_entry = NULL;
void printf_entry( void *pvParameter );


#define    CMD_STACK    384
#define    CMD_PRIO     2
void cmd_entry( void *pvParameter );
TaskHandle_t h_cmd_entry = NULL;

//extern const uint8_t addr_list[8];
const uint8_t addr_list[8] = {0x80, 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0x8E};


// ���ӡ���������ݵ���Ϣ����
static QueueHandle_t xQueuePrint = NULL; 

// log����
SemaphoreHandle_t xMutexLog = NULL;

// �����ö�ʱ��
TimerHandle_t xTimersBox_Test[NUM_TEST_TIMERS];
static void vTimerTestCallback(void* parameter);

void service_init(void)
{
    BaseType_t xReturn = pdPASS;
	uint32_t x;

	xQueuePrint = xQueueCreate( PRINT_QUEUE_LENGTH, PRINT_QUEUE_ITEM_SIZE );
	if( xQueuePrint == NULL ) { printf("\r\nxQueuePrint create failed"); }

	xReturn = xTaskCreate( (TaskFunction_t) printf_entry,
		                   (const char *) "entry_printf",
		                   (unsigned short) PRINTF_STACK,
		                   (void *) NULL,
		                   (UBaseType_t) PRINTF_PRIO,
		                   (TaskHandle_t *) &h_printf_entry );
	if( xReturn != pdPASS ) { printf("\r\nprintf_entry create failed"); }

	xReturn = xTaskCreate( (TaskFunction_t) cmd_entry,
		                   (const char *) "entry_cmd",
		                   (unsigned short) CMD_STACK,
		                   (void *) NULL,
		                   (UBaseType_t) CMD_PRIO,
		                   (TaskHandle_t *) &h_cmd_entry );
	if( xReturn != pdPASS ) { printf("\r\ncmd_entry create failed"); }

    xMutexLog = xSemaphoreCreateMutex();
    if( xMutexLog == NULL ) { kprintf("\r\ncreate log mutex failed");}
	
	// create test timers
	for( x = 0; x < NUM_TEST_TIMERS; x++ )
	{
		xTimersBox_Test[ x ] = xTimerCreate( "Timer",
										      2000,
										      pdFALSE,
										      ( void * const) x,
										      (TimerCallbackFunction_t) vTimerTestCallback
										   );
		if( xTimersBox_Test[ x ] == NULL )
		{
			/* The timer was not created. */
			kprintf("\r\nnum of %d test-timers create failed", x);
		}
	}
}

static void vTimerTestCallback(void* parameter)
{
	uint32_t ulTimerID;
	
	ulTimerID = (uint32_t)pvTimerGetTimerID( parameter );
	switch( ulTimerID )
	{
		case TEST_TIMER_ID_CAN:
			if( g_e_can_test_status == CAN_TEST_CAN_COM ) {
				// timeout, can test failed
				xTaskNotify( h_cmd_entry, NOTIFY_TSK_CMD_CAN_TEST_FAILED, eSetBits );
			}else if( g_e_can_test_status == CAN_TEST_PMBUS_COM ) {
				// timeout, can test failed
				xTaskNotify( h_cmd_entry, NOTIFY_TSK_CMD_PMBUS_TEST_FAILED, eSetBits );
			}
			break;
		case TEST_TIMER_ID_485:
			// timeout, 485 test failed
			xTaskNotify( h_cmd_entry, NOTIFY_TSK_CMD_485_TEST_FAILED, eSetBits );
			break;
		case TEST_TIMER_ID_ETHERNET:
			// timeout, ethernet test failed
			xTaskNotify( h_cmd_entry, NOTIFY_TSK_CMD_ETHERNET_TEST_FAILED, eSetBits );
			break;
		default:
			break;
	}
}


void myprintf(const char *fmt, ...)
{
    va_list args;
    int32_t length;
	uint8_t msg[500];
	uint16_t idx = 0;

taskENTER_CRITICAL();

	va_start(args, fmt);
    length = vsnprintf( (char *)msg, (sizeof(msg) - 1), fmt, args ); // ��һ��Ϊ�˴���ַ�����β\0
	va_end(args);

//	while( (msg[idx] != '\0') && ( idx <= length ))
//	    com_cmd->send_byte( msg[idx ++] ); 

//    xSemaphoreTake( xMutexLog, portMAX_DELAY );
	printf("%s", msg);
//	xSemaphoreGive(xMutexLog);

	taskEXIT_CRITICAL();
}

// ��������С100�����洢99���ַ�
// ���������ж���ʹ��
void kprintf(const char *fmt, ...)
{
    va_list args;
    int32_t length;
	
#ifdef __DEBUG
	PrintMSG msg;
	
    xSemaphoreTake( xMutexLog, portMAX_DELAY );

	va_start(args, fmt);
    length = vsnprintf( (char *)msg, (sizeof(msg) - 1), fmt, args ); // ��һ��Ϊ�˴���ַ�����β\0
	va_end(args);
	if( length < 0 ) { printf("\r\nvsnprintf failed\n"); return; }
	
	// ��ֹprintf_entry����û�д��������øú�������ϵͳ�ҵ�
	if( xQueuePrint == NULL)
	{
	    printf(fmt, args);
		xSemaphoreGive(xMutexLog);
		return;
	}
	
	// ���͸���ӡ����
	xQueueSend( xQueuePrint,
                &msg,
	            pdMS_TO_TICKS( 10 ) );

	xSemaphoreGive(xMutexLog);
#else
	uint8_t msg[500];
	uint16_t idx = 0;

    xSemaphoreTake( xMutexLog, portMAX_DELAY );
	
	va_start(args, fmt);
    length = vsnprintf( (char *)msg, (sizeof(msg) - 1), fmt, args ); // ��һ��Ϊ�˴���ַ�����β\0
	va_end(args);
	if( length < 0 ) {
	    printf("\r\nvsnprintf failed\n");
	    xSemaphoreGive(xMutexLog);
		return;
	}
	
	while( (msg[idx] != '\0') && ( idx <= length ))
	    com_cmd->send_byte( msg[idx ++] ); 

	xSemaphoreGive(xMutexLog);
#endif
}


void delay_ms( __IO uint32_t _T )
{
	__IO uint32_t i;
	while ( _T-- )
		for ( i = 0; i < 10280; i++);
}

void delay_us( __IO uint32_t _T )
{
	__IO uint32_t i;
	while ( _T-- )
		for ( i = 0; i < 11; i++);
}

static void show_version(void)
{
    printf("\r\n���̰汾:%s\r\n�豸�ͺ�:%s\r\n�̼��汾:%s\r\n����汾:%s", ver_proj, ver_dev, ver_sw, ver_hmi);
}

void show_sys_mem( void )
{
	uint32_t mem_size;
  
	mem_size = configTOTAL_HEAP_SIZE;
	printf("\r\n\r\nϵͳ���ڴ��СΪ:%ld �ֽ�\n", mem_size*2 );

    mem_size = xPortGetFreeHeapSize();
    printf("ϵͳ��ǰʣ���ڴ��СΪ��%ld �ֽ�\n", mem_size);

}

#ifdef __DEBUG
void printf_entry( void *pvParameter )
{	
//uint32_t ulNotifiedValue;
uint16_t idx = 0;
	
PrintMSG xMessage;
BaseType_t xReturn = pdTRUE;
	
	for( ;; )
	{
		xReturn = xQueueReceive( xQueuePrint, &xMessage, portMAX_DELAY );
		if( pdTRUE == xReturn )
		{
			idx = 0;
			while( (xMessage[idx ++] != '\0') && (idx < PRINT_QUEUE_ITEM_SIZE) )
			{
				com_cmd->send_byte( xMessage[(idx-1)] );
			}
		}
		else
		{
			printf("\r\nqueue error\r\n");
		}
	}
}

#endif

static void process_cmd(void)
{
uint8_t buffer[60];
uint8_t buffer_part1[15];
uint8_t buffer_part2[15];
uint8_t buffer_part3[15];
uint8_t buffer_part4[15];
uint8_t *ptr = buffer;
uint8_t rx_len = 0;
uint8_t rx_len_tmp = 0;
uint8_t idx = 0;
	
	ptr = buffer;
	rx_len = com_cmd->get_rx_cnt();
	rx_len_tmp = rx_len;
	while( rx_len_tmp --)
	{
		sprintf((char*)ptr++, "%c",  com_cmd->read());
	}
	com_cmd->reset_rx();

	if( strchr( (const char *)buffer, '_') != NULL )
	{
		// ȡ����һ����
		ptr = buffer_part1;
		for( idx = 0; ( ( idx < ( sizeof( buffer ) / sizeof( buffer[0] ) ) ) && (buffer[idx] != '_') ); idx++ )
		{
			*ptr ++ = buffer[idx];
			rx_len --;
		}
		*ptr = '\0';
//					printf("\r\npart1:%s, left len:%d\n", buffer_part1, rx_len);
		
		// ȡ���ڶ�����
		if( rx_len != 0 )
		{
			ptr = buffer_part2;
			for( idx++; ( ( buffer[idx] != '_') && ( buffer[idx] != '\0' ) ); idx++ )
			{
				*ptr ++ = buffer[idx];
				rx_len --;
			}

			rx_len--;

			*ptr = '\0';
//					    printf("\r\npart2:%s, left_len:%d\n", buffer_part2, rx_len);
		}
		
		// ȡ����������
		if( rx_len != 0 )
		{
			ptr = buffer_part3;
			for( idx++; ( ( buffer[idx] != '_') && ( buffer[idx] != '\0' ) ); idx++ )
			{
				*ptr ++ = buffer[idx];
				rx_len --;
			}

			rx_len --;

			*ptr = '\0';
//					    printf("\r\npart3:%s, left_len:%d\n", buffer_part3, rx_len);
		}

		// ȡ�����Ĳ���
		if( rx_len != 0 )
		{
			ptr = buffer_part4;
			for( idx++; ( ( buffer[idx] != '_') && ( buffer[idx] != '\0' ) ); idx++ )
			{
				*ptr ++ = buffer[idx];
				rx_len --;
			}
			*ptr = '\0';
//					    printf("\r\npart4:%s, left_len:%d\n", buffer_part4, rx_len);
		}
	}
				
	if( strcmp((const char*)buffer, "ps") == 0 ) 
	{
		print_tsk_status();
	}
	else if( strcmp((const char*)buffer, "ver") == 0 ) 
	{
		show_version();
	}
	else if( strcmp((const char*)buffer, "mem") == 0 ) 
	{	
		show_sys_mem();
	}
	else if( strcmp((const char*)buffer, "pmbus_comm") == 0 ) 
	{
		for( idx = 0; idx < 8; idx ++ )
		{
			if( ChkAddrDev( addr_list[ idx ] ) )
			{
				printf("\r\ndev %d online", idx+1);
			}
			else
			{
				printf("dev %d pmbus comm failed\n", idx+1);
				vTaskDelay( pdMS_TO_TICKS( 20 ) );
			}
		}
	}
	else if( strcmp((const char*)buffer, "gz_test com uart_debug") == 0 ) 
	{	
		printf("gz_test com ack uart_debug");
	}
	else if( strcmp((const char*)buffer, "gz_test com ethernet") == 0 ) 
	{
	    xTaskNotify( h_ext_wifi_entry, NOTIFY_TSK_WIFI_TEST, eSetBits );
	}
	else if( strcmp((const char*)buffer, "gz_test com 485") == 0 ) 
	{	
	    xTaskNotify( h_ext_bms_comm_entry, NOTIFY_TSK_BMS_TEST, eSetBits );
	}
	else if( strcmp((const char*)buffer, "gz_test com can") == 0 ) 
	{	
	    xTaskNotify( h_can_comm_entry, NOTIFY_TSK_CAN_TEST, eSetBits );
	}else if( strcmp((const char*)buffer, "gz_test com pmbus") == 0 ) 
	{	
		for( idx = 0; idx < 8; idx ++ )
		{
			if( ChkAddrDev( addr_list[ idx ] ) )
			{
				printf("gz_test com ack pmbus");
				return;
			}
			vTaskDelay( pdMS_TO_TICKS( 20 ) );
		}
		printf("gz_test com nack pmbus");
	}
}

static void process_test_result(uint32_t result)
{
	char tmp_buff[50];
	uint32_t len = 0, i = 0;

	switch( result )
	{
		case NOTIFY_TSK_CMD_CAN_TEST_SUCCESS:
			sprintf(tmp_buff, "gz_test com ack can");
		    break;
	    case NOTIFY_TSK_CMD_CAN_TEST_FAILED:
			sprintf(tmp_buff, "gz_test com nack can");
		    break;
	    case NOTIFY_TSK_CMD_485_TEST_SUCCESS:
			sprintf(tmp_buff, "gz_test com ack 485");
		    break;
	    case NOTIFY_TSK_CMD_485_TEST_FAILED:
			sprintf(tmp_buff, "gz_test com nack 485 %02X", g_u32_test_result_bms);
		    break;
	    case NOTIFY_TSK_CMD_ETHERNET_TEST_SUCCESS:
			sprintf(tmp_buff, "gz_test com ack ethernet");
		    break;
	    case NOTIFY_TSK_CMD_ETHERNET_TEST_FAILED:
			sprintf(tmp_buff, "gz_test com nack ethernet");
		    break;
		case NOTIFY_TSK_CMD_PMBUS_TEST_SUCCESS:
			sprintf(tmp_buff, "gz_test com ack pmbus");
		    break;
	    case NOTIFY_TSK_CMD_PMBUS_TEST_FAILED:
			sprintf(tmp_buff, "gz_test com nack pmbus");
		    break;
		default:
			break;
	}

	len = strlen(tmp_buff);
	for(i=0; i<len; i++)
	{
		com_cmd->write(tmp_buff[i]);
	}

    com_cmd->send();
}

void cmd_entry( void *pvParameter )
{
const TickType_t xMaxBlockTime = portMAX_DELAY; /*pdMS_TO_TICKS( 500 );*/
//const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 1000 );
BaseType_t xResult;
uint32_t ulNotifiedValue;

	for( ;; )
	{
		/* Wait to be notified of an interrupt. */
        xResult = xTaskNotifyWait( (uint32_t) pdFALSE, /* Don't clear bits on entry. */
                                   (uint32_t) ULONG_MAX, /* Clear all bits on exit. */
                                   (uint32_t *) &ulNotifiedValue, /* Stores the notified value. */
                                   (TickType_t) xMaxBlockTime );
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
				process_cmd();
            }
			else
			{
			    process_test_result( ulNotifiedValue );
			}
		}
        else
        {
            /* Did not receive a notification within the expected time. */
//            prvCheckForErrors();
			printf("hello,gz\r\n");
        }
	}
}

#define tskRUNNING_CHAR		( 'X' )
#define tskBLOCKED_CHAR		( 'B' )
#define tskREADY_CHAR		( 'R' )
#define tskDELETED_CHAR		( 'D' )
#define tskSUSPENDED_CHAR	( 'S' )

static char *pWriteNameToBuffer( char *pcBuffer, const char *pcTaskName )
{
size_t x;

	/* Start by copying the entire string. */
	strcpy( pcBuffer, pcTaskName );

	/* Pad the end of the string with spaces to ensure columns line up when
	printed out. */
	for( x = strlen( pcBuffer ); x < ( size_t ) ( configMAX_TASK_NAME_LEN - 1 ); x++ )
	{
		pcBuffer[ x ] = ' ';
	}

	/* Terminate. */
	pcBuffer[ x ] = 0x00;

	/* Return the new end of string. */
	return &( pcBuffer[ x ] );
}

void print_tsk_status(void)
{
	TaskStatus_t *pxTaskStatusArray;
	volatile UBaseType_t uxArraySize, x;
	char cStatus;	
	char pcBuffer[configMAX_TASK_NAME_LEN] = {0};
	
	/* Take a snapshot of the number of tasks in case it changes while this
	function is executing. */
	uxArraySize = uxTaskGetNumberOfTasks();

	/* Allocate an array index for each task.  NOTE!  if
	configSUPPORT_DYNAMIC_ALLOCATION is set to 0 then pvPortMalloc() will
	equate to NULL. */
	pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

	if( pxTaskStatusArray != NULL )
	{
		/* Generate the (binary) data. */
		uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, NULL );

		printf("\r\n\r\n\r\n\r\n--------------------����״̬--------------------");
		printf("\r\n��������: %ld", uxArraySize);
		printf("\r\n�����б�:\r\n����\t\t\t״̬\t���ȼ�\t����\t���\r\n");
		/* Create a human readable table from the binary data. */
		for( x = 0; x < uxArraySize; x++ )
		{
			switch( pxTaskStatusArray[ x ].eCurrentState )
			{
				case eRunning:		cStatus = tskRUNNING_CHAR;
									break;

				case eReady:		cStatus = tskREADY_CHAR;
									break;

				case eBlocked:		cStatus = tskBLOCKED_CHAR;
									break;

				case eSuspended:	cStatus = tskSUSPENDED_CHAR;
									break;

				case eDeleted:		cStatus = tskDELETED_CHAR;
									break;

				case eInvalid:		/* Fall through. */
				default:			/* Should not get here, but it is included
									to prevent static checking errors. */
									cStatus = ( char ) 0x00;
									break;
			}

			pWriteNameToBuffer( pcBuffer, pxTaskStatusArray[ x ].pcTaskName );
			/* Write the rest of the string. */
			printf("%s\t%c\t%u\t%u\t%u\r\n", pcBuffer, cStatus, ( unsigned int ) pxTaskStatusArray[ x ].uxCurrentPriority, ( unsigned int ) pxTaskStatusArray[ x ].usStackHighWaterMark, ( unsigned int ) pxTaskStatusArray[ x ].xTaskNumber );
		}

		/* Free the array again.  NOTE!  If configSUPPORT_DYNAMIC_ALLOCATION
		is 0 then vPortFree() will be #defined to nothing. */
		vPortFree( pxTaskStatusArray );
	}

}


/* ����ڴ��Ƿ�Խ�磬����ʱ�򿪣���ʽ����ʱ�ر�
 * �رշ�������FreeRTOSConfig.h�У���configCHECK_FOR_STACK_OVERFLOW��ֵ��Ϊ0����
 */
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
	printf("\r\nStack Over Flow!!!");
	print_tsk_status();
}

void vApplicationMallocFailedHook( void )
{
    printf("\r\nmalloc failed");
}

