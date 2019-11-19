#include "stm32f10x.h"
#include "stdarg.h"
#include "string.h"
#include "stdlib.h"

#include "FreeRTOS.h"
#include "task.h"
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


// 向打印任务传输数据的消息队列
static QueueHandle_t xQueuePrint = NULL; 

// log互斥
SemaphoreHandle_t xMutexLog = NULL;


void service_init(void)
{
    BaseType_t xReturn = pdPASS;

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
	
}

void myprintf(const char *fmt, ...)
{
    va_list args;
    int32_t length;
	uint8_t msg[500];
	uint16_t idx = 0;

taskENTER_CRITICAL();

	va_start(args, fmt);
    length = vsnprintf( (char *)msg, (sizeof(msg) - 1), fmt, args ); // 减一是为了存放字符串结尾\0
	va_end(args);

//	while( (msg[idx] != '\0') && ( idx <= length ))
//	    com_cmd->send_byte( msg[idx ++] ); 

//    xSemaphoreTake( xMutexLog, portMAX_DELAY );
	printf("%s", msg);
//	xSemaphoreGive(xMutexLog);

	taskEXIT_CRITICAL();
}

// 缓冲区大小100，最多存储99个字符
// 不可以在中断中使用
void kprintf(const char *fmt, ...)
{
    va_list args;
    int32_t length;
	
#ifdef __DEBUG
	PrintMSG msg;
	
    xSemaphoreTake( xMutexLog, portMAX_DELAY );

	va_start(args, fmt);
    length = vsnprintf( (char *)msg, (sizeof(msg) - 1), fmt, args ); // 减一是为了存放字符串结尾\0
	va_end(args);
	if( length < 0 ) { printf("\r\nvsnprintf failed\n"); return; }
	
	// 防止printf_entry任务还没有创建，调用该函数导致系统挂掉
	if( xQueuePrint == NULL)
	{
	    printf(fmt, args);
		xSemaphoreGive(xMutexLog);
		return;
	}
	
	// 发送给打印任务
	xQueueSend( xQueuePrint,
                &msg,
	            pdMS_TO_TICKS( 10 ) );

	xSemaphoreGive(xMutexLog);
#else
	uint8_t msg[500];
	uint16_t idx = 0;

    xSemaphoreTake( xMutexLog, portMAX_DELAY );
	
	va_start(args, fmt);
    length = vsnprintf( (char *)msg, (sizeof(msg) - 1), fmt, args ); // 减一是为了存放字符串结尾\0
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
    printf("\r\n工程版本:%s\r\n设备型号:%s\r\n固件版本:%s\r\n界面版本:%s", ver_proj, ver_dev, ver_sw, ver_hmi);
}

void show_sys_mem( void )
{
	uint32_t mem_size;
  
	mem_size = configTOTAL_HEAP_SIZE;
	printf("\r\n\r\n系统总内存大小为:%ld 字节\n", mem_size*2 );

    mem_size = xPortGetFreeHeapSize();
    printf("系统当前剩余内存大小为：%ld 字节\n", mem_size);

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


void cmd_entry( void *pvParameter )
{
//const TickType_t xMaxBlockTime = portMAX_DELAY; /*pdMS_TO_TICKS( 500 );*/
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 1000 );
BaseType_t xResult;
uint32_t ulNotifiedValue;
uint8_t rx_len = 0;
uint8_t rx_len_tmp = 0;
uint8_t idx = 0;

uint8_t buffer[60];
uint8_t buffer_part1[15];
uint8_t buffer_part2[15];
uint8_t buffer_part3[15];
uint8_t buffer_part4[15];
uint8_t *ptr = buffer;

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
					// 取出第一部分
					ptr = buffer_part1;
					for( idx = 0; ( ( idx < ( sizeof( buffer ) / sizeof( buffer[0] ) ) ) && (buffer[idx] != '_') ); idx++ )
					{
						*ptr ++ = buffer[idx];
						rx_len --;
					}
					*ptr = '\0';
//					printf("\r\npart1:%s, left len:%d\n", buffer_part1, rx_len);
					
					// 取出第二部分
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
					
					// 取出第三部分
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

					// 取出第四部分
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

		printf("\r\n\r\n\r\n\r\n--------------------任务状态--------------------");
		printf("\r\n任务数量: %ld", uxArraySize);
		printf("\r\n任务列表:\r\n名称\t\t\t状态\t优先级\t空闲\t序号\r\n");
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


/* 检查内存是否越界，调试时打开，正式发布时关闭
 * 关闭方法，在FreeRTOSConfig.h中，把configCHECK_FOR_STACK_OVERFLOW的值改为0即可
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

