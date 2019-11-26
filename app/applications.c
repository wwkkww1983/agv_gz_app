#include "applications.h"
#include "limits.h"
//#include "Ext_Interface.h"
#include "eeprom.h"
#include "service.h"

#define    STARTUP_STACK    256
#define    STARTUP_PRIO     1
static TaskHandle_t h_startup_entry = NULL;
static void startup_entry(void *pvParameters);


static void startup_entry(void *pvParameters)
{
    BaseType_t xReturn = pdPASS;
	
	/* disable interrupt first */
    taskENTER_CRITICAL();
	
	/* init board */
    hw_borad_init();
    service_init();

	kprintf("agv充电站工装测试程序");

	xReturn = xTaskCreate( (TaskFunction_t) led_entry,
		                   (const char *) "entry_led",
		                   (unsigned short) LED_STACK,
		                   (void *) NULL,
		                   (UBaseType_t) LED_PRIO,
		                   (TaskHandle_t *) &h_led_entry );
	if(xReturn != pdPASS) { kprintf("\r\nentry_led create failed"); return; }

	xReturn = xTaskCreate( (TaskFunction_t) key_entry,
		                   (const char *) "entry_key",
		                   (unsigned short) KEY_STACK,
		                   (void *) NULL,
		                   (UBaseType_t) KEY_PRIO,
		                   (TaskHandle_t *) &h_key_entry );
	if(xReturn != pdPASS) { kprintf("\r\nentry_key create failed"); return; }
	
	xReturn = xTaskCreate( (TaskFunction_t) bms_comm_entry,
		                   (const char *) "entry_ext_bms_interface",
		                   (unsigned short) EXT_BMS_COMM_STACK,
		                   (void *) NULL,
		                   (UBaseType_t) EXT_BMS_COMM_PRIO,
		                   (TaskHandle_t *) &h_ext_bms_comm_entry );
	if( xReturn != pdPASS ) { kprintf("\r\next bms comm create failed"); }
	
	xReturn = xTaskCreate( (TaskFunction_t) can_comm_entry,
		                   (const char *) "entry can comm",
		                   (unsigned short) CAN_COMM_STACK,
		                   (void *) NULL,
		                   (UBaseType_t) CAN_COMM_PRIO,
		                   (TaskHandle_t *) &h_can_comm_entry );
	if( xReturn != pdPASS ) { kprintf("\r\ncan comm entry create failed"); }
	
	xReturn = xTaskCreate( (TaskFunction_t) ext_wifi_entry,
		                   (const char *) "entry_ext_wifi",
		                   (unsigned short) EXT_WIFI_STACK,
		                   (void *) NULL,
		                   (UBaseType_t) EXT_WIFI_PRIO,
		                   (TaskHandle_t *) &h_ext_wifi_entry );
	if( xReturn != pdPASS ) { kprintf("\r\next_wifi create failed"); }	
	
	xReturn = xTaskCreate( (TaskFunction_t) touch_screen_entry,
		                   (const char *) "entry_touch_screen",
		                   (unsigned short) TOUCH_SCREEN_STACK,
		                   (void *) NULL,
		                   (UBaseType_t) TOUCH_SCREEN_PRIO,
		                   (TaskHandle_t *) &h_touch_screen_entry );
	if( xReturn != pdPASS ) { kprintf("\r\ntouch_screen_entry create failed"); }
	
	// 打开can的接收中断
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN1, CAN_IT_FF0, ENABLE);
	CAN_ITConfig(CAN1, CAN_IT_FF1, ENABLE);
	
	/* 删除自身 */
	vTaskDelete( h_startup_entry );
	
	/* enable interrupt */
	taskEXIT_CRITICAL();
}

int main(void)
{
	BaseType_t xReturn = pdPASS;
	xReturn = xTaskCreate( (TaskFunction_t) startup_entry,
		                   (const char *) "entry_startup",
		                   (unsigned short) STARTUP_STACK,
		                   (void *) NULL,
		                   (UBaseType_t) STARTUP_PRIO,
		                   (TaskHandle_t *) &h_startup_entry );
	if( xReturn != pdPASS ) { printf("\r\n init failed"); return -1; }
	else vTaskStartScheduler();

	for( ;; );
}

