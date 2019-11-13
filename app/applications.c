#include "applications.h"
#include "limits.h"
//#include "Ext_Interface.h"
#include "eeprom.h"

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

	kprintf("agv³äµçÕ¾¹¤×°£º´ý²â°å²âÊÔ³ÌÐò");

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
	
	/* É¾³ý×ÔÉí */
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

