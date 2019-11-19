#include "applications.h"
#include "gpio.h"
#include "key.h"
#include "service.h"

TaskHandle_t h_key_entry = NULL;
void key_entry(void *pvParameters);

void key_entry(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS( KEY_SCAN_INTERVAL );
	uint32_t i = 0;
	uint32_t mask = 0;

	KeyTypeDef key_tab[] = { 
	{ DC_OK1_GPIO_PORT, DC_OK1_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ DC_OK2_GPIO_PORT, DC_OK2_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ DC_OK3_GPIO_PORT, DC_OK3_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ DC_OK4_GPIO_PORT, DC_OK4_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ DC_OK5_GPIO_PORT, DC_OK5_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ DC_OK6_GPIO_PORT, DC_OK6_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ DC_OK7_GPIO_PORT, DC_OK7_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ DC_OK8_GPIO_PORT, DC_OK8_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ T_ALARM1_GPIO_PORT, T_ALARM1_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ T_ALARM2_GPIO_PORT, T_ALARM2_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ T_ALARM3_GPIO_PORT, T_ALARM3_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ T_ALARM4_GPIO_PORT, T_ALARM4_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ T_ALARM5_GPIO_PORT, T_ALARM5_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ T_ALARM6_GPIO_PORT, T_ALARM6_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ T_ALARM7_GPIO_PORT, T_ALARM7_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
	{ T_ALARM8_GPIO_PORT, T_ALARM8_GPIO_PIN, 0, 0, 0xff, KEY_LOW, KEY_OFF, KEY_CHK_PRESS },
    { FAN_1_ERR_CHK_GPIO_PORT, FAN_1_ERR_CHK_GPIO_PIN, 0, 0, 0xff, KEY_HIGH, KEY_OFF, KEY_CHK_PRESS },
    { FAN_2_ERR_CHK_GPIO_PORT, FAN_2_ERR_CHK_GPIO_PIN, 0, 0, 0xff, KEY_HIGH, KEY_OFF, KEY_CHK_PRESS }
	};
	
xLastWakeTime = xTaskGetTickCount();

	while (1)
	{
		for( i=0; i<18; i++ )
		{
			mask = (uint32_t)(1 << i);
			switch( key_tab[i].chk_step )
			{
				case KEY_CHK_PRESS:
			        if( key_detect( &key_tab[i] ) != KEY_OFF )
					{
						LOG_INFO_APP("\r\nkey:%d pressed, msk:0X%08X", i+1, mask);
						xTaskNotify( h_led_entry, mask, eSetBits );
					}
					break;
				case KEY_CHK_RELEASE:
			        if( key_detect( &key_tab[i] ) == KEY_OFF )
					{
						LOG_INFO_APP("\r\nkey:%d released, msk:0X%08X", i+1, mask);
						xTaskNotify( h_led_entry, mask, eSetBits );
					}
					break;
				default:
					break;
			}
		}

		vTaskDelayUntil( &xLastWakeTime, xPeriod );
	}
}
