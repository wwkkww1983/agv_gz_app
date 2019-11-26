#include "board.h"
#include "uart.h"
#include "gpio.h"
#include "bsp_rpb1600_48.h"
#include "bsp_pmbus_stack.h"
#include "eeprom.h"
#include "adc.h"
#include "SPI_WK2168.h"
#include "CAN.h"
#include "tim.h"
#include "exti.h"
#include "iwdg.h"   


static void gz_test_gpio_init(void)
{
	GPIO_RESET( RELAY_AC_INPUT_1_GPIO_PORT, RELAY_AC_INPUT_1_GPIO_PIN |
									      RELAY_AC_INPUT_2_GPIO_PIN | 
									      RELAY_AC_INPUT_3_GPIO_PIN | 
									      RELAY_AC_INPUT_4_GPIO_PIN | 
									      RELAY_AC_INPUT_5_GPIO_PIN | 
									      RELAY_AC_INPUT_6_GPIO_PIN | 
									      RELAY_AC_INPUT_7_GPIO_PIN | 
									      RELAY_AC_INPUT_8_GPIO_PIN );

	GPIO_RESET( RELAY_DC_OUTPUT_1_GPIO_PORT, RELAY_DC_OUTPUT_1_GPIO_PIN |
	  								         RELAY_DC_OUTPUT_2_GPIO_PIN | 
	  								         RELAY_DC_OUTPUT_3_GPIO_PIN | 
	  								         RELAY_DC_OUTPUT_4_GPIO_PIN | 
	  								         RELAY_DC_OUTPUT_5_GPIO_PIN | 
	  								         RELAY_DC_OUTPUT_6_GPIO_PIN | 
	  								         RELAY_DC_OUTPUT_7_GPIO_PIN | 
	  								         RELAY_DC_OUTPUT_8_GPIO_PIN );

	GPIO_RESET(ON_OFF_1_GPIO_PORT,  ON_OFF_1_GPIO_PIN |
									ON_OFF_2_GPIO_PIN | 
									ON_OFF_3_GPIO_PIN |
									ON_OFF_4_GPIO_PIN |
									ON_OFF_5_GPIO_PIN |
									ON_OFF_6_GPIO_PIN |
									ON_OFF_7_GPIO_PIN |
									ON_OFF_8_GPIO_PIN );

	GPIO_RESET( RELAY_CHMOD_D1_GPIO_PORT, RELAY_CHMOD_D1_GPIO_PIN |
	  								      RELAY_CHMOD_D2_GPIO_PIN |
	  								      RELAY_CHMOD_D3_GPIO_PIN |
	  								      RELAY_CHMOD_D4_GPIO_PIN |
	  								      RELAY_CHMOD_D5_GPIO_PIN |
	  								      RELAY_CHMOD_D6_GPIO_PIN |
	  								      RELAY_CHMOD_D7_GPIO_PIN |
	  								      RELAY_CHMOD_D8_GPIO_PIN );

	// led-green
	GPIO_RESET( LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PIN );
	// led-red
	GPIO_RESET( LED_RED_GPIO_PORT, LED_RED_GPIO_PIN );
	// led-wifi
	GPIO_RESET( LED_WIFI_CONN_GPIO_PORT, LED_WIFI_CONN_GPIO_PIN );
	// fan1-speed control
	GPIO_RESET( FAN_1_SPEED_CTL_GPIO_PORT, FAN_1_SPEED_CTL_GPIO_PIN );
	// fan2-speed control
	GPIO_RESET( FAN_2_SPEED_CTL_GPIO_PORT, FAN_2_SPEED_CTL_GPIO_PIN );
}

void hw_borad_init(void)
{
	// FreeRTOS默认使用中断分组4，NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );	
	
    uart_init();
	sInitGpio();
    gz_test_gpio_init();
	
	sInitPMBus();

	sInitRCB1600();
//	
//	eeprom_init();

	adc_init();

    WK2168_Sys_Init();

	sCanInitial();

//	basic_tim_init();

    exti_init();
//	
//	iwdg_init();
}

