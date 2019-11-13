#include "gpio.h"

#define    GPIO_ON    0x01
#define    GPIO_OFF   0x02

static void init_gpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd ( RELAY_AC_INPUT_1_GPIO_CLK   | \
							 RELAY_DC_OUTPUT_1_GPIO_CLK  | \
							 ON_OFF_1_GPIO_CLK           | \
							 RELAY_CHMOD_D1_GPIO_CLK     | \
							 DC_OK1_GPIO_CLK             | \
							 T_ALARM1_GPIO_CLK           | \
                             SPI_EXT_1_NSS_GPIO_CLK      | \
                             SPI_EXT_2_NSS_GPIO_CLK      | \
							 SPI_EXT_2_IRQ_GPIO_CLK      | \
							 SPI_EXT_2_RST_GPIO_CLK      | \
                             EXT_WIFI_RST_GPIO_CLK       | \
	                         STOP_GPIO_CLK               | \
                             LED_WIFI_CONN_GPIO_CLK      | \
							 FAN_1_SPEED_CTL_GPIO_CLK    | \
							 FAN_2_SPEED_CTL_GPIO_CLK
/*
							 BAT_CONN_GPIO_CLK |
							 STOP_GPIO_CLK	   |
							 RELAY_GPIO_CLK    |
							 LED_GREEN_GPIO_CLK   |
	                         LED_RED_GPIO_CLK, ENABLE |    */
							 , ENABLE );
	
// 输入相关
/*
	// AC-OK
	GPIO_InitStructure.GPIO_Pin = ( AC_OK1_GPIO_PIN |
									AC_OK2_GPIO_PIN |
									AC_OK3_GPIO_PIN |
									AC_OK4_GPIO_PIN |
									AC_OK5_GPIO_PIN );
	GPIO_InitStructure.GPIO_Mode = AC_OK1_GPIO_Mode; 
	GPIO_Init(AC_OK1_GPIO_PORT, &GPIO_InitStructure);
	


	// Bat conn check
	GPIO_InitStructure.GPIO_Pin = BAT_CONN_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = BAT_CONN_GPIO_Mode; 
	GPIO_Init(BAT_CONN_GPIO_PORT, &GPIO_InitStructure);
*/

    // dc-ok, high-err, low - normal
	GPIO_InitStructure.GPIO_Pin = ( DC_OK1_GPIO_PIN |
									DC_OK2_GPIO_PIN |
									DC_OK3_GPIO_PIN |
									DC_OK4_GPIO_PIN |
									DC_OK5_GPIO_PIN |
									DC_OK6_GPIO_PIN |
									DC_OK7_GPIO_PIN |
									DC_OK8_GPIO_PIN );
	GPIO_InitStructure.GPIO_Mode = DC_OK1_GPIO_Mode; 
	GPIO_Init(DC_OK1_GPIO_PORT, &GPIO_InitStructure);

    // t-alarm input, high-err, low - normal
	GPIO_InitStructure.GPIO_Pin = ( T_ALARM1_GPIO_PIN |
									T_ALARM2_GPIO_PIN |
									T_ALARM3_GPIO_PIN |
									T_ALARM4_GPIO_PIN |
									T_ALARM5_GPIO_PIN |
									T_ALARM6_GPIO_PIN |
									T_ALARM7_GPIO_PIN |
									T_ALARM8_GPIO_PIN );
	GPIO_InitStructure.GPIO_Mode = T_ALARM1_GPIO_Mode; 
	GPIO_Init( T_ALARM1_GPIO_PORT, &GPIO_InitStructure );

	// fan1 err_chk
	GPIO_InitStructure.GPIO_Pin = FAN_1_ERR_CHK_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = FAN_1_ERR_CHK_GPIO_Mode; 
	GPIO_Init( FAN_1_SPEED_CTL_GPIO_PORT, &GPIO_InitStructure );
	// fan2 err_chk
	GPIO_InitStructure.GPIO_Pin = FAN_2_ERR_CHK_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = FAN_2_ERR_CHK_GPIO_Mode; 
	GPIO_Init( FAN_2_SPEED_CTL_GPIO_PORT, &GPIO_InitStructure );


//	GPIO_EXTILineConfig( STOP_EXTI_PORTSOURCE, STOP_EXTI_PINSOURCE ); 
//	EXTI_InitStructure.EXTI_Line = STOP_EXTI_LINE;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // 双边沿触发
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);

// 输出相关

    // Relay AC input, high->open, low->close
// Relay AC input, high->open, low->close - 20190428 (最新)
	GPIO_InitStructure.GPIO_Pin = ( RELAY_AC_INPUT_1_GPIO_PIN |
									RELAY_AC_INPUT_2_GPIO_PIN |
									RELAY_AC_INPUT_3_GPIO_PIN |
									RELAY_AC_INPUT_4_GPIO_PIN |
									RELAY_AC_INPUT_5_GPIO_PIN |
									RELAY_AC_INPUT_6_GPIO_PIN |
									RELAY_AC_INPUT_7_GPIO_PIN |
									RELAY_AC_INPUT_8_GPIO_PIN );
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = RELAY_AC_INPUT_1_GPIO_Mode; 
	GPIO_Init( RELAY_AC_INPUT_1_GPIO_PORT, &GPIO_InitStructure );
	    // default, close!!!
	GPIO_RESET( RELAY_AC_INPUT_1_GPIO_PORT, RELAY_AC_INPUT_1_GPIO_PIN |
									      RELAY_AC_INPUT_2_GPIO_PIN | 
									      RELAY_AC_INPUT_3_GPIO_PIN | 
									      RELAY_AC_INPUT_4_GPIO_PIN | 
									      RELAY_AC_INPUT_5_GPIO_PIN | 
									      RELAY_AC_INPUT_6_GPIO_PIN | 
									      RELAY_AC_INPUT_7_GPIO_PIN | 
									      RELAY_AC_INPUT_8_GPIO_PIN );

// Relay DC output, high->close, low->open
// Relay DC output, low->close, high->open - 20190228 (最新)
	GPIO_InitStructure.GPIO_Pin = ( RELAY_DC_OUTPUT_1_GPIO_PIN |
									RELAY_DC_OUTPUT_2_GPIO_PIN |
									RELAY_DC_OUTPUT_3_GPIO_PIN |
									RELAY_DC_OUTPUT_4_GPIO_PIN |
									RELAY_DC_OUTPUT_5_GPIO_PIN |
									RELAY_DC_OUTPUT_6_GPIO_PIN |
									RELAY_DC_OUTPUT_7_GPIO_PIN |
									RELAY_DC_OUTPUT_8_GPIO_PIN );
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = RELAY_DC_OUTPUT_1_GPIO_Mode; 
	GPIO_Init( RELAY_DC_OUTPUT_1_GPIO_PORT, &GPIO_InitStructure );
	    // default, close!!!
	GPIO_RESET( RELAY_DC_OUTPUT_1_GPIO_PORT, RELAY_DC_OUTPUT_1_GPIO_PIN |
	  								         RELAY_DC_OUTPUT_2_GPIO_PIN | 
	  								         RELAY_DC_OUTPUT_3_GPIO_PIN | 
	  								         RELAY_DC_OUTPUT_4_GPIO_PIN | 
	  								         RELAY_DC_OUTPUT_5_GPIO_PIN | 
	  								         RELAY_DC_OUTPUT_6_GPIO_PIN | 
	  								         RELAY_DC_OUTPUT_7_GPIO_PIN | 
	  								         RELAY_DC_OUTPUT_8_GPIO_PIN );

	// ON/OFF control
	GPIO_InitStructure.GPIO_Pin = ( ON_OFF_1_GPIO_PIN |
									ON_OFF_2_GPIO_PIN | 
									ON_OFF_3_GPIO_PIN |
									ON_OFF_4_GPIO_PIN |
									ON_OFF_5_GPIO_PIN |
									ON_OFF_6_GPIO_PIN |
									ON_OFF_7_GPIO_PIN |
									ON_OFF_8_GPIO_PIN );
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = ON_OFF_1_GPIO_Mode; 
	GPIO_Init(ON_OFF_1_GPIO_PORT, &GPIO_InitStructure);
	    // v2, modifid, set low, output closed
	GPIO_RESET(ON_OFF_1_GPIO_PORT,  ON_OFF_1_GPIO_PIN |
									ON_OFF_2_GPIO_PIN | 
									ON_OFF_3_GPIO_PIN |
									ON_OFF_4_GPIO_PIN |
									ON_OFF_5_GPIO_PIN |
									ON_OFF_6_GPIO_PIN |
									ON_OFF_7_GPIO_PIN |
									ON_OFF_8_GPIO_PIN );
	
	
    // ChgrMod select, high->curve, low->pmbus control
	GPIO_InitStructure.GPIO_Pin = ( RELAY_CHMOD_D1_GPIO_PIN |
									RELAY_CHMOD_D2_GPIO_PIN |
									RELAY_CHMOD_D3_GPIO_PIN |
									RELAY_CHMOD_D4_GPIO_PIN |
									RELAY_CHMOD_D5_GPIO_PIN |
									RELAY_CHMOD_D6_GPIO_PIN |
									RELAY_CHMOD_D7_GPIO_PIN |
									RELAY_CHMOD_D8_GPIO_PIN );
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = RELAY_CHMOD_D1_GPIO_Mode; 
	GPIO_Init( RELAY_CHMOD_D1_GPIO_PORT, &GPIO_InitStructure );
	    // default, charging curve
	GPIO_SET( RELAY_CHMOD_D1_GPIO_PORT, RELAY_CHMOD_D1_GPIO_PIN |
	  								      RELAY_CHMOD_D2_GPIO_PIN |
	  								      RELAY_CHMOD_D3_GPIO_PIN |
	  								      RELAY_CHMOD_D4_GPIO_PIN |
	  								      RELAY_CHMOD_D5_GPIO_PIN |
	  								      RELAY_CHMOD_D6_GPIO_PIN |
	  								      RELAY_CHMOD_D7_GPIO_PIN |
	  								      RELAY_CHMOD_D8_GPIO_PIN );


	// sys indicator led， 默认关闭
	// led - green, 置高-亮
	GPIO_InitStructure.GPIO_Pin = LED_GREEN_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = LED_GREEN_GPIO_Mode; 
	GPIO_Init(LED_GREEN_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_RESET( LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PIN );
	
	// led - red，置高-亮
	GPIO_InitStructure.GPIO_Pin = LED_RED_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = LED_RED_GPIO_Mode; 
	GPIO_Init(LED_RED_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_RESET( LED_RED_GPIO_PORT, LED_RED_GPIO_PIN );

	// led - wifi，置高-亮
	GPIO_InitStructure.GPIO_Pin = LED_WIFI_CONN_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = LED_WIFI_CONN_GPIO_Mode; 
	GPIO_Init(LED_WIFI_CONN_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_RESET( LED_WIFI_CONN_GPIO_PORT, LED_WIFI_CONN_GPIO_PIN );


// 485 comm ext , spi-1
	GPIO_InitStructure.GPIO_Pin = ( SPI_EXT_1_NSS_GPIO_PIN | SPI_EXT_1_RST_GPIO_PIN );
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = SPI_EXT_1_NSS_GPIO_MODE; 
	GPIO_Init(SPI_EXT_1_NSS_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = (	SPI_EXT_1_SCK_GPIO_PIN | 
									SPI_EXT_1_MISO_GPIO_PIN |
									SPI_EXT_1_MOSI_GPIO_PIN );
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = SPI_EXT_1_SCK_GPIO_MODE; 
	GPIO_Init(SPI_EXT_1_SCK_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_EXT_1_IRQ_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = SPI_EXT_1_IRQ_GPIO_MODE; 
	GPIO_Init(SPI_EXT_1_IRQ_GPIO_PORT, &GPIO_InitStructure);


// 485 comm ext , spi-2
	GPIO_InitStructure.GPIO_Pin = SPI_EXT_2_NSS_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = SPI_EXT_2_NSS_GPIO_MODE; 
	GPIO_Init(SPI_EXT_2_NSS_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = ( SPI_EXT_2_SCK_GPIO_PIN | 
									SPI_EXT_2_MISO_GPIO_PIN |
									SPI_EXT_2_MOSI_GPIO_PIN );
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = SPI_EXT_2_SCK_GPIO_MODE; 
	GPIO_Init(SPI_EXT_2_SCK_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_EXT_2_RST_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = SPI_EXT_2_RST_GPIO_MODE; 
	GPIO_Init(SPI_EXT_2_RST_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_EXT_2_IRQ_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = SPI_EXT_2_IRQ_GPIO_MODE; 
	GPIO_Init(SPI_EXT_2_IRQ_GPIO_PORT, &GPIO_InitStructure);

// ext wifi
	GPIO_InitStructure.GPIO_Pin = ( EXT_WIFI_RST_GPIO_PIN | 
									EXT_WIFI_RELOAD_GPIO_PIN );
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = EXT_WIFI_RST_GPIO_MODE; 
	GPIO_Init(EXT_WIFI_RST_GPIO_PORT, &GPIO_InitStructure);

	// rst default output high, if output set low>300ms, wifi-module reset
	GPIO_SET( EXT_WIFI_RST_GPIO_PORT, EXT_WIFI_RST_GPIO_PIN );
	// reload default output high, if output set low>3s, then set high, wifi-module restore and restart
	GPIO_SET( EXT_WIFI_RELOAD_GPIO_PORT, EXT_WIFI_RELOAD_GPIO_PIN );

	// rdy pin, input low -> wifi start finished
	// link pin, input low -> wifi connected 
	GPIO_InitStructure.GPIO_Pin = ( EXT_WIFI_RDY_GPIO_PIN | 
									EXT_WIFI_LINK_GPIO_PIN );
	GPIO_InitStructure.GPIO_Mode = EXT_WIFI_RDY_GPIO_MODE; 
	GPIO_Init(EXT_WIFI_RDY_GPIO_PORT, &GPIO_InitStructure);


// fan1 speed contrl
	GPIO_InitStructure.GPIO_Pin = FAN_1_SPEED_CTL_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = FAN_1_SPEED_CTL_GPIO_Mode; 
	GPIO_Init(FAN_1_SPEED_CTL_GPIO_PORT, &GPIO_InitStructure);
	
// fan2 speed contrl
	GPIO_InitStructure.GPIO_Pin = FAN_2_SPEED_CTL_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = FAN_2_SPEED_CTL_GPIO_Mode; 
	GPIO_Init(FAN_2_SPEED_CTL_GPIO_PORT, &GPIO_InitStructure);
#if 0	
	// emergency stop, low-stop
	GPIO_InitStructure.GPIO_Pin = STOP_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = STOP_GPIO_Mode; 
	GPIO_Init(STOP_GPIO_PORT, &GPIO_InitStructure);
	
	// Relay, higl level - open, low level - close
	GPIO_InitStructure.GPIO_Pin = RELAY_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = RELAY_GPIO_Mode; 
	GPIO_Init(RELAY_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_SET(RELAY_GPIO_PORT, RELAY_GPIO_PIN); //default, open

#endif

}

static void relay_ac_input( uint8_t num, uint8_t op )
{
#if 1
	uint16_t gpio_pin_tab[] = 
	{
		RELAY_AC_INPUT_1_GPIO_PIN,
		RELAY_AC_INPUT_2_GPIO_PIN,
		RELAY_AC_INPUT_3_GPIO_PIN,
		RELAY_AC_INPUT_4_GPIO_PIN,
		RELAY_AC_INPUT_5_GPIO_PIN,
		RELAY_AC_INPUT_6_GPIO_PIN,
		RELAY_AC_INPUT_7_GPIO_PIN,
		RELAY_AC_INPUT_8_GPIO_PIN
	};
#endif
#if 0
	uint16_t gpio_pin_tab[] = 
	{
		RELAY_AC_INPUT_3_GPIO_PIN,
		RELAY_AC_INPUT_6_GPIO_PIN,
		RELAY_AC_INPUT_2_GPIO_PIN,
		RELAY_AC_INPUT_8_GPIO_PIN,
		RELAY_AC_INPUT_7_GPIO_PIN,
		RELAY_AC_INPUT_5_GPIO_PIN,
		RELAY_AC_INPUT_1_GPIO_PIN,
		RELAY_AC_INPUT_4_GPIO_PIN
	};
#endif

	if( op == GPIO_ON )
	{
		GPIO_SET( RELAY_AC_INPUT_1_GPIO_PORT, gpio_pin_tab[ num ] );
	}
	else if( op == GPIO_OFF )
	{
		GPIO_RESET( RELAY_AC_INPUT_1_GPIO_PORT, gpio_pin_tab[ num ] );
	}
}

static void relay_dc_output( uint8_t num, uint8_t op )
{
	uint16_t gpio_pin_tab[] = 
	{
		RELAY_DC_OUTPUT_1_GPIO_PIN,
		RELAY_DC_OUTPUT_2_GPIO_PIN,
		RELAY_DC_OUTPUT_3_GPIO_PIN,
		RELAY_DC_OUTPUT_4_GPIO_PIN,
		RELAY_DC_OUTPUT_5_GPIO_PIN,
		RELAY_DC_OUTPUT_6_GPIO_PIN,
		RELAY_DC_OUTPUT_7_GPIO_PIN,
		RELAY_DC_OUTPUT_8_GPIO_PIN
	};

	if( op == GPIO_ON )
	{
		GPIO_SET( RELAY_DC_OUTPUT_1_GPIO_PORT, gpio_pin_tab[ num ] );
	}
	else if( op == GPIO_OFF )
	{
		GPIO_RESET( RELAY_DC_OUTPUT_1_GPIO_PORT, gpio_pin_tab[ num ] );
	}
}


void sInitGpio(void)
{
	init_gpio();
}

void gpio_close_ac_input( uint8_t num )
{	
	relay_ac_input( num, GPIO_OFF );
}

void gpio_open_ac_input( uint8_t num )
{
	relay_ac_input( num, GPIO_ON );
}

void gpio_open_dc_relay( uint8_t num )
{
	relay_dc_output( num, GPIO_ON);
}

void gpio_close_dc_relay( uint8_t num )
{
	relay_dc_output( num, GPIO_OFF);
}

void gpio_light_on_led_wifi_conn(void)
{
    GPIO_SET( LED_WIFI_CONN_GPIO_PORT, LED_WIFI_CONN_GPIO_PIN );
}

void gpio_light_off_led_wifi_conn(void)
{
    GPIO_RESET( LED_WIFI_CONN_GPIO_PORT, LED_WIFI_CONN_GPIO_PIN );
}


void gpio_set( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin )
{
    GPIO_SetBits( GPIOx, GPIO_Pin );
}

void gpio_reset( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin )
{
    GPIO_ResetBits( GPIOx, GPIO_Pin );
}

void gpio_toggle( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin )
{
    GPIO_TOGGLE( GPIOx, GPIO_Pin );
}



