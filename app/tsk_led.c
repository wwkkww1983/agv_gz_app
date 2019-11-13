#include "applications.h"
#include "gpio.h"
#include "service.h"


TaskHandle_t h_led_entry = NULL;
void led_entry(void *pvParameters);


#define    LED_NUM    8

struct gpio_tab {
    GPIO_TypeDef *port;
	uint16_t pin;
}GpioTabDef;

enum led_operation {
    LED_ON,
	LED_OFF,
	LED_TOGGLE
}LedOpDef;

enum led_pin
{
    PIN_1 = 0,
    PIN_2,
    PIN_3,
    PIN_4,
    PIN_5,
    PIN_6,
    PIN_7,
    PIN_8,
	PIN_ALL
}LedPinDef;

enum led_type
{
    AC_RELAY,
	DC_RELAY,
	ON_OFF,
	CHMOD,
	FAN_1_SPEED,
	FAN_2_SPEED
}LedTypeDef;

static void ac_relay_led_operation(enum led_operation op, enum led_pin pin);
static void dc_relay_led_operation(enum led_operation op, enum led_pin pin);
static void on_off_led_operation(enum led_operation op, enum led_pin pin);
static void chmod_led_operation(enum led_operation op, enum led_pin pin);
static void fan1_speed_ctl_led_operation(enum led_operation op, enum led_pin pin);
static void fan2_speed_ctl_led_operation(enum led_operation op, enum led_pin pin);

#define    ULONG_MAX    0xFFFFFFFF

void led_entry(void *pvParameters)
{
const TickType_t xTicksToWait = pdMS_TO_TICKS( 1000 );
BaseType_t xResult = pdFALSE;
uint32_t ulNotifiedValue;

	uint32_t i = 0;
	uint8_t reg_ac_relay = 0; // 0-toggle, 1-close
	uint8_t reg_chmod = 0; // 0-toggle, 1-close
	uint8_t reg_fan_fb = 0;
	uint32_t mask = 1;

	while(1)
	{
        xResult = xTaskNotifyWait( (uint32_t)   pdFALSE, /* Don't clear bits on entry. */
                                   (uint32_t)   ULONG_MAX, /* Clear all bits on exit. */
                                   (uint32_t *) &ulNotifiedValue, /* Stores the notified value. */
                                   (TickType_t) xTicksToWait );
		if( xResult == pdPASS )	
		{
			for( i=0; i<KEY_NUM; i++ )
			{
				mask = 1 << i;
				if( i<8 )
				{
				    if( ulNotifiedValue & mask )
				    {
				        toggle_bit( reg_ac_relay, i );
				    }
				}
				else if( i<16 )
				{
				    if( ulNotifiedValue & mask )
				    {
				        toggle_bit( reg_chmod, (i-8) );
				    }
				}
				else
				{
				    if( ulNotifiedValue & mask )
				    {
				        toggle_bit( reg_fan_fb, (i-16) );
				    }
				}
			}
		}
		else
		{
		    on_off_led_operation( LED_TOGGLE, PIN_ALL );
		    dc_relay_led_operation( LED_TOGGLE, PIN_ALL );

		    for( i=0; i<LED_NUM; i++ )
		    {
		    	mask = 1 << i;
		    	if( reg_ac_relay & mask )
		            ac_relay_led_operation( LED_OFF, i );
		    	else
		            ac_relay_led_operation( LED_TOGGLE, i );

		    	if( reg_chmod & mask )
		            chmod_led_operation( LED_OFF, i );
		    	else
		            chmod_led_operation( LED_TOGGLE, i );
		    }

			if( reg_fan_fb & 0x01 )
		        fan1_speed_ctl_led_operation( LED_OFF, PIN_ALL );
			else
		        fan1_speed_ctl_led_operation( LED_TOGGLE, PIN_ALL );

			if( reg_fan_fb & 0x02 )
		        fan2_speed_ctl_led_operation( LED_OFF, PIN_ALL );
			else
		        fan2_speed_ctl_led_operation( LED_TOGGLE, PIN_ALL );
		}
	}
}


static void led_op( void (*fun_op)(GPIO_TypeDef *port, uint16_t pin), struct gpio_tab *tab, enum led_pin pin )
{
	uint32_t i;
   	if( pin == PIN_ALL )
   	{
        for( i = 0; i < LED_NUM; i++ )
            fun_op(tab[i].port, tab[i].pin);
   	}
   	else
            fun_op(tab[pin].port, tab[pin].pin);
}

static void on_off_led_operation(enum led_operation op, enum led_pin pin)
{
	struct gpio_tab tab[] = {
	{ON_OFF_1_GPIO_PORT, ON_OFF_1_GPIO_PIN},
	{ON_OFF_2_GPIO_PORT, ON_OFF_2_GPIO_PIN},
	{ON_OFF_3_GPIO_PORT, ON_OFF_3_GPIO_PIN},
	{ON_OFF_4_GPIO_PORT, ON_OFF_4_GPIO_PIN},
	{ON_OFF_5_GPIO_PORT, ON_OFF_5_GPIO_PIN},
	{ON_OFF_6_GPIO_PORT, ON_OFF_6_GPIO_PIN},
	{ON_OFF_7_GPIO_PORT, ON_OFF_7_GPIO_PIN},
	{ON_OFF_8_GPIO_PORT, ON_OFF_8_GPIO_PIN}
	};

	switch( op )
	{
		case LED_ON:
	        LOG_INFO_APP("on/off_led_on");
			led_op( gpio_set, tab, pin );
			break;
		case LED_OFF:
	        LOG_INFO_APP("on/off_led_off");
			led_op( gpio_reset, tab, pin );
			break;
			break;
		case LED_TOGGLE:
	        LOG_INFO_APP("on/off_led_toggle");
			led_op( gpio_toggle, tab, pin );
			break;
		default:
			break;
	}
}

static void dc_relay_led_operation(enum led_operation op, enum led_pin pin)
{
	struct gpio_tab tab[] = {
	{RELAY_DC_OUTPUT_1_GPIO_PORT, RELAY_DC_OUTPUT_1_GPIO_PIN},
	{RELAY_DC_OUTPUT_2_GPIO_PORT, RELAY_DC_OUTPUT_2_GPIO_PIN},
	{RELAY_DC_OUTPUT_3_GPIO_PORT, RELAY_DC_OUTPUT_3_GPIO_PIN},
	{RELAY_DC_OUTPUT_4_GPIO_PORT, RELAY_DC_OUTPUT_4_GPIO_PIN},
	{RELAY_DC_OUTPUT_5_GPIO_PORT, RELAY_DC_OUTPUT_5_GPIO_PIN},
	{RELAY_DC_OUTPUT_6_GPIO_PORT, RELAY_DC_OUTPUT_6_GPIO_PIN},
	{RELAY_DC_OUTPUT_7_GPIO_PORT, RELAY_DC_OUTPUT_7_GPIO_PIN},
	{RELAY_DC_OUTPUT_8_GPIO_PORT, RELAY_DC_OUTPUT_8_GPIO_PIN}
	};

	switch( op )
	{
		case LED_ON:
			LOG_INFO_APP("dc_relay_led_on");
			led_op( gpio_set, tab, pin );
			break;
		case LED_OFF:
			LOG_INFO_APP("dc_relay_led_off");
			led_op( gpio_reset, tab, pin );
			break;
		case LED_TOGGLE:
			LOG_INFO_APP("dc_relay_led_toggle");
			led_op( gpio_toggle, tab, pin );
			break;
		default:
			LOG_INFO_APP("dc_relay_led none");
			break;
	}
}

static void ac_relay_led_operation(enum led_operation op, enum led_pin pin)
{
	struct gpio_tab tab[] = {
	{RELAY_AC_INPUT_1_GPIO_PORT, RELAY_AC_INPUT_1_GPIO_PIN},
	{RELAY_AC_INPUT_2_GPIO_PORT, RELAY_AC_INPUT_2_GPIO_PIN},
	{RELAY_AC_INPUT_3_GPIO_PORT, RELAY_AC_INPUT_3_GPIO_PIN},
	{RELAY_AC_INPUT_4_GPIO_PORT, RELAY_AC_INPUT_4_GPIO_PIN},
	{RELAY_AC_INPUT_5_GPIO_PORT, RELAY_AC_INPUT_5_GPIO_PIN},
	{RELAY_AC_INPUT_6_GPIO_PORT, RELAY_AC_INPUT_6_GPIO_PIN},
	{RELAY_AC_INPUT_7_GPIO_PORT, RELAY_AC_INPUT_7_GPIO_PIN},
	{RELAY_AC_INPUT_8_GPIO_PORT, RELAY_AC_INPUT_8_GPIO_PIN}
	};

	switch( op )
	{
		case LED_ON:
			LOG_INFO_APP("ac_relay_led_on");
			led_op( gpio_set, tab, pin );
			break;
		case LED_OFF:
			LOG_INFO_APP("ac_relay_led_off");
			led_op( gpio_reset, tab, pin );
			break;
		case LED_TOGGLE:
			LOG_INFO_APP("ac_relay_led_toggle");
			led_op( gpio_toggle, tab, pin );
			break;
		default:
			LOG_INFO_APP("ac_relay_led none");
			break;
	}
}

static void chmod_led_operation(enum led_operation op, enum led_pin pin)
{
	struct gpio_tab tab[] = {
	{RELAY_CHMOD_D1_GPIO_PORT, RELAY_CHMOD_D1_GPIO_PIN},
	{RELAY_CHMOD_D2_GPIO_PORT, RELAY_CHMOD_D2_GPIO_PIN},
	{RELAY_CHMOD_D3_GPIO_PORT, RELAY_CHMOD_D3_GPIO_PIN},
	{RELAY_CHMOD_D4_GPIO_PORT, RELAY_CHMOD_D4_GPIO_PIN},
	{RELAY_CHMOD_D5_GPIO_PORT, RELAY_CHMOD_D5_GPIO_PIN},
	{RELAY_CHMOD_D6_GPIO_PORT, RELAY_CHMOD_D6_GPIO_PIN},
	{RELAY_CHMOD_D7_GPIO_PORT, RELAY_CHMOD_D7_GPIO_PIN},
	{RELAY_CHMOD_D8_GPIO_PORT, RELAY_CHMOD_D8_GPIO_PIN}
	};

	switch( op )
	{
		case LED_ON:
			LOG_INFO_APP("chmod_led_on");
			led_op( gpio_set, tab, pin );
			break;
		case LED_OFF:
			LOG_INFO_APP("chmod_led_off");
			led_op( gpio_reset, tab, pin );
			break;
		case LED_TOGGLE:
			LOG_INFO_APP("chmod_led_toggle");
			led_op( gpio_toggle, tab, pin );
			break;
		default:
			LOG_INFO_APP("ac_relay_led none");
			break;
	}
}

static void fan1_speed_ctl_led_operation(enum led_operation op, enum led_pin pin)
{
	struct gpio_tab tab[] = { FAN_1_SPEED_CTL_GPIO_PORT, FAN_1_SPEED_CTL_GPIO_PIN };

	switch( op )
	{
		case LED_ON:
			LOG_INFO_APP("fan1 speed ctl led_on");
	        GPIO_SET(tab[0].port, tab[0].pin);
			break;
		case LED_OFF:
			LOG_INFO_APP("fan1 speed ctl led_off");
            GPIO_RESET(tab[0].port, tab[0].pin);
			break;
		case LED_TOGGLE:
			LOG_INFO_APP("fan1 speed ctl led_toggle");
	        GPIO_TOGGLE(tab[0].port, tab[0].pin);
			break;
		default:
			LOG_INFO_APP("fan1 speed ctl led none");
			break;
	}
}

static void fan2_speed_ctl_led_operation(enum led_operation op, enum led_pin pin)
{
	struct gpio_tab tab[] = { FAN_2_SPEED_CTL_GPIO_PORT, FAN_2_SPEED_CTL_GPIO_PIN };

	switch( op )
	{
		case LED_ON:
			LOG_INFO_APP("fan2 speed ctl led_on");
	        GPIO_SET(tab[0].port, tab[0].pin);
			break;
		case LED_OFF:
			LOG_INFO_APP("fan2 speed ctl led_off");
            GPIO_RESET(tab[0].port, tab[0].pin);
			break;
		case LED_TOGGLE:
			LOG_INFO_APP("fan2 speed ctl led_toggle");
	        GPIO_TOGGLE(tab[0].port, tab[0].pin);
			break;
		default:
			LOG_INFO_APP("fan2 speed ctl led none");
			break;
	}
}

