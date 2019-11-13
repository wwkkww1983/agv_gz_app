
#include "key.h"
#include "stdio.h"
#include "uart.h"

#ifndef __LOG
#define    __LOG_DEBUG    0
#endif
#ifndef __LOG_DEBUG
#define    __LOG_DEBUG      0
#endif

#define LOG_DEBUG(fmt,arg...)         do{\
                                          if(__LOG_DEBUG)\
											  printf("\r\n<<-DEBUG_DRIVER->> "fmt"\n",##arg);\
                                          }while(0)

static void do_key_scan( KeyTypeDef *pkey )
{
    uint8_t key_val;

	key_val = GPIO_ReadInputDataBit( pkey->GPIOx, pkey->GPIO_Pin );

#if 0
    // 如果按键默认状态是低电平，那么取个反
    if( KEY_DEFAULT_LEVEL == KEY_LOW )
	    key_val = ~ key_val;
#endif
    // 如果按键默认状态是低电平，那么取个反
    if( pkey->default_level == KEY_LOW )
	    key_val = ~ key_val;

	pkey->trigger = key_val & ( key_val ^ pkey->pre);
	pkey->pre = key_val;
}

uint8_t key_detect( KeyTypeDef *pkey )
{
	uint8_t key_press = KEY_OFF;

	do_key_scan( pkey );

	if( (pkey->pre & KEY_MASK) == 0 )
	{
        if( pkey->timers >  KEY_LONG_TIME )
		{
			pkey->timers = KEY_LONG_TIME;
			key_press = KEY_ON_LONG;
		    LOG_DEBUG("\r\nlong key");
		}
		else if( pkey->timers >  KEY_MID_TIME )
		{
			key_press = KEY_ON_MID;
		    LOG_DEBUG("\r\nmid key");
		}
		else if( pkey->timers >  KEY_ANTI_SHAKE_TIME )
		{
	        key_press = KEY_ON;
		    LOG_DEBUG("\r\nkey on");
		}
	}

	if( (~pkey->pre) & KEY_MASK )
	{
	    pkey->timers ++;
		LOG_DEBUG("\r\nkey timers:%d", pkey->timers);
	}

	if( pkey->trigger & KEY_MASK )
	{
		pkey->timers = 0;
	    key_press = KEY_OFF;
		LOG_DEBUG("\r\nkey off");
	}

	pkey->cur = key_press;
	if( key_press != KEY_OFF )
	{
		pkey->chk_step = KEY_CHK_RELEASE;
	}
	else
	{
		pkey->chk_step = KEY_CHK_PRESS;
	}

	return key_press;
}




