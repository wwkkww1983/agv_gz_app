#ifndef  __KEY_H_
#define  __KEY_H_

#include "stm32f10x.h"

#define    KEY_HIGH   1
#define    KEY_LOW    0

// 定义按键默认默认状态是高还是低, 1-高，0-低
#define    KEY_DEFAULT_LEVEL    KEY_LOW    // 无用

#define    KEY_MASK   0x01    // 按键的掩码，禁止更改


#define    KEY_OFF         0x01   // 表示按键没有按下
#define    KEY_ON          0x02   // 表示按键按下
#define    KEY_ON_MID      0x04   // 表示按键中按
#define    KEY_ON_LONG     0x08   // 表示按键长按

// 定义key的处于哪种检测状态
//#define    KEY_CHK_PRESS    (uint8_t)0x00  // 默认状态，检测是否按下
//#define    KEY_CHK_RELEASE  (uint8_t)0x01  // 按下后，检测是否释放

#define    KEY_SCAN_INTERVAL      10     // 每隔10ms扫描一次
#define    KEY_ANTI_SHAKE_TIME    ( ( 50 ) / ( KEY_SCAN_INTERVAL ) )    // 消抖时间，100ms
#define    KEY_MID_TIME           ( ( 1000 ) / ( KEY_SCAN_INTERVAL ) )   // 中按时间定义为：1s
#define    KEY_LONG_TIME          ( ( 2000 ) / ( KEY_SCAN_INTERVAL ) )   // 长按时间定为为：2s

typedef enum _key_chk_step{
    KEY_CHK_PRESS,
    KEY_CHK_RELEASE
}KeyChkStepDef;

#pragma pack(1)
typedef struct _key_type_def
{
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;     
	uint16_t timers;       // 默认初始化为0
	uint8_t trigger;       // 默认初始化为0
	uint8_t pre;           // 保存按键上一次的状态，默认初始化为:0xff
	uint8_t default_level; // 按键的默认电平

	uint8_t cur;           // 保存当前按键状态
	enum _key_chk_step chk_step;      // 保存当前按键处于哪种检测状态
}KeyTypeDef;


uint8_t key_detect( KeyTypeDef *pkey );

#endif // key.h

