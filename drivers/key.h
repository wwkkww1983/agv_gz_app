#ifndef  __KEY_H_
#define  __KEY_H_

#include "stm32f10x.h"

#define    KEY_HIGH   1
#define    KEY_LOW    0

// ���尴��Ĭ��Ĭ��״̬�Ǹ߻��ǵ�, 1-�ߣ�0-��
#define    KEY_DEFAULT_LEVEL    KEY_LOW    // ����

#define    KEY_MASK   0x01    // ���������룬��ֹ����


#define    KEY_OFF         0x01   // ��ʾ����û�а���
#define    KEY_ON          0x02   // ��ʾ��������
#define    KEY_ON_MID      0x04   // ��ʾ�����а�
#define    KEY_ON_LONG     0x08   // ��ʾ��������

// ����key�Ĵ������ּ��״̬
//#define    KEY_CHK_PRESS    (uint8_t)0x00  // Ĭ��״̬������Ƿ���
//#define    KEY_CHK_RELEASE  (uint8_t)0x01  // ���º󣬼���Ƿ��ͷ�

#define    KEY_SCAN_INTERVAL      10     // ÿ��10msɨ��һ��
#define    KEY_ANTI_SHAKE_TIME    ( ( 50 ) / ( KEY_SCAN_INTERVAL ) )    // ����ʱ�䣬100ms
#define    KEY_MID_TIME           ( ( 1000 ) / ( KEY_SCAN_INTERVAL ) )   // �а�ʱ�䶨��Ϊ��1s
#define    KEY_LONG_TIME          ( ( 2000 ) / ( KEY_SCAN_INTERVAL ) )   // ����ʱ�䶨ΪΪ��2s

typedef enum _key_chk_step{
    KEY_CHK_PRESS,
    KEY_CHK_RELEASE
}KeyChkStepDef;

#pragma pack(1)
typedef struct _key_type_def
{
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;     
	uint16_t timers;       // Ĭ�ϳ�ʼ��Ϊ0
	uint8_t trigger;       // Ĭ�ϳ�ʼ��Ϊ0
	uint8_t pre;           // ���水����һ�ε�״̬��Ĭ�ϳ�ʼ��Ϊ:0xff
	uint8_t default_level; // ������Ĭ�ϵ�ƽ

	uint8_t cur;           // ���浱ǰ����״̬
	enum _key_chk_step chk_step;      // ���浱ǰ�����������ּ��״̬
}KeyTypeDef;


uint8_t key_detect( KeyTypeDef *pkey );

#endif // key.h

