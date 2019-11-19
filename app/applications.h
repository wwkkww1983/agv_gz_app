#ifndef __APPLICATIONS_H
#define __APPLICATIONS_H

/************************************************************************
note.

*************************************************************************/


#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "board.h"
#include "uart.h"
//#include "service.h"
#include "bsp_pmbus_stack.h"

typedef    uint8_t    BOOL;
#define    TRUE       1u
#define    FALSE      0u

#define    ON         1u
#define    OFF        0u

#define    bit( n )                ( 1 << ( n ) )
#define    set_bit( value, n )     ( (value) |= (bit(n)) )
#define    reset_bit( value, n )   ( (value) &= (~bit(n)) )
#define    toggle_bit( value, n )  ( (value) ^= (bit(n)))

#define    VER_HMI    "01.11.01"
#define    VER_SW    "gz_01.11.13"

#define    LED_STACK    256
#define    LED_PRIO     1
extern TaskHandle_t h_led_entry;
void led_entry(void *pvParameters);

#define    KEY_NUM      18
#define    KEY_STACK    256
#define    KEY_PRIO     2
extern TaskHandle_t h_key_entry;
void key_entry(void *pvParameters);



#define    MS_BASE    10280ul    // ��Լ1ms
#define    MS(n)      ((MS_BASE)*(n))

#define    MODE_PMBUS 0x01u   // ͨ��pmbus���Ƴ�磬��Ӧ�Զ�����
#define    MODE_CURVE 0x00u   // ��ģ�鱾��ĳ�����߿��Ƴ�磬��Ӧ�ֶ�����
#define    MODE_MANUAL MODE_CURVE
#define    MODE_AUTO   MODE_PMBUS

#define    OP_DISP_START   0x01u
#define    OP_DISP_STOP    0x00u
#define    OP_DISP_NONE    0x02u


// ģʽ�л�
#define    BASIC_MODE_NONE_SWITCH             0x00
#define    BASIC_MODE_AUTO_TO_MANUAL          0X01
#define    BASIC_MODE_MANUAL_TO_AUTO          0X02
#define    BASIC_MODE_THREE_SECTION_TO_PRE    0X03
#define    BASIC_MODE_PRE_TO_THREE_SECTION    0X04

// ���ͼ�����
#define    BASIC_BAT_DISCONN    0x00u
#define    BASIC_BAT_CONN       0x01u
#define    BASIC_BAT_REVERSE    0x02u
#define    BASIC_BAT_CHGR_ING   0x04u
#define    BASIC_BAT_FULL       0x08u

#define    LED_GREEN      0x00u
#define    LED_RED        0x01u

// ϵͳ״̬�����
#define    BASIC_ERR_NO   0x00u
#define    ERR_BAT_REVERSE  1u    // �����ִ������λ
#define    ERR_BAT_TYPE     2u
#define    ERR_DC_OK        3u
#define    ERR_T_ALARM      4u
#define    ERR_OVER_OUTPUT  5u
#define    ERR_BAT_CHGR_MOS_CLOSE    6u

#define    BASIC_WARN_NO     0x00u
#define    WARN_ILLEGAL_STOP 1u

#define    BASIC_CHGR_OFF      0x00u
#define    BASIC_CHGR_STARTING 0x01u
#define    BASIC_CHGR_ING      0x02u
#define    BASIC_CHGR_FINISH   0x03u
#define    BASIC_CHGR_STOP_BY_HOST    0x04u  // ����λ���ֶ�ֹͣ

#define    BASIC_RESTART_NO    0x00u
#define    BASIC_RESTART_YES   0x01u

// ģ���Ƿ񿪻�
#define    BASIC_POWER_ON    0x00u
#define    BASIC_POWER_OFF   0x01u

// touch_screen_entry������ֵ֪ͨ
#define    TX_BIT                                 (uint32_t)(bit(0))
#define    RX_BIT                                 (uint32_t)(bit(1))
#define    TX_IDLE_BIT                            (uint32_t)(bit(2))
#define    RX_IDLE_BIT                            (uint32_t)(bit(3))

#define    CHGR_NOTIFY_TO_START                   (uint32_t)(bit(5))
#define    CHGR_NOTIFY_START_SUCCESS              (uint32_t)(bit(6))
#define    CHGR_NOTIFY_START_FAILED               (uint32_t)(bit(7))
#define    CHGR_NOTIFY_TO_STOP                    (uint32_t)(bit(8))
#define    CHGR_NOTIFY_STOP_SUCCESS               (uint32_t)(bit(9))
#define    CHGR_NOTIFY_NONE                       (uint32_t)(bit(10))
#define    CHGR_NOTIFY_EMERGENCY_STOP             (uint32_t)(bit(11))
#define    CHGR_NOTIFY_START_FROM_EMERGENCY_STOP  (uint32_t)(bit(12))
#define    SET_NOTIFY_TO_RESTART                  (uint32_t)(bit(13))
#define    SET_NOTIFY_TO_RESTART_ALL              (uint32_t)(bit(14))
#define    SET_NOTIFY_RESTART_FINISHED            (uint32_t)(bit(15))
#define    CHGR_NOTIFY_TYPE_SWITCH                (uint32_t)(bit(16))
#define    CHGR_NOTIFY_TYPE_SWITCH_FINISH         (uint32_t)(bit(17))
#define    CHGR_NOTIFY_NO_START                   (uint32_t)(bit(18))
#define    CHGR_NOTIFY_NO_SYN_SET                 (uint32_t)(bit(19))
#define    CHGR_NOTIFY_NO_SET                     (uint32_t)(bit(20))



// ������ʽ
#define    START_TYPE_HMI        0x00u
#define    START_TYPE_BOOKING    0x01u
#define    START_TYPE_BMS        0x02u

// ֹͣ��ʽ
#define    STOP_TYPE_HMI        0x00u
#define    STOP_TYPE_BOOKING    0x01u
#define    STOP_TYPE_BMS        0x02u
#define    STOP_TYPE_HOST       0x03u

//
#define    CHGR_OP_TYPE_HMI       (uint8_t)0x00
#define    CHGR_OP_TYPE_NO_HMI    (uint8_t)0x01


// 
#define    CHGR_STEP_IDLE                                   0u
#define    CHGR_STEP_CHK_BAT_REVERSE                        (uint16_t)(bit(0))
#define    CHGR_STEP_CHK_BAT_CONN                           (uint16_t)(bit(1))
#define    CHGR_STEP_CHK_BAT_TYPE                           (uint16_t)(bit(2))
#define    CHGR_STEP_OPEN_OUTPUT                            (uint16_t)(bit(3))
#define    CHGR_STEP_CCM                                    (uint16_t)(bit(4))
#define    CHGR_STEP_CVM                                    (uint16_t)(bit(5))
#define    CHGR_STEP_FVM                                    (uint16_t)(bit(6))
#define    CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING             (uint16_t)(bit(7))
#define    CHGR_STEP_CHK_BAT_DISCONN_IN_FLOATING            (uint16_t)(bit(8))
#define    CHGR_STEP_PRE                                    (uint16_t)(bit(9))
#define    CHGR_STEP_CHECK_OUTPUT                           (uint16_t)(bit(10))
#define    CHGR_STEP_CHECK_OUTPUT_VOUT                      (uint16_t)(bit(11))
#define    CHGR_STEP_CHECK_OUTPUT_IOUT                      (uint16_t)(bit(12))
#define    CHGR_STEP_CHECK_STAGE                            (uint16_t)(bit(13))
#define    CHGR_STEP_RESTART                                (uint16_t)(bit(14))




//
#define    CHGR_TYPE_THREE_SECTION           0x01
#define    CHGR_TYPE_PRE                     0x02
#define    CHGR_TYPE_AUTO                    0x03

//
#define    CHGR_BAT_TYPE_LI                  0x01
#define    CHGR_BAT_TYPE_LEAD_ACID           0x02

//

#define    POWER_UNIT_1    0x00
#define    POWER_UNIT_2    0x01
#define    POWER_UNIT_3    0x02
#define    POWER_UNIT_4    0x03
#define    POWER_UNIT_5    0x04
#define    POWER_UNIT_6    0x05
#define    POWER_UNIT_7    0x06
#define    POWER_UNIT_8    0x07
#define    POWER_UNIT_SY   0x08 // ��ʾ������ͬ��ģʽ


// max in bsp/applications/applications.h, at  #define CHGR_CNT_BOX_SIZE    30
#define    IDX_CNT_OPEN_OUTPUT_FAIL    0
#define    IDX_CNT_OPEN_OUTPUT_SUS     1
#define    IDX_CNT_CCM                 2
#define    IDX_CNT_CVM                 3
#define    IDX_CNT_FVM                 4
#define    IDX_CNT_PMBUS_COMM          5
#define    IDX_CNT_BAT_DISCONN         6
#define    IDX_CNT_CHK_BAT_REVERSE     7
#define    IDX_CNT_CHK_BAT_CONN        8
#define    IDX_CNT_CHK_BAT_TYPE        9
#define    IDX_CNT_CHK_T_ALARM         10
#define    IDX_CNT_CHK_DC_OK           11
#define    IDX_CNT_CHGR_TIME           12
#define    IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING     13
#define    IDX_CNT_CHK_OVER_OUTPUT                14
#define    IDX_CNT_BMS_TIMEOUT                    15
#define    IDX_CNT_DELAY_CLOSE_AC                 16
#define    IDX_CNT_DELAY_MODE_SWITCH              17
#define    IDX_CNT_BOOKING_TIME                   18
#define    IDX_CNT_TIMING_TIME                    19
#define    IDX_CNT_STOP_TIME                      20
#define    IDX_CNT_IS_PRE_CHGR                    21
#define    IDX_CNT_START_FAIL_TIME                22
#define    IDX_CNT_ON_OFF_OPEN_TIME               23
#define    IDX_CNT_AVE_IOUT                       24    // ����ƽ������ֵ
#define    IDX_CNT_OPEN_OUTPUT                    25
#define    IDX_CNT_CHK_VOUT_DISAPPEAR_IN_CHGRING  26
#define    IDX_CNT_FILTER_CLOSE_AC                27
#define    IDX_CNT_DELAY_BMS_START_QUERY          28


#define    START_UNIT_1    (uint32_t)(bit(0))
#define    START_UNIT_2    (uint32_t)(bit(1))
#define    START_UNIT_3    (uint32_t)(bit(2))
#define    START_UNIT_4    (uint32_t)(bit(3))
#define    START_UNIT_5    (uint32_t)(bit(4))
#define    START_UNIT_6    (uint32_t)(bit(5))
#define    START_UNIT_7    (uint32_t)(bit(6))
#define    START_UNIT_8    (uint32_t)(bit(7))

#define    STOP_UNIT_1     (uint32_t)(bit(8))
#define    STOP_UNIT_2     (uint32_t)(bit(9))
#define    STOP_UNIT_3     (uint32_t)(bit(10))
#define    STOP_UNIT_4     (uint32_t)(bit(11))
#define    STOP_UNIT_5     (uint32_t)(bit(12))
#define    STOP_UNIT_6     (uint32_t)(bit(13))
#define    STOP_UNIT_7     (uint32_t)(bit(14))
#define    STOP_UNIT_8     (uint32_t)(bit(15))



/*
 * RCB1600����س��״̬��Ϣ���壬����RCB-1600�ֲ�
 */
// Low byte
// Bit 0 FULLM������ģʽ��B��0��δ���늣�1������
#define			FULLM						0x0001
// Bit 1 CCM����������ģʽ��B��0���������̎춶����ģʽ��1�������̎춶����ģʽ
#define			CCM							0x0002
// Bit 2 CVM����늉����ģʽ��B��0���������̎춶�늉�ģʽ��1�������̎춶�늉�ģʽ
#define			CVM							0x0004
// Bit 3 FVM������ģʽ��B��0���������̎춸���ģʽ��1�������̎춸���ģʽ
#define			FVM							0x0008

// High byte
// Bit 0 EEPER: EEPROM��늅����e�`��0����늅����Y�����_��1����늅����Y���e�`
#define			EEPER						0x0100
// Bit 2 NTCER: �ض��a����·��0���ض��a����·�o�l����·��1���ض��a����·�l����·
#define			NTCER						0x0400
// Bit 3 BTNC: 늳�δ�ӣ�0���ɜy��늳أ�1��δ�ɜy��늳�
#define			BTNC						0x0800
// Bit 5 CCTOF��������A�γ�늳��r��ˣ�0��������A�γ��δ���r��1��������A�γ�늳��r
#define			CCTOF						0x2000
// Bit 6 CVTOF����늉��A�γ�늳��r��ˣ�0����늉��A�γ��δ���r��1����늉��A�γ�늳��r
#define			CVTOF						0x4000
// Bit 7 FTTOF�������A�γ�늳��r��ˣ�0�������A�γ��δ���r��1�������A�γ�늳��r
#define			FVTOF						0x8000





#define    POWER_UNIT_NUM       8
#define    CHGR_CNT_BOX_SIZE    30

#define    PMBUS_COMM_MS_DELAY  15 // 15ms
#define    EE_MS_DELAY    5

#define    POWER_ON_MS_DELAY    3000
#define    POWER_OFF_MS_DELAY    8000

#define    TIMEOUT_TOUCH_SCREEN    120 //120s


#define    CHGR_TIMING_TIME_DEFAULT    30
#define    CHGR_BOOKING_TIME_DEFAULT    30



#define    BAT_REVERSE_VOL    0

#define    BAT_CONN_VOL_MIN       20
#define    BAT_CONN_VOL_MAX       60

#define    BAT_TYPE_48    0
#define    BAT_TYPE_24    1

#define    CHK_VOL_MIN_BAT_48    40
#define    CHK_VOL_MAX_BAT_48    60

#define    CHK_VOL_MIN_BAT_24    20
#define    CHK_VOL_MAX_BAT_24    30



// �������
#define    TOUCH_SCREEN_STACK    512
#define    TOUCH_SCREEN_PRIO     5
extern TaskHandle_t h_touch_screen_entry;
extern TaskHandle_t xTouchScreenDamonHandle; // ��Ļˢ������ľ��
void touch_screen_entry( void *pvParameter );


//#define   CHGR_STACK    256
//#define   CHGR_PRIO     5
//extern TaskHandle_t h_charge_entry;
//void charge_entry( void * pvParameters );

// ����BMSͨ�ţ�����������Ҫ������������
#define   EXT_BMS_COMM_STACK    256
#define   EXT_BMS_COMM_PRIO     5
extern TaskHandle_t h_ext_bms_comm_entry; 
//void ext_interface_entry( void * pvParameters );
void 	sExt_bms_comm_Task(void);

// ����BMSͨ�Ž��յ�����
#define   EXT_BMS_COMM_RX_STACK    356
#define   EXT_BMS_COMM_RX_PRIO     5
extern TaskHandle_t h_ext_bms_comm_rx_entry; 
//void ext_interface_entry( void * pvParameters );
void 	sExt_bms_comm_rx_Task(void);


#define   EXT_WIFI_STACK    1024
#define   EXT_WIFI__PRIO     3
extern TaskHandle_t h_ext_wifi_entry; 
void   ext_wifi_entry( void * pvParameters );

#ifdef USE_CAN_SYNC // ʹ��˫��ͨ��, 20191101���
#define   CAN_COMM_STACK    512
#define   CAN_COMM_PRIO     3
extern TaskHandle_t h_can_comm_entry; 
void   can_comm_entry( void * pvParameters );
#endif

#ifdef USE_WDG
#define    WDG_STACK    256
#define    WDG_PRIO     8    // ���ȼ���Ҫ����Ϊ���
extern TaskHandle_t h_wdg_daemon;
void wdg_daemon_task(void *pvParameters);

// ���Ź����
// ��Ӧ���������������������µ��¼���ֻ�е����е����񶼷����¼��󣬲Ż�ִ��ι������
#define    WDG_BIT_TOUCH_SCREEN           bit(0)
#define    WDG_BIT_TOUCH_SCREEN_DAEMON    bit(1)
#define    WDG_BIT_SETTING         bit(2)
#define    WDG_BIT_BMS_COMM        bit(3)
#define    WDG_BIT_BMS_COMM_RX     bit(4)
#define    WDG_BIT_WIFI            bit(5)
#define    WDG_BIT_EVENT_WRITE     bit(6)
#define    WDG_BIT_CHGR_TIME_CTRL  bit(7)
#define    WDG_BIT_CHGR_DAEMON     bit(8)
#define    WDG_BIT_SMP_VOLT        bit(9)
#define    WDG_BIT_TASK_ALL ( WDG_BIT_TOUCH_SCREEN | WDG_BIT_TOUCH_SCREEN_DAEMON | WDG_BIT_BMS_COMM | WDG_BIT_BMS_COMM_RX |\
				              WDG_BIT_SMP_VOLT | WDG_BIT_EVENT_WRITE  | WDG_BIT_CHGR_TIME_CTRL | WDG_BIT_CHGR_DAEMON | WDG_BIT_SETTING | WDG_BIT_WIFI ) 
#endif

extern QueueHandle_t xQueuePrint; // ���ӡ���������ݵ���Ϣ����


// �����ʱ��ض���
#define    HMI_SW_TIMER_NUM    2
#define    ID_SW_TIMER_0       0    // ���ó�ʱ��ʱ��
#define    ID_SW_TIMER_1       1    // ������ʱ��ʱ��

#define    HMI_SW_TIMER_0_PERIOD    20000 // ���ó�ʱ��20s
#define    HMI_SW_TIMER_1_PERIOD    60000 // ������ʱ��60s


#define    SW_TO_RIGHT    0x00
#define    SW_TO_LEFT     0x01

#define    EVENT_NULL        (uint32_t)0

#define    EVENT_MAX_NUM    9        // �¼���
#define    EVENT_MAX_STORE_NUM    40 // ����ģ�����洢���¼�����
#define    EVENT_EACH_PAGE_NUM    4    // ÿҳ��ʾ4��
#define    EVENT_MAX_PAGE_NUM    ( ( EVENT_MAX_STORE_NUM ) / ( EVENT_EACH_PAGE_NUM ) )

#define    EVENT_START        0u        // �¼� - ����
#define    EVENT_STOP         1u        // �¼� - ֹͣ
#define    EVENT_BAT_REVERSE  2u        // �¼� - ��ط���
#define    EVENT_BAT_TYPE_ERR 3u        // �¼� - ������ʹ���
#define    EVENT_DC_OK_ERR    4u        // �¼� - ����쳣
#define    EVENT_T_ALARM      5u        // �¼� - �¶��쳣
#define    EVENT_BAT_DISCONN  6u        // �¼� - ���δ����
#define    EVENT_OVER_OUTPUT  7u        // �¼� - ��ѹ
#define    EVENT_ILLEGAL_STOP 8u        // �¼� - �Ƿ�ֹͣ


/*
 *  eeporm�д洢���ַ�Χ
 *  base - 166
 *           | is_full(2Byte) | latest_idx(2Byte*power_uni_num) | store_base |
 *
 *  latest_idx -> ��ʾ����д����������
 */
#define    EE_EVENT_IS_FULL_LEN    ( 2 )
#define    EE_EVENT_LATEST_IDX_LEN ( 2 )

#define    EE_EVENT_BASE_ADDR       ( 900 )
#define    EE_EVENT_IS_FULL_ADDR    ( EE_EVENT_BASE_ADDR )
#define    EE_EVENT_LATEST_IDX_BASE_ADDR ( EE_EVENT_IS_FULL_ADDR + EE_EVENT_IS_FULL_LEN )
#define    EE_EVENT_STORE_BASE_ADDR      ( EE_EVENT_LATEST_IDX_BASE_ADDR + ( EE_EVENT_LATEST_IDX_LEN * POWER_UNIT_NUM ) )


// ��Ʒ���������������
#define  EE_SERIAL_NUM_ADDR   550
#define  EE_SERIAL_NUM_LEN    16 
#define  EE_PRODUCT_CODE_ADDR 580
#define  EE_PRODUCT_CODE_LEN  11



typedef struct _sys_time_def
{
	uint8_t year;    // ��
	uint8_t month;    // ��
	uint8_t day;      // ��
	uint8_t hour;     // ʱ
	uint8_t min;      // ��
	uint8_t sec;      // ��
}Sys_TimeDef;


enum _pu_chgr_op
{
    op_stop = 0,  //��ʾִ��ֹͣ��������ʾ��������
	op_start,      //��ʾִ��������������ʾ��ֹͣ��
};
typedef enum _pu_chgr_op CHGR_OPTypeDef;


#define    CHGR_PARA_CURVE_1    0
#define    CHGR_PARA_CURVE_2    1
#define    CHGR_PARA_CURVE_3    2
#define    CHGR_PARA_CURVE_4    3
#define    CHGR_PARA_CURVE_U    4

#define     CHGR_PARA_DATA_MAX   0
#define     CHGR_PARA_DATA_MIN   1

typedef struct _pu_chgr_curve_para
{
    float ichg;    // �������
	float itaper;  // �������
	float vbst;    // ��ֹ��ѹ
	float vfloat;  // �����ѹ
}CHGR_ParaDataDef;

typedef struct _chgr_pre_para_data_def
{
	float pre_iout;    // Ԥ�����
	float pre_vout;    // Ԥ���ѹ
	float ichg;        // ������
	float vbst;        // ����ѹ
	float itaper;      // ��ֹ����
}PreChgrParaDataDef;

typedef struct _chgr_para_data_auto_def
{
    uint16_t threshold;    // �Զ�������ֵ�������ٷֱ�>=��ֵʱ����Ϊ������ֹͣ���
}AutoChgrParaDataDef;

typedef struct _pu_chgr_para
{
	uint16_t ee_addr;       // ��������eeprom�б������ʼ��ַ
    uint8_t curve;          // ��ǰ������
    CHGR_ParaDataDef data;  // ��ϸ������
    PreChgrParaDataDef data_pre; // Ԥ��������
    AutoChgrParaDataDef data_auto;  // Ԥ������
}ChgrParaDef;

typedef struct _sys_ee_sys_event_def
{
	uint16_t code;   // ����, ��8λ��ʾ�¼����룬�Ͱ�λ�ĸ���λ����ģʽ������λ�������
	Sys_TimeDef sys_time; // ϵͳʱ��
}EE_SysEventDef;

typedef struct _sys_event_cache_def
{
    uint32_t code;
    Sys_TimeDef time[ EVENT_MAX_NUM ];
}PU_EventCacheDef;


#define    IOUT_AVE_QUEUE_SIZE    5
typedef struct iout_queue
{
	float buff[ IOUT_AVE_QUEUE_SIZE ];
	uint16_t head;
//	uint16_t tail;
	uint8_t full_flag;
}Queue_Float;

#define    SIZE_BMS_VER    21
#define    SIZE_SOH_INFO   9
#define    SIZE_BAT_ID_NUM   18

#define    TIMEOUT_START_BMS_QUERY    12

enum BAT_MATERIAL
{
	TERPOLYMER = 0,               // ��Ԫ���
	LITHIUM_FER_PHOSPHATE,        // ��﮵��
	LITHIUM_TITANATE,             // ����﮵��
	ERROR_TYPE                    // δ֪����
};

typedef struct _bat_info_from_bms
{
	uint16_t volt;     // ��صĵ�ѹ���ֱ���0.01V
	uint16_t curr_in;  // ��صĳ��������ֱ���0.1A
	uint16_t temp;     // ��ص��¶�2���ֱ���0.1
	uint16_t cap;      // ��ص�ʣ��������0.1%
	uint16_t limit_curr;  // 500��ʾ���׮�������50%�����������30����
	uint16_t volt_single_14;  // ����14�ĵ�ѹ���ֱ���1mV
	uint16_t volt_single_16;
	uint16_t soh_val;
	uint16_t full_volt;     // �����ѹ
	uint16_t single_bat_volt_len;
	uint8_t flag_full_volt; // �Ƿ�ȷ���������ѹ, TRUE-ȷ���ˣ�FALSE-û��ȷ��
	uint8_t flag_full;      // 1-full, 0-not full
	uint8_t warn_1;
	uint8_t warn_2;
	uint8_t warn_3;
	uint8_t warn_4;
	char bms_ver[ SIZE_BMS_VER ];
	char soh_info[ SIZE_SOH_INFO ];
	char id_num[ SIZE_BAT_ID_NUM ];
	enum BAT_MATERIAL material;
}BatInfoDefFromBMS;

typedef struct _power_unit_info_def
{
	struct
	{
		uint8_t bat;       // ����Ƿ�����
		uint8_t chgr;      // �Ƿ��ڳ��
		uint8_t restart;   // �Ƿ�������
		uint8_t err;          // �Ƿ����쳣��0->����û���쳣
		uint8_t warn;        // ������Ϣ
		uint8_t num;         // ģ�����
		uint8_t addr;        // ģ���PMBusͨѶ��ַ
		int8_t  temp;        // ģ���¶�
		int8_t temp_before;  // ��һ�ε��¶�
		uint8_t flag_ac_power;   // �Ƿ񿪻�
		uint8_t flag_delay_close_ac;    // ��ʱ�ر�AC���룬�÷��ȼ���ת������ɢ��
		uint8_t flag_bms_conn;   // bms�Ƿ�����
		uint8_t flag_mode_switch; // ģʽ�л���־
		uint16_t fan_speed;   // ģ��ķ���ת��

		uint8_t flag_smp_calibrated; // �Ƿ�ִ�й���ѹ����У׼,TRUE-ִ�й���FALSE-ûִ�й�
		float smp_bat_volt_ori;  // ������ԭʼ��ѹֵ
		float smp_bat_calibrate_value; // ������ѹУ׼ֵ
		float bat_volt;      // �洢���˼�⵽�ĵ�ص�ѹ
	}basic;
	
	PU_EventCacheDef event_cache;    // �¼�
	uint8_t flag_bat_disconn_event_cached;  // ��¼�Զ�ģʽ�£���ͨ�����ӣ����û�����ӵ�����£���ضϿ����¼��Ƿ��¼����TRUE-��¼����FALSE-û�м�¼��
	uint32_t event;    // �¼�
    BatInfoDefFromBMS bat_info_bms;  // ͨ��bms��ȡ�ĵ����Ϣ
	
	struct
	{
		uint8_t flag_full;    // ������־��������������󣬹ر�ģ�鵼������쳣������
		uint8_t flag_stop;    // ������ʾ�Ǹ��豸ֹͣ��
		uint8_t flag_start_failed;  // ��ʾ����ʧ��
		uint8_t st_timing_time;         // �Ƿ������ʱ��
		uint8_t st_booking_time;        // �Ƿ���ԤԼ���
		uint8_t flag_timing;            // ��ʾ��ʱʱ���Ƿ���
		uint8_t flag_booking;           // ��ʾԤԼʱ���Ƿ���
		uint8_t flag_bms_timeout;       // ��ʾbmsͨѶ��ʱ��

		uint8_t start_type;       // ������ʽ����������������ԤԼ������bms����
		uint8_t stop_type;        // ֹͣ��ʽ����������ֹͣ��ԤԼֹͣ��bmsֹͣ����λ��ֹͣ

	    float iout;             // ��������������Ϊƽ������
        float vout;             // �����ѹ
		float iout_ori;         // ʵʱ�ɼ��ĵ���
		float iout_disp;        // ������ʾ����
        Queue_Float iout_queue; // �����������ƽ��ֵ��ƽ���Ĵ���ʹ��IOUT_AVE_QUEUE_SIZE����
		float limit_iout;
		
		float auto_limit_iout;         // �Զ�ģʽ�£���ط��������Ƶ���
		float auto_bat_volt;           // �Զ�ģʽ�£���ط����ĵ�ص�ѹ
		float auto_bat_iout;           // �Զ�ģʽ�£���ط����ĳ�����
		float auto_bat_cap;          // �Զ�ģʽ�£���ص�ʣ������
		float auto_bat_default_cap;  // �Զ�ģʽ�£���صĶ����
		int8_t  auto_bat_temp;         // �Զ�ģʽ�£���ص��¶�
		
		float auto_vout_step;          // �Զ�ģʽ�£������ѹ�Ĳ���ֵ
//		float auto_flag_full;  // �Զ�ģʽ�£�����Ƿ������true��false
		uint8_t auto_flag_full_bms;
		uint8_t auto_flag_full_mcu;
		uint8_t auto_flag_full_threshold;
		uint8_t auto_flag_bms_start_query;  // ������Ӻ�bms��Ҫ10s�Ż��ȶ������Ե�һ�μ�⵽bms�����ݺ���Ҫ��ʱ10s֮�󣬽��������ݲ�����ȷ��

		// Ԥ���
		float volt_base;
		float iout_pre;

		uint8_t type;    // ��緽ʽ������ʽ��Ԥ���
		uint8_t bat_type;  // �������
		uint8_t flag_mode_switch_directly;  // ���ģʽ�Ƿ����ֱ���л���Ĭ��ΪFALSE
		
		ChgrParaDef para;       // ������
		CHGR_ParaDataDef data_max;   // �������ֵ
		CHGR_ParaDataDef data_min;   // ������Сֵ
        PreChgrParaDataDef pre_data_max; // Ԥ���������ֵ
        PreChgrParaDataDef pre_data_min; // Ԥ���������ֵ

		uint8_t mode;  // ���ģʽ���Զ����ֶ�
		CHGR_OPTypeDef op;      // ��¼֮ǰ������״̬������/ֹͣ
		uint16_t time;          // ���ʱ��
		uint16_t booking_time;     // ԤԼ���ʱ�䣬��λ����
		uint16_t timing_time;   // ��ʱ���ʱ�䣬��λ����
		uint16_t AH;            // �ѳ����
		__IO uint16_t step;    // ���Ĳ���״̬���������ccm��cvm��fvm�ȵ�
		__IO uint16_t status;    // ģ��״̬��ccm��cvm��fvm�ȵ�
		int32_t cnt_box[ CHGR_CNT_BOX_SIZE ];   // ���������ı�����
		uint16_t notify;         // ����ֵ֪ͨ��to_start, to_start_success, to_stop, to_stop success....
		uint8_t flag_in_chgr;    // �Ƿ��ڳ��״̬��TRUE��FALSE
	}chgr;
}PU_InfoDef;

typedef struct _bat_info
{
    float voltage;
}BAT_InfoDef;


typedef struct _dev_list_def
{
    PU_InfoDef dev;
	struct _dev_list_def *next;
}DevListDef, *pDevListDef;


typedef struct _chgr_dev_list_def
{
    PU_InfoDef *pdev;
	struct _chgr_dev_list_def *next;
}ChgrListDef, *pChgrListDef;

// ������ͨѶ�������
#define    HMI_FRAME_DATA_SIZE    125

enum _hmi_ico_def
{
    net = 0,
	bat,
	led,
	mode,
	op
};
typedef enum _hmi_ico_def HMI_IcoTypeDef;


typedef struct _hmi_rx_frame_def
{
    uint16_t header;
	uint8_t  len; // ����֡���ȣ���λ���ֽ�
	uint8_t  ins;
	struct
	{
	    uint16_t addr;
	    uint8_t  len;  // ���ݳ��ȣ���λ����
	    uint16_t  data[HMI_FRAME_DATA_SIZE];
	}data;
	struct
	{
	    uint8_t addr;
		uint8_t len;
		uint8_t data[HMI_FRAME_DATA_SIZE];  // �Ĵ����������� Byte�ģ�Ҳ��word�ģ����Զ����byte���͵�
	}data_reg;
}HMI_RxFrameDef;



#define     PASSWD_LEN_SET         6
#define     PASSWD_LEN_MAINTAIN    6

#define     DISP_NOTIFY_FLUSH                     (uint32_t)(bit(0))

typedef struct _hmi_info_def
{
	struct
	{
		struct
		{
		    uint16_t pre;
			uint16_t cur;
			uint16_t cur_rd;
		}id;
		uint16_t type; //���ģ��+�������ֱ�����ҳ�����ã��¼���ά��������������
	}page;
	
	struct
	{
		uint8_t type;
	    uint8_t st;
	}ico;
	
	struct
    {
	    uint8_t start_num;
		uint8_t stop_num;
		uint8_t set_num;
	}op;
	
	struct
    {
	    uint8_t cur_num;
		__IO ChgrParaDef tmp_para;
		__IO uint8_t tmp_set_passwd[ PASSWD_LEN_SET+1 ]; // ���볤��+�ַ�����β'\0'
		__IO uint16_t tmp_chgr_timing_time;
		__IO uint16_t tmp_chgr_booking_time;
		__IO uint8_t tmp_maintain_passwd[ PASSWD_LEN_MAINTAIN+1 ]; // ���볤��+�ַ�����β'\0'	
		uint8_t chgr_type;
		__IO uint16_t tmp_chgr_threshold;
	}set;

	struct
	{
		uint8_t cur_num;
	}maintain;

	struct
	{
		uint8_t cur_num;
		uint8_t page_sw;    // 0 - right, 1 - left
		int16_t page_num;
	}event;

	struct
	{
        uint8_t page_id;	
		uint8_t flag;
	}verify;
	
	uint32_t timeout;  // ������ʱ����ʱ����Ϊ5����
	uint32_t page_flush_time; // ˢ��ʱ��
	uint8_t flag_setting;
    Sys_TimeDef sys_time; // �洢ϵͳʱ��
	uint8_t serial_num[EE_SERIAL_NUM_LEN];
	uint8_t product_code[EE_PRODUCT_CODE_LEN];
}HMI_InfoDef;


// �����Ϣ���涨��
typedef struct _bat_info_fifo_def
{
	uint8_t *pfifo;    // ָ�򻺴���׵�ַ
	uint8_t *p_local;  // ָ�򱾵ص����Ϣ������׵�ַ
	uint8_t *p_remote; // ָ��Զ�����������Ϣ������׵�ַ
}BAT_InfoFifoDef;

typedef struct _sys_info_def
{
    HMI_InfoDef hmi_info;
	uint8_t net_status;
	BAT_InfoFifoDef  bat_fifo;
}SYS_InfoDef;

#define    sleep_ms(time)    vTaskDelay( pdMS_TO_TICKS( ( time ) ) )

void delay_ms( __IO uint32_t _T );
void delay_us( __IO uint32_t _T );
void erase_ee_flag( void );

void hmi_init(void);
void pu_info_init(void);
DevListDef *get_device_list( void );
void chgr_init(void);
void get_sys_time( Sys_TimeDef *ptime );
PU_InfoDef *get_device( uint8_t num );
BOOL is_been_init( uint8_t num );
void set_been_init( uint8_t num );
HMI_InfoDef *get_hmi_info( void );
BOOL is_exec_smp_calibrate( uint8_t num );
void erase_smp_calibrate( void );

BOOL is_wifi_conn(void);

void close_ac_input( uint8_t num );
void open_ac_input( uint8_t num );
void close_ac_input_all( void );
void open_ac_input_all( void );

BOOL read_event( PU_InfoDef *pdev, EE_SysEventDef *pevent, uint8_t page_num, uint8_t num_to_read, uint8_t *p_num_actual_read );

void power_on( uint8_t num );
void power_on_all( void );
void power_off( uint8_t num );
void power_off_all( void );

void open_dc_relay( uint8_t num );
void close_dc_relay( uint8_t num );

void on_off_control( uint8_t num, uint8_t op );

/* mode -> MODE_PMBUS - ѡ��pmbus����
 *         MODE_CURVE - ѡ�������߿���
 */
void chmod_select( uint8_t num, uint8_t op );

void pub_start( uint8_t start_type, uint8_t num );
void pub_stop(uint8_t stop_type,  uint8_t num );
void pub_stop_all( void );

void pub_open_output( uint8_t num );

void pub_bat_type_switch( uint8_t num, uint8_t to );
void disp_pmbus_comm_status( StatusTypeDef st_pmbus );
BOOL isBatConn( float volt );


uint16_t cache_bms_rx( SPI_TypeDef* SPIx, u8 port );

ChgrListDef *get_chgr_list(void);

void deal_with_bms_timeout( PU_InfoDef *pdev );
void deal_with_chgr_booking( PU_InfoDef *pdev );
void deal_with_chgr_timing( PU_InfoDef *pdev );

void suspend_hmi( void );
void resume_hmi( void );

void suspen_chgr( void );
void resume_chgr( void );

void suspend_ext( void );
void resume_ext( void );




#ifdef    ASSERT_APP
#define   _ASSERT_APP    1
#else
#define    _ASSERT_APP   0
#endif

#define    assert_app(x)      do{ \
	                                if (_ASSERT_APP) \
									{ \
									    if((x) ) \
									        assert_called(__FILE__,__LINE__); \
									} \
								}while(0)								
#define    assert_called(char,int)  printf("Error:%s,%d\r\n",char,int)

#endif // applications.h

