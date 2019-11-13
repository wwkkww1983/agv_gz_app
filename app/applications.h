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



#define    MS_BASE    10280ul    // 大约1ms
#define    MS(n)      ((MS_BASE)*(n))

#define    MODE_PMBUS 0x01u   // 通过pmbus控制充电，对应自动控制
#define    MODE_CURVE 0x00u   // 由模块本身的充电曲线控制充电，对应手动控制
#define    MODE_MANUAL MODE_CURVE
#define    MODE_AUTO   MODE_PMBUS

#define    OP_DISP_START   0x01u
#define    OP_DISP_STOP    0x00u
#define    OP_DISP_NONE    0x02u


// 模式切换
#define    BASIC_MODE_NONE_SWITCH             0x00
#define    BASIC_MODE_AUTO_TO_MANUAL          0X01
#define    BASIC_MODE_MANUAL_TO_AUTO          0X02
#define    BASIC_MODE_THREE_SECTION_TO_PRE    0X03
#define    BASIC_MODE_PRE_TO_THREE_SECTION    0X04

// 电池图标相关
#define    BASIC_BAT_DISCONN    0x00u
#define    BASIC_BAT_CONN       0x01u
#define    BASIC_BAT_REVERSE    0x02u
#define    BASIC_BAT_CHGR_ING   0x04u
#define    BASIC_BAT_FULL       0x08u

#define    LED_GREEN      0x00u
#define    LED_RED        0x01u

// 系统状态灯相关
#define    BASIC_ERR_NO   0x00u
#define    ERR_BAT_REVERSE  1u    // 该数字代表的是位
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
#define    BASIC_CHGR_STOP_BY_HOST    0x04u  // 被上位机手动停止

#define    BASIC_RESTART_NO    0x00u
#define    BASIC_RESTART_YES   0x01u

// 模块是否开机
#define    BASIC_POWER_ON    0x00u
#define    BASIC_POWER_OFF   0x01u

// touch_screen_entry的任务通知值
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



// 启动方式
#define    START_TYPE_HMI        0x00u
#define    START_TYPE_BOOKING    0x01u
#define    START_TYPE_BMS        0x02u

// 停止方式
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
#define    POWER_UNIT_SY   0x08 // 表示进入了同步模式


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
#define    IDX_CNT_AVE_IOUT                       24    // 用来平均电流值
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
 * RCB1600，电池充电状态信息定义，来自RCB-1600手册
 */
// Low byte
// Bit 0 FULLM：充飽電模式狀態，0＝未充飽電，1＝充飽電
#define			FULLM						0x0001
// Bit 1 CCM：定電流充電模式狀態，0＝充電器非處於定電流模式，1＝充電器處於定電流模式
#define			CCM							0x0002
// Bit 2 CVM：定電壓充電模式狀態，0＝充電器非處於定電壓模式，1＝充電器處於定電壓模式
#define			CVM							0x0004
// Bit 3 FVM：浮充模式狀態，0＝充電器非處於浮充模式，1＝充電器處於浮充模式
#define			FVM							0x0008

// High byte
// Bit 0 EEPER: EEPROM充電參數錯誤，0＝充電參數資料正確，1＝充電參數資料錯誤
#define			EEPER						0x0100
// Bit 2 NTCER: 溫度補償短路，0＝溫度補償線路無發生短路，1＝溫度補償線路發生短路
#define			NTCER						0x0400
// Bit 3 BTNC: 電池未接，0＝偵測到電池，1＝未偵測到電池
#define			BTNC						0x0800
// Bit 5 CCTOF：定電流階段充電超時旗標，0＝定電流階段充電未超時，1＝定電流階段充電超時
#define			CCTOF						0x2000
// Bit 6 CVTOF：定電壓階段充電超時旗標，0＝定電壓階段充電未超時，1＝定電壓階段充電超時
#define			CVTOF						0x4000
// Bit 7 FTTOF：浮充階段充電超時旗標，0＝浮充階段充電未超時，1＝浮充階段充電超時
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



// 任务相关
#define    TOUCH_SCREEN_STACK    512
#define    TOUCH_SCREEN_PRIO     5
extern TaskHandle_t h_touch_screen_entry;
extern TaskHandle_t xTouchScreenDamonHandle; // 屏幕刷新任务的句柄
void touch_screen_entry( void *pvParameter );


//#define   CHGR_STACK    256
//#define   CHGR_PRIO     5
//extern TaskHandle_t h_charge_entry;
//void charge_entry( void * pvParameters );

// 处理BMS通信，该任务尽量不要被其他任务打断
#define   EXT_BMS_COMM_STACK    256
#define   EXT_BMS_COMM_PRIO     5
extern TaskHandle_t h_ext_bms_comm_entry; 
//void ext_interface_entry( void * pvParameters );
void 	sExt_bms_comm_Task(void);

// 处理BMS通信接收的任务
#define   EXT_BMS_COMM_RX_STACK    356
#define   EXT_BMS_COMM_RX_PRIO     5
extern TaskHandle_t h_ext_bms_comm_rx_entry; 
//void ext_interface_entry( void * pvParameters );
void 	sExt_bms_comm_rx_Task(void);


#define   EXT_WIFI_STACK    1024
#define   EXT_WIFI__PRIO     3
extern TaskHandle_t h_ext_wifi_entry; 
void   ext_wifi_entry( void * pvParameters );

#ifdef USE_CAN_SYNC // 使用双机通信, 20191101添加
#define   CAN_COMM_STACK    512
#define   CAN_COMM_PRIO     3
extern TaskHandle_t h_can_comm_entry; 
void   can_comm_entry( void * pvParameters );
#endif

#ifdef USE_WDG
#define    WDG_STACK    256
#define    WDG_PRIO     8    // 优先级需要设置为最高
extern TaskHandle_t h_wdg_daemon;
void wdg_daemon_task(void *pvParameters);

// 看门狗相关
// 对应的任务会给监视任务发送以下的事件，只有当所有的任务都发了事件后，才会执行喂狗操作
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

extern QueueHandle_t xQueuePrint; // 向打印任务传输数据的消息队列


// 软件定时相关定义
#define    HMI_SW_TIMER_NUM    2
#define    ID_SW_TIMER_0       0    // 设置超时定时器
#define    ID_SW_TIMER_1       1    // 启动超时定时器

#define    HMI_SW_TIMER_0_PERIOD    20000 // 设置超时：20s
#define    HMI_SW_TIMER_1_PERIOD    60000 // 启动超时：60s


#define    SW_TO_RIGHT    0x00
#define    SW_TO_LEFT     0x01

#define    EVENT_NULL        (uint32_t)0

#define    EVENT_MAX_NUM    9        // 事件数
#define    EVENT_MAX_STORE_NUM    40 // 单个模块最多存储的事件数量
#define    EVENT_EACH_PAGE_NUM    4    // 每页显示4个
#define    EVENT_MAX_PAGE_NUM    ( ( EVENT_MAX_STORE_NUM ) / ( EVENT_EACH_PAGE_NUM ) )

#define    EVENT_START        0u        // 事件 - 启动
#define    EVENT_STOP         1u        // 事件 - 停止
#define    EVENT_BAT_REVERSE  2u        // 事件 - 电池反接
#define    EVENT_BAT_TYPE_ERR 3u        // 事件 - 电池类型错误
#define    EVENT_DC_OK_ERR    4u        // 事件 - 输出异常
#define    EVENT_T_ALARM      5u        // 事件 - 温度异常
#define    EVENT_BAT_DISCONN  6u        // 事件 - 电池未连接
#define    EVENT_OVER_OUTPUT  7u        // 事件 - 过压
#define    EVENT_ILLEGAL_STOP 8u        // 事件 - 非法停止


/*
 *  eeporm中存储划分范围
 *  base - 166
 *           | is_full(2Byte) | latest_idx(2Byte*power_uni_num) | store_base |
 *
 *  latest_idx -> 表示可以写入的最新序号
 */
#define    EE_EVENT_IS_FULL_LEN    ( 2 )
#define    EE_EVENT_LATEST_IDX_LEN ( 2 )

#define    EE_EVENT_BASE_ADDR       ( 900 )
#define    EE_EVENT_IS_FULL_ADDR    ( EE_EVENT_BASE_ADDR )
#define    EE_EVENT_LATEST_IDX_BASE_ADDR ( EE_EVENT_IS_FULL_ADDR + EE_EVENT_IS_FULL_LEN )
#define    EE_EVENT_STORE_BASE_ADDR      ( EE_EVENT_LATEST_IDX_BASE_ADDR + ( EE_EVENT_LATEST_IDX_LEN * POWER_UNIT_NUM ) )


// 产品编码和生产序号相关
#define  EE_SERIAL_NUM_ADDR   550
#define  EE_SERIAL_NUM_LEN    16 
#define  EE_PRODUCT_CODE_ADDR 580
#define  EE_PRODUCT_CODE_LEN  11



typedef struct _sys_time_def
{
	uint8_t year;    // 年
	uint8_t month;    // 月
	uint8_t day;      // 日
	uint8_t hour;     // 时
	uint8_t min;      // 分
	uint8_t sec;      // 秒
}Sys_TimeDef;


enum _pu_chgr_op
{
    op_stop = 0,  //表示执行停止，界面显示“启动”
	op_start,      //表示执行启动，界面显示“停止”
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
    float ichg;    // 均冲电流
	float itaper;  // 浮充电流
	float vbst;    // 终止电压
	float vfloat;  // 浮充电压
}CHGR_ParaDataDef;

typedef struct _chgr_pre_para_data_def
{
	float pre_iout;    // 预充电流
	float pre_vout;    // 预充电压
	float ichg;        // 恒充电流
	float vbst;        // 恒充电压
	float itaper;      // 终止电流
}PreChgrParaDataDef;

typedef struct _chgr_para_data_auto_def
{
    uint16_t threshold;    // 自动充电的阈值，电量百分比>=该值时，认为充满，停止充电
}AutoChgrParaDataDef;

typedef struct _pu_chgr_para
{
	uint16_t ee_addr;       // 充电参数在eeprom中保存的起始地址
    uint8_t curve;          // 当前的曲线
    CHGR_ParaDataDef data;  // 详细充电参数
    PreChgrParaDataDef data_pre; // 预充电充电参数
    AutoChgrParaDataDef data_auto;  // 预充电参数
}ChgrParaDef;

typedef struct _sys_ee_sys_event_def
{
	uint16_t code;   // 代码, 高8位表示事件代码，低八位的高四位代表模式，低四位代表参数
	Sys_TimeDef sys_time; // 系统时间
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
	TERPOLYMER = 0,               // 三元电池
	LITHIUM_FER_PHOSPHATE,        // 铁锂电池
	LITHIUM_TITANATE,             // 钛酸锂电池
	ERROR_TYPE                    // 未知类型
};

typedef struct _bat_info_from_bms
{
	uint16_t volt;     // 电池的电压，分辨率0.01V
	uint16_t curr_in;  // 电池的充电电流，分辨率0.1A
	uint16_t temp;     // 电池的温度2，分辨率0.1
	uint16_t cap;      // 电池的剩余容量，0.1%
	uint16_t limit_curr;  // 500表示充电桩额定电流的50%，额定电流按照30计算
	uint16_t volt_single_14;  // 单体14的电压，分辨率1mV
	uint16_t volt_single_16;
	uint16_t soh_val;
	uint16_t full_volt;     // 满电电压
	uint16_t single_bat_volt_len;
	uint8_t flag_full_volt; // 是否确定了满电电压, TRUE-确定了，FALSE-没有确定
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
		uint8_t bat;       // 电池是否连接
		uint8_t chgr;      // 是否在充电
		uint8_t restart;   // 是否在重启
		uint8_t err;          // 是否有异常，0->代表没有异常
		uint8_t warn;        // 警告信息
		uint8_t num;         // 模块序号
		uint8_t addr;        // 模块的PMBus通讯地址
		int8_t  temp;        // 模块温度
		int8_t temp_before;  // 上一次的温度
		uint8_t flag_ac_power;   // 是否开机
		uint8_t flag_delay_close_ac;    // 延时关闭AC输入，让风扇继续转，用来散热
		uint8_t flag_bms_conn;   // bms是否连接
		uint8_t flag_mode_switch; // 模式切换标志
		uint16_t fan_speed;   // 模块的风扇转速

		uint8_t flag_smp_calibrated; // 是否执行过电压采样校准,TRUE-执行过，FALSE-没执行过
		float smp_bat_volt_ori;  // 采样的原始电压值
		float smp_bat_calibrate_value; // 采样电压校准值
		float bat_volt;      // 存储过滤检测到的电池电压
	}basic;
	
	PU_EventCacheDef event_cache;    // 事件
	uint8_t flag_bat_disconn_event_cached;  // 记录自动模式下，当通信连接，电池没有连接的情况下，电池断开的事件是否记录过，TRUE-记录过，FALSE-没有记录过
	uint32_t event;    // 事件
    BatInfoDefFromBMS bat_info_bms;  // 通过bms读取的电池信息
	
	struct
	{
		uint8_t flag_full;    // 充满标志，用来处理充满后，关闭模块导致输出异常的问题
		uint8_t flag_stop;    // 用来表示那个设备停止了
		uint8_t flag_start_failed;  // 表示启动失败
		uint8_t st_timing_time;         // 是否开启充电时间
		uint8_t st_booking_time;        // 是否开启预约充电
		uint8_t flag_timing;            // 表示定时时间是否到了
		uint8_t flag_booking;           // 表示预约时间是否到了
		uint8_t flag_bms_timeout;       // 表示bms通讯超时了

		uint8_t start_type;       // 启动方式，包含界面启动，预约启动，bms启动
		uint8_t stop_type;        // 停止方式，包含界面停止，预约停止，bms停止，上位机停止

	    float iout;             // 输出电流，后更改为平均电流
        float vout;             // 输出电压
		float iout_ori;         // 实时采集的电流
		float iout_disp;        // 界面显示电流
        Queue_Float iout_queue; // 用来计算电流平均值，平均的次数使用IOUT_AVE_QUEUE_SIZE定义
		float limit_iout;
		
		float auto_limit_iout;         // 自动模式下，电池反馈的限制电流
		float auto_bat_volt;           // 自动模式下，电池反馈的电池电压
		float auto_bat_iout;           // 自动模式下，电池反馈的充电电流
		float auto_bat_cap;          // 自动模式下，电池的剩余容量
		float auto_bat_default_cap;  // 自动模式下，电池的额定容量
		int8_t  auto_bat_temp;         // 自动模式下，电池的温度
		
		float auto_vout_step;          // 自动模式下，输出电压的步进值
//		float auto_flag_full;  // 自动模式下，检测是否充满，true，false
		uint8_t auto_flag_full_bms;
		uint8_t auto_flag_full_mcu;
		uint8_t auto_flag_full_threshold;
		uint8_t auto_flag_bms_start_query;  // 电池连接后，bms需要10s才会稳定，所以第一次检测到bms有数据后，需要延时10s之后，解析的数据才是正确的

		// 预充电
		float volt_base;
		float iout_pre;

		uint8_t type;    // 充电方式，三段式，预充电
		uint8_t bat_type;  // 电池类型
		uint8_t flag_mode_switch_directly;  // 标记模式是否可以直接切换，默认为FALSE
		
		ChgrParaDef para;       // 充电参数
		CHGR_ParaDataDef data_max;   // 参数最大值
		CHGR_ParaDataDef data_min;   // 参数最小值
        PreChgrParaDataDef pre_data_max; // 预充电参数最大值
        PreChgrParaDataDef pre_data_min; // 预充电参数最大值

		uint8_t mode;  // 充电模式，自动，手动
		CHGR_OPTypeDef op;      // 记录之前操作的状态，启动/停止
		uint16_t time;          // 充电时间
		uint16_t booking_time;     // 预约充电时间，单位分钟
		uint16_t timing_time;   // 定时充电时间，单位分钟
		uint16_t AH;            // 已充电量
		__IO uint16_t step;    // 充电的步骤状态，打开输出，ccm，cvm，fvm等等
		__IO uint16_t status;    // 模块状态，ccm，cvm，fvm等等
		int32_t cnt_box[ CHGR_CNT_BOX_SIZE ];   // 用来计数的变量；
		uint16_t notify;         // 任务通知值，to_start, to_start_success, to_stop, to_stop success....
		uint8_t flag_in_chgr;    // 是否处于充电状态，TRUE，FALSE
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

// 触摸屏通讯任务相关
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
	uint8_t  len; // 数据帧长度，单位是字节
	uint8_t  ins;
	struct
	{
	    uint16_t addr;
	    uint8_t  len;  // 数据长度，单位是字
	    uint16_t  data[HMI_FRAME_DATA_SIZE];
	}data;
	struct
	{
	    uint8_t addr;
		uint8_t len;
		uint8_t data[HMI_FRAME_DATA_SIZE];  // 寄存器的数据有 Byte的，也有word的，所以定义成byte类型的
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
		uint16_t type; //五大模块+屏保，分别是主页，设置，事件，维护，帮助，屏保
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
		__IO uint8_t tmp_set_passwd[ PASSWD_LEN_SET+1 ]; // 密码长度+字符串结尾'\0'
		__IO uint16_t tmp_chgr_timing_time;
		__IO uint16_t tmp_chgr_booking_time;
		__IO uint8_t tmp_maintain_passwd[ PASSWD_LEN_MAINTAIN+1 ]; // 密码长度+字符串结尾'\0'	
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
	
	uint32_t timeout;  // 屏保超时，暂时定义为5分钟
	uint32_t page_flush_time; // 刷新时间
	uint8_t flag_setting;
    Sys_TimeDef sys_time; // 存储系统时间
	uint8_t serial_num[EE_SERIAL_NUM_LEN];
	uint8_t product_code[EE_PRODUCT_CODE_LEN];
}HMI_InfoDef;


// 电池信息缓存定义
typedef struct _bat_info_fifo_def
{
	uint8_t *pfifo;    // 指向缓存的首地址
	uint8_t *p_local;  // 指向本地电池信息缓存的首地址
	uint8_t *p_remote; // 指向远程主机电池信息缓存的首地址
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

/* mode -> MODE_PMBUS - 选择pmbus控制
 *         MODE_CURVE - 选择充电曲线控制
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

