/************************************************************************
note.
20181219，完成数据解析模块的开发
20181220，完成系统数据结构的设计；完成主页和主页详细页面显示的开发
20181221, 完成主页和详细页面的调试
          优化数据结构设计，把所有的设备使用链表管理起来，便于操作，精简代码
		  添加5个模块的概览显示，详细显示，交互
20181226, 完成启动和停止的切换
        , 添加设置，三段式充电设置
20181227, 修改默认参数为：RCB1600-48的参数
20190109, 添加急停显示
20190114, 同步设置调试通过
          SET_NOTIFY_TO_RESTART_ALL, ->touch_screen_entry->charge_entry
          SET_NOTIFY_RESTART_FINISHED, restart_dev_all -> touch_screen_entry
20190115, 添加充电时长设置功能，调试通过
          添加预约充电设置功能，调试通过

		  添加模块6-8的主页详细信息刷新
		  添加模块6-8的主页概览信息刷新
		  添加模块6-8的充电设置
20190118, 添加维护功能，开发完成，初步调试通过 
20190124, 完成历史事件查询功能
          - 事件查询
20190216, 添加"电池未连接"事件显示
20190222, 修改获取屏幕时间机制
          - 间隔5s发送一次获取时间指令，把获取到的时间存储到phmi_info里面
		  - 别的模块从phmi_info里面获取时间

备忘：
1. 记得添加参数设置的上下限制!
2. 模式切换已经屏蔽，后期需要记得打开
3. 已经屏蔽6,7,8模块的详细信息刷新，后期需要记得打开 - 20190115已经打开所有模块的数据刷新
*************************************************************************/


#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "limits.h"

#include "board.h"
#include "gpio.h"
#include "uart.h"
#include "eeprom.h"
#include "bsp_rpb1600_48.h"


#include "applications.h"
#include "service.h"

#include <string.h>
#include <stdlib.h>


#define    FRAME_HEADER     (uint16_t)0x5AA5
#define    CMD_WR_REG    0x80
#define    CMD_RD_REG    0x81
#define    CMD_WR_VAR    0x82
#define    CMD_RD_VAR    0x83
#define    CMD_WR_CURVE  0x84


#define    HMI_KEY_ADDR_START_SYSTEM  0x0005

#define    HMI_KEY_ADDR_START_MAIN    0x0011
#define    HMI_KEY_ADDR_END_MAIN      0x0021

#define    HMI_KEY_ADDR_SET           0x0022

#define    HMI_KEY_ADDR_EVENT         0x0033
#define    HMI_VAR_ADDR_START_EVENT   0x1811
#define    HMI_VAR_ADDR_END_EVENT     0x2010

#define    HMI_KEY_ADDR_MAINTAIN         0x0044
#define    HMI_VAR_ADDR_START_MAINTAIN   0x2011
#define    HMI_VAR_ADDR_END_MAINTAIN     0x2810

#define    HMI_KEY_ADDR_HELP         0x0055
#define    HMI_VAR_ADDR_START_HELP   0x2811
#define    HMI_VAR_ADDR_END_HELP     0x3010


#define    HMI_MAX_PAGE_ID    184  // 界面版本20191101更新为184

#define    HMI_SCREEN_SAVE_PAGE_ID 0
#define    HMI_MAIN_PAGE_ID        1
#define    HMI_SET_PAGE_ID         10
#define    HMI_EVENT_PAGE_ID       46
#define    HMI_MAINTAIN_PAGE_ID    64
#define    HMI_HELP_PAGE_ID        75

#define    HMI_NO_START_PAGE_ID    96
#define    HMI_NO_SYN_SETTING_PAGE_ID    97
#define    HMI_NO_SETTING_PAGE_ID    98

// 定义需要数据刷新页面的ID
#define    HMI_MAIN_OVERVIEW_ID    1
#define    HMI_MAIN_DETAIL_1_ID    2
#define    HMI_MAIN_DETAIL_2_ID    3
#define    HMI_MAIN_DETAIL_3_ID    4
#define    HMI_MAIN_DETAIL_4_ID    5
#define    HMI_MAIN_DETAIL_5_ID    6
#define    HMI_MAIN_DETAIL_6_ID    7
#define    HMI_MAIN_DETAIL_7_ID    8
#define    HMI_MAIN_DETAIL_8_ID    9
#define    HMI_MAIN_STARTING_ID    82
#define    HMI_MAIN_START_FAILED_ID    83
#define    HMI_MAIN_STOP_ING_ID    84
#define    HMI_MAIN_EMERGENCY_STOP_ID    86

#define    HMI_MAIN_TYPE_SWITCH_ID    93

//  
#define    HMI_SET_PASSWD_ERR_ID      13
#define    HMI_SET_ENTER_ID           14
#define    HMI_SET_CHGR_SET_1_ID   18
#define    HMI_SET_CHGR_SET_2_ID   19
#define    HMI_SET_CHGR_SET_3_ID   20
#define    HMI_SET_CHGR_SET_4_ID   21
#define    HMI_SET_CHGR_SET_5_ID   22
#define    HMI_SET_CHGR_SET_6_ID   23
#define    HMI_SET_CHGR_SET_7_ID   24
#define    HMI_SET_CHGR_SET_8_ID   25
#define    HMI_SET_CHGR_SET_SY_ID  26

#define    HMI_SET_CURVE1_ID       27
#define    HMI_SET_CURVE2_ID       28
#define    HMI_SET_CURVE3_ID       29
#define    HMI_SET_CURVE4_ID       30
#define    HMI_SET_CURVE_U_ID      31
#define    HMI_SET_SET_ING_ID      38

#define    HMI_SET_CHGR_TIME_OFF_ID     40
#define    HMI_SET_CHGR_TIME_ON_ID      41
#define    HMI_SET_CHGR_BOOKING_OFF_ID      43
#define    HMI_SET_CHGR_BOOKING_ON_ID       44

#define    HMI_SET_CHGR_RESTROE_FACTORY_ID           78
#define    HMI_SET_CHGR_RESTROE_FACTORY_CONFIRM_ID   79
#define    HMI_SET_CHGR_RESTROE_FACTORY_ING_ID       80
#define    HMI_SET_CHGR_RESTROE_FACTORY_FINISH_ID    81


#define    HMI_SET_PASSWD_SETTING_ID            87
#define    HMI_SET_PASSWD_SETTING_MODIFY_ID     88
#define    HMI_SET_PASSWD_SETTING_FAIL_ID       90
//

#define    HMI_MAINTAIN_PASSWD_ERR_ID      66
#define    HMI_MAINTAIN_ENTER_ID           67
#define    HMI_MAINTAIN_DETAIL_1_ID        67
#define    HMI_MAINTAIN_DETAIL_2_ID        68
#define    HMI_MAINTAIN_DETAIL_3_ID        69
#define    HMI_MAINTAIN_DETAIL_4_ID        70
#define    HMI_MAINTAIN_DETAIL_5_ID        71
#define    HMI_MAINTAIN_DETAIL_6_ID        72
#define    HMI_MAINTAIN_DETAIL_7_ID        73
#define    HMI_MAINTAIN_DETAIL_8_ID        74


#define    FLAG_BENN_INIT            0x01  // 已经初始化的标志
#define    EE_ADDR_BEEN_INIT_FLAG    0     // 存放已经初始化标志的地址 

#define    EE_CHGR_PARA_ADDR_1     10
#define    EE_CHGR_PARA_ADDR_2     70
#define    EE_CHGR_PARA_ADDR_3     130
#define    EE_CHGR_PARA_ADDR_4     190
#define    EE_CHGR_PARA_ADDR_5     250
#define    EE_CHGR_PARA_ADDR_6     310
#define    EE_CHGR_PARA_ADDR_7     370
#define    EE_CHGR_PARA_ADDR_8     430

#define    EE_PASSWD_SET           499
#define    EE_PASSWD_MAINTAIN      500

#define    EE_OFFSET_CHGR_THRESHOLD    37 // EE_CHGR_PARA_ADDR + 该偏移量是存放阈值的地址

#define    EE_OFFSET_SMP_CALIBRATED    49 // 存储是否校准过标志位的偏移地址
#define    EE_OFFSET_SMP_CALIBRATE_VAL 50 // 存储校准的值
#define    LEN_SMP_CALIBRATE_VAL       2  // 校准值长度


/**********************************************************
 当前文件的全局变量定义
**********************************************************/
// 解析后的数据帧存储变量
static HMI_RxFrameDef hmi_rx_frame;
static HMI_RxFrameDef *phmi_rx_frame = &hmi_rx_frame;
// 存储屏幕信息的变量
static HMI_InfoDef hmi_info;
static HMI_InfoDef *phmi_info = &hmi_info;
// 存储功率模块信息的变量
static PU_InfoDef *pdev[POWER_UNIT_NUM+1] = { NULL };
static DevListDef *phead_device_list = NULL;
static DevListDef *create_dev_list( uint8_t num );

static uint16_t Op_NumToPageId[] = 
{
    HMI_MAIN_DETAIL_1_ID,
    HMI_MAIN_DETAIL_2_ID,
    HMI_MAIN_DETAIL_3_ID,
    HMI_MAIN_DETAIL_4_ID,
    HMI_MAIN_DETAIL_5_ID,
    HMI_MAIN_DETAIL_6_ID,
    HMI_MAIN_DETAIL_7_ID,
    HMI_MAIN_DETAIL_8_ID,
    HMI_MAIN_OVERVIEW_ID
};

static uint16_t Set_NumToPageId[] = 
{
    HMI_SET_CHGR_SET_1_ID, 
    HMI_SET_CHGR_SET_2_ID, 
    HMI_SET_CHGR_SET_3_ID, 
    HMI_SET_CHGR_SET_4_ID, 
    HMI_SET_CHGR_SET_5_ID, 
    HMI_SET_CHGR_SET_6_ID, 
    HMI_SET_CHGR_SET_7_ID, 
    HMI_SET_CHGR_SET_8_ID, 
    HMI_SET_CHGR_SET_SY_ID
};

const static uint16_t ee_chgr_para_addr[] =
{
    EE_CHGR_PARA_ADDR_1,
    EE_CHGR_PARA_ADDR_2,
    EE_CHGR_PARA_ADDR_3,
    EE_CHGR_PARA_ADDR_4,
    EE_CHGR_PARA_ADDR_5,
    EE_CHGR_PARA_ADDR_6,
    EE_CHGR_PARA_ADDR_7,
    EE_CHGR_PARA_ADDR_8
};

static uint8_t set_passwd_default[] = { '6', '6', '6', '6', '6', '6' };
static uint8_t maintain_passwd_default[] = { '6', '6', '6', '6', '6', '6' };

const static char *Event_history_tab[] = 
{
"启动",
"停止",
"电池反接",
"电池类型错误",
"输出异常",
"温度异常",
"电池未连接",
"过压",
"非法停止"
};
const static char *Event_chgr_mode_tab[] = 
{
"自动",
"手动"
};
const static char *Event_chgr_para_tab[] = 
{
"一",
"二",
"三",
"四",
"五"
};



// 对应二维下标
#define    IDX_ICHG   0
#define    IDX_ITAPER 1
#define    IDX_VBST   2
#define    IDX_VFLOAT 3

//[x][0] - [x][3], ichg,itaper,vbst,vfloat
const float default_curve_data[][4] =
{
{27.5, 2.8, 57.6, 55.2},
{27.5, 2.8, 56, 54.4},
{27.5, 2.8, 56.8, 53.6},
{27.5, 2.8, 58, 54},
{27.5, 2.8, 57.6, 55.2},
};

/* 参数最大值，最小值，[0] - 最大值，[1]-最小值
 * 其中，max_itaper <= max_ichg, max_vfloat <= max_vbst
 *       min_itaper <= min_ichg, min_vfloat <= min_vbst
 * 在参数设置时，需要注意及时的判断
 */
const float curve_data_mm[][4] = 
{
{27.5, 8.3, 60, 60},
{5.5, 1.3, 36, 36}
};

#define    IDX_PRE_CHGR_PRE_IOUT    0
#define    IDX_PRE_CHGR_PRE_VOUT    1
#define    IDX_PRE_CHGR_ICHG        2
#define    IDX_PRE_CHGR_VBST        3
#define    IDX_PRE_CHGR_ITAPER      4
// 预充电相关
const float default_pre_chgr_para_data[] = { 3, 48, 27.5, 57.6, 1.5 };
const float pre_chgr_data_mm[][5] = 
{
{10, 50, 27.5, 60, 5},
{1,  32, 1,    48, 1}
};

/**********************************************************
 当前文件的私有函数
**********************************************************/
// 处理屏幕接收的数据帧，包括解析，判断，执行等
static void process_hmi_rx_frame(void);
// 解析数据
static BOOL frame_parse(void);
// 简单的数据帧检查
static BOOL frame_chk(void);

// 初始化相关
static void init_device_list(void);
static void init_power_unit_info(void);

// for debug
static void dbg_print_hmi_frame(void);


/**********
  屏保+其它
**********/
static void hmi_screen_save(void);
static void hmi_update_page_id( uint16_t page_id );
static void hmi_switch_to_spec_page( uint16_t page_id );
static void hmi_start_animate( void );
static void hmi_stop_animate( void );
static void hmi_disp_para_data( uint16_t addr, uint16_t data );
static void hmi_disp_emergency_stop( void );
static void hmi_disp_start_from_emergency_stop( void );
static void hmi_read_cur_page_id(void);

static void hmi_disp_string( uint16_t addr, uint8_t *str, uint8_t len );

static void sys_led_indicator_flush(void);

static void hmi_read_sys_time( void );

static void hmi_disp_wifi( uint8_t st );

static void hmi_verify_page_switch( uint16_t page_num );

static void hmi_timeout( void );
static void hmi_reset( void );

static void hmi_start_chgr( void );
static void hmi_stop_chgr( void );

static void flush(void);


/********** 
  主页
**********/
static void hmi_main(void);
static void hmi_main_overview(void);
static void hmi_main_detail(void);
static void main_disp_bat_info( uint8_t num );
static void main_disp_bat_type( uint8_t bat_type, uint8_t num );
static void main_disp_bat_switch_ico( uint8_t to, uint8_t num );

static void exec_chgr_op_switch( uint8_t num );
static void exec_bat_type_switch( uint8_t num );

/*
 * 主页显示相关的函数
 */
// 显示充电模式，自动/手动
static void main_disp_mode( uint8_t to, uint8_t num );
// 显示充电操作，启动/停止
static void main_disp_op( uint8_t to, uint8_t num );
// 显示电池图标状态
static void main_disp_bat( uint8_t to, uint8_t num );
// 显示指示灯的状态
static void main_disp_led( uint8_t to, uint8_t num );
// 显示输出电压
static void main_disp_vout( float fvalue, uint8_t num );
// 显示电池电压
static void main_disp_bat_volt( float fvalue, uint8_t num );
// 显示输出电流
static void main_disp_iout( float fvalue, uint8_t num );
// 显示充电状态，异常/正常
static void main_disp_chgr_status( uint8_t st, uint8_t num );
// 显示充电时间
static void main_disp_chgr_time( uint16_t time, uint8_t num );
// 显示模块温度，-128 - 127
static void main_disp_temperature( int8_t time, uint8_t num );
// 显示已充电量
static void main_disp_AH( uint16_t cap, uint8_t num );
// 显示输出电压错误
static void main_disp_DC_OK( uint8_t dc_ok, uint8_t num );
// 显示启动相关
static void main_disp_starting( void );
static void main_disp_start_success( void );
static void main_disp_start_failed( void );

static void main_disp_chgr_type_switch(void);
static void main_disp_chgr_type_switch_finish(void);

/**********
  设置
**********/
static void hmi_set( void );

// 输入密码
static void set_enter( void );
static void set_update_tmp_input_passwd( void );
static void set_read_passwd( uint8_t *passwd ); 
static void set_save_passwd( uint8_t *passwd );

// 充电设置
  // 三段式充电设置
static void set_disp_chgr_set( uint8_t num );
static void set_disp_curve_data( CHGR_ParaDataDef *pdata );
static void set_disp_curve( uint8_t curve );
  // 预充电充电设置
static void set_disp_pre_chgr_para_data( PreChgrParaDataDef *pdata );
static void set_save_pre_chgr_para_syn( void );
static void set_save_pre_chgr_para( uint8_t num );

static void set_update_temp_curve_data( uint8_t num, uint8_t curve );
static void set_save_chgr_para( uint8_t num );
static void set_save_chgr_para_syn( void );
static void set_read_chgr_para_from_ee( ChgrParaDef *para  );
static void set_write_chgr_para_to_ee( ChgrParaDef *para );
static BOOL set_write_chgr_para_to_unit_verify( void (*write)(uint8_t addr, float para), float (*read)(uint8_t addr), \
	                                 uint8_t addr, float para_wr, float range );
static void set_restart( uint8_t num );
static void set_restart_finished( uint8_t num );

static void set_chgr_para_setting( void );

static void set_save_chgr_para_finished( uint8_t num );

static void set_chk_input_para( ChgrParaDef *para );

  // 充电时长设置
static void set_disp_chgr_timing_time( uint16_t timing_time );
static void set_disp_chgr_timing_time_set( uint8_t num );
static void set_update_temp_chgr_timing_time( uint8_t num );
static void set_chgr_timing_time_set_finish( uint8_t num );
static void set_close_chgr_timing_time_set( uint8_t num );

  // 预约充电设置
static void set_disp_chgr_booking_time( uint16_t book_time );
static void set_disp_chgr_booking_time_set( uint8_t num );
static void set_update_temp_chgr_booking_time( uint8_t num );
static void set_chgr_booking_time_set_finish( uint8_t num );
static void set_close_chgr_booking_time_set( uint8_t num );

  // 阈值设置
static void set_update_temp_chgr_threshold( uint8_t num );
static void set_disp_chgr_threshold( uint16_t data  );
static void set_chgr_threshold_set_finish( uint8_t num );

// 密码设置

static void set_enter_passwd_setting( void );
static void set_passwd_setting_verify( void );

// 恢复出厂设置
static void set_restore_factory_default( void );

/**********
  事件
**********/
static void hmi_event(void);

static void event_disp_cur_setting_num( uint8_t num );
static void event_disp_cur_setting( uint8_t num );

static void event_disp_cur_history_num( uint8_t num );
static void event_disp_cur_history( uint8_t num, uint8_t page_num );
static void event_history_sw_page( uint8_t num, uint8_t sw_dir );
static void event_disp_one_line( uint8_t line, uint8_t num, EE_SysEventDef event );
static void event_disp_clear_one_line( uint8_t line, uint8_t num );

/**********
  维护
**********/
static void hmi_maintain(void);

static void maintain_enter( void );
static void maintain_update_tmp_input_passwd( void );
static void maintain_read_passwd( uint8_t *passwd ); 
static void maintain_save_passwd( uint8_t *passwd );

static void maintain_disp_detail( uint8_t num );
static void maintain_disp_chgr_status( uint16_t addr, char *str );
static void maintain_update_detail(void);

/**********
  帮助
**********/
static void hmi_help(void);

#define    TOUCH_SCREEN_DAMON_STACK    256
#define    TOUCH_SCREEN_DAMON_PRIO     3
TaskHandle_t xTouchScreenDamonHandle = NULL;
static void prvTouchScreenDamonTask( void *pvParameter );
static void suspend_touch_screen_damon(void);
static void resume_touch_screen_damon(void);


#define    SETTING_DAMON_STACK    256
#define    SETTING_DAMON_PRIO     3
TaskHandle_t xSettingDamonHandle = NULL;
static void prvSettingDamonTask( void *pvParameter );

#define    NOTIFY_CHGR_PARA_SETTING        (uint16_t)(bit(0))
#define    NOTIFY_RESTORE_FACTORY_SETTING  (uint16_t)(bit(1))

//static uint8_t addr_list[8] = {0x82, 0x84, 0x86, 0x88, 0x8A, 0, 0, 0};
//static uint8_t addr_list[8] = {0x86, 0x84, 0x82, 0x88, 0x8A, 0, 0, 0};
//static uint8_t addr_list[8] = {0x80, 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0x8E};
const uint8_t addr_list[8] = {0x80, 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0x8E};

void touch_screen_entry( void *pvParameter )
{
BaseType_t xResult;
uint32_t ulNotifiedValue;
const TickType_t xTicksToWait = pdMS_TO_TICKS( 1000 );

	hmi_init();
	
	for( ;; )
	{
		// 每隔1s告诉wdg_daemon我还活着
	    xTaskNotify( h_wdg_daemon, WDG_BIT_TOUCH_SCREEN, eSetBits );

		/* Wait to be notified of an interrupt. */
        xResult = xTaskNotifyWait( (uint32_t)   pdFALSE, /* Don't clear bits on entry. */
                                   (uint32_t)   ULONG_MAX, /* Clear all bits on exit. */
                                   (uint32_t *) &ulNotifiedValue, /* Stores the notified value. */
                                   (TickType_t) xTicksToWait );
        if( xResult == pdPASS )
        {
            /* A notification was received. See which bits were set. */
            if( ( ulNotifiedValue & TX_IDLE_BIT ) != 0 )
            {
			    /* The TX ISR has set a bit. */
//                prvProcessTx();
            }
            else if( ( ulNotifiedValue & RX_IDLE_BIT ) != 0 )
            {
				process_hmi_rx_frame();
                /* The RX ISR has set a bit. */
//                prvProcessRx();
            }
			else if( ulNotifiedValue & CHGR_NOTIFY_TO_START )
			{
                hmi_start_chgr();
				LOG_DEBUG_APP_1( ".... touch_screen rec to start");
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_TO_STOP )
			{
                hmi_stop_chgr();
				LOG_DEBUG_APP_1( ".... touch_screen rec to stop");
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_START_SUCCESS )
			{
				main_disp_start_success();
				LOG_DEBUG_APP_1( ".... touch_screen rec start success");
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_START_FAILED )
			{
				main_disp_start_failed();
				LOG_DEBUG_APP_1( ".... touch_screen rec start failed");
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_TYPE_SWITCH )
			{
				LOG_DEBUG_APP_1( ".... touch_screen rec chgr type switch");
                main_disp_chgr_type_switch();
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_TYPE_SWITCH_FINISH )
			{
				LOG_DEBUG_APP_1( ".... touch_screen rec chgr type switch finish");
                main_disp_chgr_type_switch_finish();
			}
#if 0
			else if( ulNotifiedValue & SET_NOTIFY_TO_RESTART )
			{
				LOG_TRACE("\r\nsend restart to chagre_entry ... ...");
				set_restart( phmi_info->set.cur_num );
				xTaskNotify( h_charge_entry, SET_NOTIFY_TO_RESTART, eSetBits );
				LOG_TRACE_1(" finish");
			}
			else if( ulNotifiedValue & SET_NOTIFY_TO_RESTART_ALL )
			{ 
				LOG_TRACE("\r\nsend restart all to chagre_entry ... ...");
				set_restart( phmi_info->set.cur_num );
				xTaskNotify( h_charge_entry, SET_NOTIFY_TO_RESTART_ALL, eSetBits );
				LOG_TRACE_1(" finish");
			}
			else if( ulNotifiedValue & SET_NOTIFY_RESTART_FINISHED )
			{
				LOG_TRACE_1("   ....touch_screen rec restart finished");
			    set_restart_finished( phmi_info->set.cur_num );
			}
#endif
			else if( ulNotifiedValue & CHGR_NOTIFY_EMERGENCY_STOP )
			{
				LOG_DEBUG_APP_1("          .... ts, stop");
				// 挂起刷新任务
                suspend_touch_screen_damon();
				// 显示
			    hmi_disp_emergency_stop();
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_START_FROM_EMERGENCY_STOP )
			{
				LOG_DEBUG_APP_1("          .... ts, resume");
				// 恢复刷新任务
                resume_touch_screen_damon();
				// 从急停状态恢复，返回之前页面
			    hmi_disp_start_from_emergency_stop();
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_NO_START ) // 显示无法启动
			{
				hmi_switch_to_spec_page( HMI_NO_START_PAGE_ID );
				LOG_DEBUG_APP_1( "\r\ndisp no start page" );
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_NO_SYN_SET ) // 显示禁止同步设置
			{
				hmi_switch_to_spec_page( HMI_NO_SYN_SETTING_PAGE_ID );
				LOG_DEBUG_APP_1( "\r\ndisp no syn setting page" );
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_NO_SET ) // 显示禁止设置
			{
				hmi_switch_to_spec_page( HMI_NO_SETTING_PAGE_ID );
				LOG_DEBUG_APP_1( "\r\ndisp no setting page" );
			}
        }
        else
        {
            /* Did not receive a notification within the expected time. */
//            prvCheckForErrors();
        }
	}
}

static void suspend_touch_screen_damon(void)
{	
    if( eSuspended != eTaskGetState( xTouchScreenDamonHandle ) )
		vTaskSuspend( xTouchScreenDamonHandle );
}

static void resume_touch_screen_damon(void)
{	
    if( eSuspended == eTaskGetState( xTouchScreenDamonHandle ) )
		vTaskResume( xTouchScreenDamonHandle );
}

static void prvTouchScreenDamonTask( void *pvParameter )
{
BaseType_t xResult;
uint32_t ulNotifiedValue;

uint8_t get_time_cnt = 0;
uint8_t idx = 0;
uint16_t page_id = 0;
uint8_t cnt = 0;

	for( ;; )
	{
        // 每隔50ms，告诉wdg_daemon我还活着
        xTaskNotify( h_wdg_daemon, WDG_BIT_TOUCH_SCREEN_DAEMON, eSetBits );

		/* Wait to be notified of an interrupt. */
        xResult = xTaskNotifyWait( (uint32_t)   pdFALSE, /* Don't clear bits on entry. */
                                   (uint32_t)   ULONG_MAX, /* Clear all bits on exit. */
                                   (uint32_t *) &ulNotifiedValue, /* Stores the notified value. */
                                   (TickType_t) 50 ); // 50ms执行一次
        if( xResult == pdPASS )
        {
            /* A notification was received. See which bits were set. */
            if( ulNotifiedValue & DISP_NOTIFY_FLUSH )
			{
				flush();
				continue;
			}
		}

		// 校验页面，防止页面切换出问题
		if( phmi_info->verify.flag != FALSE )
		{
		    if( phmi_info->page.id.cur_rd != phmi_info->verify.page_id )
			{
				LOG_DEBUG_APP_1("\r\ncur_page:%d, 页面校验失败，重新切换到页面: %d", phmi_info->page.id.cur_rd, phmi_info->verify.page_id);
			    hmi_switch_to_spec_page( phmi_info->verify.page_id );
				hmi_read_cur_page_id();
			}
			else
			{
			    phmi_info->verify.flag = FALSE;
				LOG_DEBUG_APP_1("\r\n页面校验成功，页面ID: %d", phmi_info->verify.page_id);
			}
		}

#if 0
	    vTaskDelay( pdMS_TO_TICKS(50) );

		cnt ++;
		if( cnt >= 20 )
		{
			cnt = 0;
		}
		else
			continue;

		/************ 1s执行一次 ***********/
		// 刷新系统指示灯
		sys_led_indicator_flush();
		
		hmi_main_overview();
		hmi_main_detail();

	    maintain_update_detail();

		// 屏幕超时处理
		phmi_info->timeout ++;
		if( phmi_info->timeout >= TIMEOUT_TOUCH_SCREEN ) // 120 超时，切换到屏保页面
		{
		    phmi_info->timeout = 0;

			if( phmi_info->page.id.cur != HMI_SCREEN_SAVE_PAGE_ID )
			{
				LOG_DEBUG_APP("\r\n屏幕操作超时，切换到屏保页面");
//			    hmi_switch_to_spec_page( HMI_SCREEN_SAVE_PAGE_ID );
                hmi_update_page_id( HMI_SCREEN_SAVE_PAGE_ID );
                hmi_timeout();
			}
		}

		// 获取时间，每5秒获取一次
		get_time_cnt ++;
		if( get_time_cnt > 5 )
		{
			get_time_cnt = 0;	
		    hmi_read_sys_time();
		}
	
		// 读取wifi连接状态
        if( is_wifi_conn() )
		{
            gpio_light_on_led_wifi_conn();
		    hmi_disp_wifi( ON );
		}
		else
		{
            gpio_light_off_led_wifi_conn();
		    hmi_disp_wifi( OFF );
		}
		
		// 显示启动失败 
		for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
		{
		    if( pdev[ idx ]->chgr.flag_start_failed == TRUE )
			{
			    pdev[ idx ]->chgr.cnt_box[ IDX_CNT_START_FAIL_TIME ] ++;
				if( pdev[ idx ]->chgr.cnt_box[ IDX_CNT_START_FAIL_TIME ] >= 2 )
				{
					pdev[ idx ] ->chgr.flag_start_failed = FALSE;
			        pdev[ idx ]->chgr.cnt_box[ IDX_CNT_START_FAIL_TIME ] = 0;

		            LOG_DEBUG_APP("\r\nstart failed finish, switch to pre page:%d", page_id);

					page_id = Op_NumToPageId[ phmi_info->op.start_num ];
					hmi_switch_to_spec_page( page_id );
	                hmi_stop_animate();

	                // 添加页面切换校验机制
                    hmi_verify_page_switch( page_id );
				}
			}
		}	
#endif
	}
}

static void prvSettingDamonTask( void *pvParameter )
{
//BaseType_t xReturn = pdPASS;
BaseType_t xResult;
uint32_t ulNotifiedValue;
const TickType_t xTicksToWait = pdMS_TO_TICKS( 1000 );

    for( ;; )
	{
		// 每隔1s告诉wdg_daemon我还活着
	    xTaskNotify( h_wdg_daemon, WDG_BIT_SETTING, eSetBits );

		/* Wait to be notified of an interrupt. */
        xResult = xTaskNotifyWait( (uint32_t)   pdFALSE, /* Don't clear bits on entry. */
                                   (uint32_t)   ULONG_MAX, /* Clear all bits on exit. */
                                   (uint32_t *) &ulNotifiedValue, /* Stores the notified value. */
                                   (TickType_t) xTicksToWait );
        if( xResult == pdPASS )
        {
            /* A notification was received. See which bits were set. */
            if( ulNotifiedValue & NOTIFY_CHGR_PARA_SETTING )
			{
				// 充电参数设置
				set_chgr_para_setting(); 
			}
			else if( ulNotifiedValue & NOTIFY_RESTORE_FACTORY_SETTING )
			{
			    // 恢复出厂设置
				set_restore_factory_default();	
			}
		}
	}

}

static void sys_led_indicator_flush(void)
{
    uint8_t idx = 0; 
	uint8_t warn_flag = FALSE;
	uint8_t err_flag = FALSE;

	for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
	{
	    if( pdev[idx]->basic.err != BASIC_ERR_NO )
			err_flag = TRUE;
		if( pdev[idx]->basic.warn != BASIC_WARN_NO )
			warn_flag = TRUE;
	}
	
	if( err_flag == TRUE )
	{
		LED_RED_ON();
		LED_GREEN_OFF();
	}
	else if( warn_flag == TRUE )
	{
		LED_RED_TOGGLE();
		LED_GREEN_ON();
	}
	else
	{
		LED_RED_OFF();
		LED_GREEN_ON();
	}
}

static void flush(void)
{
static uint8_t get_time_cnt = 0;
uint8_t idx = 0;
uint16_t page_id = 0;

		/************ 1s执行一次 ***********/
		// 刷新系统指示灯
		sys_led_indicator_flush();
		
		hmi_main_overview();
		hmi_main_detail();

	    maintain_update_detail();

		// 屏幕超时处理
		phmi_info->timeout ++;
		if( phmi_info->timeout >= TIMEOUT_TOUCH_SCREEN ) // 120 超时，切换到屏保页面
		{
		    phmi_info->timeout = 0;

			if( phmi_info->page.id.cur != HMI_SCREEN_SAVE_PAGE_ID )
			{
				LOG_DEBUG_APP("\r\n屏幕操作超时，切换到屏保页面");
//			    hmi_switch_to_spec_page( HMI_SCREEN_SAVE_PAGE_ID );
                hmi_update_page_id( HMI_SCREEN_SAVE_PAGE_ID );
                hmi_timeout();
			}
		}

		// 获取时间，每5秒获取一次
		get_time_cnt ++;
		if( get_time_cnt > 5 )
		{
			get_time_cnt = 0;	
		    hmi_read_sys_time();
		}
	
		// 读取wifi连接状态
        if( is_wifi_conn() )
		{
            gpio_light_on_led_wifi_conn();
		    hmi_disp_wifi( ON );
		}
		else
		{
            gpio_light_off_led_wifi_conn();
		    hmi_disp_wifi( OFF );
		}
		
		// 显示启动失败 
		for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
		{
		    if( pdev[ idx ]->chgr.flag_start_failed == TRUE )
			{
			    pdev[ idx ]->chgr.cnt_box[ IDX_CNT_START_FAIL_TIME ] ++;
				if( pdev[ idx ]->chgr.cnt_box[ IDX_CNT_START_FAIL_TIME ] >= 2 )
				{
					pdev[ idx ] ->chgr.flag_start_failed = FALSE;
			        pdev[ idx ]->chgr.cnt_box[ IDX_CNT_START_FAIL_TIME ] = 0;

		            LOG_DEBUG_APP("\r\nstart failed finish, switch to pre page:%d", page_id);

					page_id = Op_NumToPageId[ phmi_info->op.start_num ];
					hmi_switch_to_spec_page( page_id );
	                hmi_stop_animate();

	                // 添加页面切换校验机制
                    hmi_verify_page_switch( page_id );
				}
			}
		}	
}

static void hmi_read_sys_time( void )
{
	uint8_t idx = 0;
    uint8_t buff[] = { 0x5A, 0xA5, 0x03, 0x81, 0x20, 0x07 };
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void hmi_disp_wifi( uint8_t st )
{
	uint8_t idx = 0;
    uint8_t buff[] = { 0x5A, 0xA5, 0x05, 0x82, 0x00, 0xB9, 0x00, 0x00 };

	// 写0，显示断开，写1，显示连接
	if( st == ON )
	    buff[7] = 0x01;
	else if ( st == OFF )
	    buff[7] = 0x00;
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void set_restart( uint8_t num )
{
	// 启动超时定时器
//    hmi_main_start_timer( ID_SW_TIMER_0 );
	
	// 设置重启 标志位
	if( num != POWER_UNIT_SY )
	    pdev[num]->basic.restart = BASIC_RESTART_YES;	
}

static void set_restart_finished( uint8_t num )
{
	// 停止超时定时器
//	hmi_main_stop_timer( ID_SW_TIMER_0 );
	
    // 清除重启状态
	if( num != POWER_UNIT_SY )
	    pdev[num]->basic.restart = BASIC_RESTART_NO;
	
	// 返回对应模块设置的页面
	hmi_switch_to_spec_page( Set_NumToPageId[num] );
	
	// 停止显示动画
	hmi_stop_animate();
}

static void set_chgr_para_setting( void )
{
	// 表示处于设置中的界面
	phmi_info->flag_setting = TRUE;

    // 显示动画
    hmi_start_animate();
				
	// 执行保存参数的操作
	LOG_TRACE("\r\nsave chgr para   ........");

	if( phmi_info->set.chgr_type == CHGR_TYPE_THREE_SECTION )
	{
	    LOG_TRACE_1("\r\n三段式参数设置:");
        // 进入的是同步模式的三段式曲线设置页面
        // 同步模式的参数
	    if( phmi_info->set.cur_num == POWER_UNIT_SY ) 
	    {
	    	LOG_TRACE_1("\r\n同步模式");
	    	// 打开所有模块的AC输入
            power_on_all();
	    	vTaskDelay( pdMS_TO_TICKS( POWER_ON_MS_DELAY ) );
	    	LOG_TRACE_1("\r\nopen all, after 3s");

	    	// 保存参数
	        set_save_chgr_para_syn();

	    	// delay 2s
	    	vTaskDelay( pdMS_TO_TICKS(2000) );
	    	LOG_TRACE_1("\r\ndelay 2s finish");

	    	// 关闭所有模块
	    	power_off_all();
	    	vTaskDelay( pdMS_TO_TICKS( POWER_OFF_MS_DELAY ) );

	    	LOG_TRACE_1("\r\nclose all, after 8s");
	    } 
	    else // 进入的是单模式的三段式曲线设置页面
	    {
	    	LOG_TRACE_1("\r\n单个模式");
	    	// 打开对应模块的AC输入
	    	power_on( phmi_info->set.cur_num );
	    	vTaskDelay( pdMS_TO_TICKS( POWER_ON_MS_DELAY ) );
	    	LOG_TRACE_1("\r\nopen, after 3s");

	    	// 保存参数
	    	set_save_chgr_para( phmi_info->set.cur_num );

	    	// delay 2s
	    	vTaskDelay( pdMS_TO_TICKS(2000) );
	    	LOG_TRACE_1("\r\ndelay 2s finish");

	    	// 关闭对应模块
	    	power_off( phmi_info->set.cur_num );
	    	vTaskDelay( pdMS_TO_TICKS( POWER_OFF_MS_DELAY ) );
	    	LOG_TRACE_1("\r\nclose after 8s");
	    }
    }
	else if ( phmi_info->set.chgr_type == CHGR_TYPE_PRE )
	{
	    LOG_TRACE_1("\r\n预充电参数设置:");
        // 同步模式的参数
	    if( phmi_info->set.cur_num == POWER_UNIT_SY ) 
		{
	    	LOG_TRACE_1("\r\n同步模式");
            set_save_pre_chgr_para_syn();
		}
	    else // 进入的是单模式的三段式曲线设置页面
		{
	    	LOG_TRACE_1("\r\n单个模式");
            set_save_pre_chgr_para( phmi_info->set.cur_num );
		}
	}
	else if ( phmi_info->set.chgr_type == CHGR_TYPE_AUTO )
	{
	    // 保存设定的参数
	    set_chgr_threshold_set_finish( phmi_info->set.cur_num );

    	// 切换到对应设置的主界面
//        hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );

		if( phmi_info->set.cur_num == POWER_UNIT_SY ) // 进入的是同步模式的设置
		    LOG_DEBUG_APP("\r\n充电阈值设置完成, 同步设置，阈值:%d%%", pdev[0]->chgr.para.data_auto.threshold );
		else
			LOG_DEBUG_APP("\r\n充电阈值设置完成, num:%d, 阈值:%d%%", phmi_info->set.cur_num, pdev[phmi_info->set.cur_num]->chgr.para.data_auto.threshold );
	}

    // 保存参数结束
    set_save_chgr_para_finished( phmi_info->set.cur_num );

	// 表示设置完成
	phmi_info->flag_setting = FALSE;

    LOG_TRACE_1("    ....finish");
}

static void set_save_chgr_para_finished( uint8_t num )
{
	// 返回对应模块设置的页面
	hmi_switch_to_spec_page( Set_NumToPageId[num] );
	// 添加切换校验
	hmi_verify_page_switch( Set_NumToPageId[num] );
	LOG_DEBUG_APP_1("\r\n开始校验页面切换，页面ID:%d", Set_NumToPageId[num] );
	
	// 停止显示动画
	hmi_stop_animate();	
}

static void hmi_screen_save(void)
{
	uint16_t var_addr = phmi_rx_frame->data.addr;
//	uint16_t key_value = phmi_rx_frame->data.data[0];
	
	switch( var_addr )
	{
	    case 0x005:
		break;
	}
}

static void hmi_main(void)
{
	uint16_t var_addr = phmi_rx_frame->data.addr;
	uint16_t key_value = phmi_rx_frame->data.data[0];
	
	switch( var_addr )
	{
		case 0x0011:
			if( key_value == 0x0001 )  // 表示切换到了主页-预览
			{
			    hmi_switch_to_spec_page( HMI_MAIN_PAGE_ID );
                hmi_main_overview();
			}
			break;
		case 0x0012:
			if( key_value == 0x0001 )  // 模块1的详细信息
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_1_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // 表示切换电池类型
			{
                exec_bat_type_switch( POWER_UNIT_1 );
			}
			else if( key_value == 0x0003 )  // 表示按下了启动/停止按钮
			{
				exec_chgr_op_switch( POWER_UNIT_1 );
			}
			break;
		case 0x0013:
			if( key_value == 0x0001 )  // 模块2的详细信息
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_2_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // 表示切换电池类型
			{
                exec_bat_type_switch( POWER_UNIT_2 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_2 );
			}
			break;
		case 0x0014:
			if( key_value == 0x0001 )  // 模块3的详细信息
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_3_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // 表示切换电池类型
			{
                exec_bat_type_switch( POWER_UNIT_3 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_3 );
			}
			break;
		case 0x0015:
			if( key_value == 0x0001 )  // 模块4的详细信息
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_4_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // 表示切换电池类型
			{
                exec_bat_type_switch( POWER_UNIT_4 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_4 );
			}
			break;
		case 0x0016:
			if( key_value == 0x0001 )  // 模块5的详细信息
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_5_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // 表示切换电池类型
			{
                exec_bat_type_switch( POWER_UNIT_5 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_5 );
			}
			break;
		case 0x0017:
			if( key_value == 0x0001 )  // 模块6的详细信息
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_6_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // 表示切换电池类型
			{
                exec_bat_type_switch( POWER_UNIT_6 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_6 );
			}
			break;
		case 0x0018:
			if( key_value == 0x0001 )  // 模块7的详细信息
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_7_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // 表示切换电池类型
			{
                exec_bat_type_switch( POWER_UNIT_7 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_7 );
			}
			break;
		case 0x0019:
			if( key_value == 0x0001 )  // 模块8的详细信息
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_8_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // 表示切换电池类型
			{
                exec_bat_type_switch( POWER_UNIT_8 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_8 );
			}
			break;
		case 0x001A:
			// 切换到之前的页面
			hmi_switch_to_spec_page( Op_NumToPageId[phmi_info->op.start_num] );
//			hmi_switch_to_spec_page( phmi_info->page.id.pre );
			break;
		default:
			break;
	}
}

static void hmi_main_overview(void)
{
	uint8_t idx = 0;
	
	// 如果不在main_overview的页面，直接退出，不刷新数据
    if( phmi_info->page.id.cur != HMI_MAIN_OVERVIEW_ID ) { return; }	
	
	for( idx = 0; idx < POWER_UNIT_NUM; idx ++)
	{
		// 1. 判断有无异常
	    if( (pdev[idx]->basic.err == BASIC_ERR_NO) && (pdev[idx]->basic.warn == BASIC_WARN_NO) )
		{
	        main_disp_led( LED_GREEN, idx );
		}
	    else
		{
		    main_disp_led( LED_RED, idx );
		}
	
	    // 2. 显示电池状态
	    main_disp_bat( pdev[idx]->basic.bat, idx );
	
	    // 3. 显示电池电压和电流
	    main_disp_bat_volt( pdev[idx]->basic.bat_volt, idx ); // 主界面修改为显示电池电压
	    main_disp_iout( pdev[idx]->chgr.iout_disp, idx );	

	    //4.更新模式显示
	    main_disp_mode( pdev[idx]->chgr.mode, idx );
	}
	LOG_DEBUG_APP("\r\nmain_overview flush");
}

static void hmi_main_detail(void)
{
	uint8_t num = 0;
	
	// 如果不在模块1-8的详细信息页面，直接退出，不刷新数据
    if( (phmi_info->page.id.cur < HMI_MAIN_DETAIL_1_ID) |
        (phmi_info->page.id.cur > HMI_MAIN_DETAIL_8_ID) )
    { return; }
	
	// 确定刷新哪个模块的数据
	switch( phmi_info->page.id.cur )
	{
		case HMI_MAIN_DETAIL_1_ID:
			num = POWER_UNIT_1;
			break;
		case HMI_MAIN_DETAIL_2_ID:
			num = POWER_UNIT_2;
			break;
		case HMI_MAIN_DETAIL_3_ID:
			num = POWER_UNIT_3;
			break;
		case HMI_MAIN_DETAIL_4_ID:
			num = POWER_UNIT_4;
			break;
		case HMI_MAIN_DETAIL_5_ID:
			num = POWER_UNIT_5;
			break;
		case HMI_MAIN_DETAIL_6_ID:
			num = POWER_UNIT_6;
			break;
		case HMI_MAIN_DETAIL_7_ID:
			num = POWER_UNIT_7;
			break;
		case HMI_MAIN_DETAIL_8_ID:
			num = POWER_UNIT_8;
			break;
	    default:
			break;
	}
	
	//1.显示充电状态
	main_disp_chgr_status( pdev[num]->basic.err, num );
	//2.显示充电电压
	main_disp_vout( pdev[num]->chgr.vout, num );
	//3.显示充电电流
	main_disp_iout( pdev[num]->chgr.iout_disp, num );
	//4.显示充电时间
	main_disp_chgr_time( pdev[num]->chgr.time, num );
	//5.显示模块温度
	main_disp_temperature( pdev[num]->basic.temp, num );
	//6.显示已充电量
	main_disp_AH( pdev[num]->chgr.AH, num );
	//7.显示输出电压错误
	main_disp_DC_OK( pdev[num]->basic.err, num );
	
	//8.更新操作显示
	  // 如果处于自动模式，隐藏启动和停止图标
	if( pdev[num]->chgr.mode == MODE_AUTO )
	{
		main_disp_op( OP_DISP_NONE, num );
	}
	else if( pdev[num]->chgr.mode == MODE_MANUAL )
	{
	    if( pdev[num]->chgr.op == op_start )
	    {
	    	main_disp_op( OP_DISP_STOP, num );
	    }
	    else if( pdev[num]->chgr.op == op_stop )
	    {
	    	main_disp_op( OP_DISP_START, num );
	    }
	}

	//9.显示电池信息
    main_disp_bat_info( num );

	// 10.显示电池类型
	  // 如果处于自动模式，隐藏启动和停止图标
	if( pdev[num]->chgr.mode == MODE_AUTO )
	{
		main_disp_bat_type( 0xff, num );
	}
	else
		main_disp_bat_type( pdev[num]->chgr.bat_type, num );


	// 11.显示切换图标，处于自动模式隐藏
	if( pdev[num]->chgr.mode == MODE_AUTO )
	{
		main_disp_bat_switch_ico( 0, num );
	}
	else
		main_disp_bat_switch_ico( 1, num );

	LOG_DEBUG_APP("\r\nmain_detail flush, num:%d", num);
}

static void main_disp_bat_info( uint8_t num )
{
	// 电池电压，电池电流，剩余容量，电池温度，需求电流，满电标志
    uint16_t addr_list[] = {0x0821, 0x0822, 0x0823, 0x0824, 0x0826, 0x0827, 0x0832};
	uint8_t warn_buff[22] = {0}; //实际长度19个字符+1('\0')
	uint16_t data[6] = {0};
	uint8_t idx = 0;
	
	if( pdev[num]->chgr.mode == (uint8_t)MODE_MANUAL  )
	{
	    memset(data, 0, (sizeof(data[0])*5) );

		// 手动模式，电池电压显示为采样的电池电压，电池电流显示为充电电流
		data[0] = pdev[num]->basic.bat_volt * 10; 
		data[1] = pdev[num]->chgr.iout_disp * 10;
	}
	else if( pdev[num]->chgr.mode == (uint8_t)MODE_AUTO  )
	{
	    data[0] = pdev[num]->chgr.auto_bat_volt * 10;
		data[1] = pdev[num]->chgr.auto_bat_iout * 10;
		data[2] = ( (float)pdev[num]->chgr.auto_bat_cap) * 10;
		data[3] = pdev[num]->chgr.auto_bat_temp;
		data[4] = pdev[num]->bat_info_bms.limit_curr;
		data[5] = pdev[num]->bat_info_bms.flag_full;
	}
	
	for( idx = 0; idx < 6; idx ++ )
	{
		hmi_disp_para_data( addr_list[idx], data[idx] );
	}

	// 显示告警字
	sprintf( (char *)warn_buff, "0x%02x 0x%02x 0x%02x 0x%02x", pdev[num]->bat_info_bms.warn_1, pdev[num]->bat_info_bms.warn_2, pdev[num]->bat_info_bms.warn_3, pdev[num]->bat_info_bms.warn_4);
	hmi_disp_string( addr_list[6], warn_buff, 19 );
}

static void main_disp_bat_type( uint8_t to, uint8_t num )
{
	uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x31, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	// 显示电池类型图标变量地址依次为0x0813,0x0913...0x0A13
	buff[4] = addr_h[ num ];
	
	// 写0，显示铅酸电池，写1，显示锂电池
	if( to == CHGR_BAT_TYPE_LI ) 
	    buff[7] = 0x01;
	else if( to == CHGR_BAT_TYPE_LEAD_ACID )  
		buff[7] = 0x00;
	else
		buff[7] = 0x02;

	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );

	// 发送出去
	com_touch_screen->send();

}

static void main_disp_bat_switch_ico( uint8_t to, uint8_t num )
{
	uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x30, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	// 显示电池类型图标变量地址依次为0x0813,0x0913...0x0A13
	buff[4] = addr_h[ num ];
	
	// 写0，显示切换图标，写1，隐藏切换图标
	if( to == 1 ) 
	    buff[7] = 0x00;
	else if( to == 0 )  
		buff[7] = 0x01;

	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );

	// 发送出去
	com_touch_screen->send();

}

static void exec_chgr_op_switch( uint8_t num )
{
	if( pdev[num]->chgr.op == op_start )      // 之前执行了启动操作，那么本次执行的就是停止操作，界面显示启动
	{
		LOG_DEBUG_APP("\r\nexec stop, stop num:%d .... .... ", num);

		// 记录下是哪个模块按下了停止
		phmi_info->op.stop_num  = num;

		// 执行停止操作
		pub_stop( STOP_TYPE_HMI, num );
	} 
	else if( pdev[num]->chgr.op == op_stop )  // 之前执行了停止操作，那么本次执行的就是启动操作，界面显示停止
	{
		LOG_DEBUG_APP("\r\nexec start, start num:%d .... .... ", num);

		// 记录下是哪个模块按下了启动
		phmi_info->op.start_num = num;

		// 执行启动操作
		pub_start( START_TYPE_HMI, num );
	}
}

static void exec_bat_type_switch( uint8_t num )
{
	if( pdev[num]->chgr.bat_type == CHGR_BAT_TYPE_LEAD_ACID )
	{
    	pub_bat_type_switch( num, CHGR_BAT_TYPE_LI );
	}
	else if( pdev[num]->chgr.bat_type == CHGR_BAT_TYPE_LI )
	{
    	pub_bat_type_switch( num, CHGR_BAT_TYPE_LEAD_ACID );
	}
}

static void hmi_update_page_id( uint16_t new_id )
{
	LOG_DEBUG_APP("\r\nupdate page id, id_pre:%d, id_cur:%d, id_sw_to:%d", phmi_info->page.id.pre, phmi_info->page.id.cur, new_id );
	phmi_info->page.id.pre = phmi_info->page.id.cur;
	phmi_info->page.id.cur = new_id;
}

static void hmi_switch_to_spec_page( uint16_t page_id )
{
	uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x04, 0x80, 0x03, 0x00, 0x00};
	
	// 页面过滤
	if( page_id > HMI_MAX_PAGE_ID ) return;
	
	buff[5] = ( ( page_id >> 8 ) & 0xff );
	buff[6] = ( page_id & 0xff );
	
//taskENTER_CRITICAL();
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();

//taskEXIT_CRITICAL();
	
	hmi_update_page_id( page_id ); // 20181219修改为切换后，立即更新页面id
}

static void hmi_verify_page_switch( uint16_t page_id )
{
	/* 添加页面切换校验机制
	 * 1.设置校验的页面id值
	 * 2.发送读页面的指令
	 * 3.在 prvTouchScreenDamonTask 里面，判断读取页面是否正确
	 *
	 */
	phmi_info->verify.page_id = page_id;
	phmi_info->verify.flag = TRUE;
	hmi_read_cur_page_id();
}

static void hmi_timeout( void )
{
	hmi_reset();
}

static void hmi_reset( void )
{
    uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x04, 0x80, 0xEE, 0x5A, 0xA5}; 
	
	// 清空记录的页面id信息
	phmi_info->page.id.pre = 0;
	phmi_info->page.id.cur = 0;
	phmi_info->page.id.cur_rd = 0;

	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void hmi_start_animate( void )
{
    uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x10, 0xA5, 0x00, 0x01}; 
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void hmi_stop_animate( void )
{
    uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x10, 0xA5, 0x00, 0x00}; 
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void hmi_disp_emergency_stop( void )
{
    // 读取当前页面id
	hmi_read_cur_page_id();
	delay_ms( 20 );
	phmi_info->page.id.pre = phmi_info->page.id.cur_rd;
	hmi_switch_to_spec_page( HMI_MAIN_EMERGENCY_STOP_ID );
}	

static void hmi_disp_start_from_emergency_stop( void )
{
    uint8_t num = 0;
	/* 处理频繁按下紧急停止导致界面切换出错的问题
	 *   - 如果出错，导致之前的页面是紧急停止页面，那么就切换到屏保页面
	 */
	if( phmi_info->page.id.pre == HMI_MAIN_EMERGENCY_STOP_ID )
	{
	    hmi_switch_to_spec_page( HMI_SCREEN_SAVE_PAGE_ID );
	}
	else
	{
		num = phmi_info->page.id.pre;
	    // 切换到之前的页面
        hmi_switch_to_spec_page( num );
//		
//		// 添加切换校验
//		hmi_verify_page_switch( num );
	}
}

static void hmi_read_cur_page_id(void)
{
	uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x03, 0x81, 0x03, 0x02};
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void hmi_start_chgr( void )
{
}

static void hmi_stop_chgr( void )
{
	uint8_t idx = 0;

    // 界面显示停止中
//    hmi_switch_to_spec_page( HMI_MAIN_STOP_ING_ID ); hmi_start_animate();

	for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
	{
	    if( pdev[ idx ]->chgr.flag_stop == TRUE )
		{
			pdev[ idx ] ->chgr.flag_stop = FALSE;
	        LOG_DEBUG_APP("\r\nstop finish, switch to pre page:%d", phmi_info->page.id.pre);
	        main_disp_op( OP_DISP_START, idx );
		}
	}

	// flush detail
    hmi_main_detail();
}

void get_sys_time( Sys_TimeDef *ptime )
{
	*ptime = phmi_info->sys_time;
}

static void main_disp_bat( uint8_t st, uint8_t num )
{
	uint8_t idx;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x11, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	static uint8_t val_chg_ing[8] = {0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03};
	
	// 电池图标的变量地址依次为0811,0911,...0F11
	buff[4] = addr_h[ num ];
	
	/* 向变量地址写0，显示未连接，
	            写1，表示已连接,
	            写2，表示反接，
	            依次写3、4、5、6、7依次显示充电中的图标，写8，显示充满图标 // 20190911，修改充满图标为单独的图标
	 */
	if( st & BASIC_BAT_DISCONN )
	    buff[7] = 0x00;
	else if( st & BASIC_BAT_CONN )
		buff[7] = 0x01;
	else if( st & BASIC_BAT_REVERSE )
		buff[7] = 0x02;
	else if( st & BASIC_BAT_CHGR_ING )
	{
		val_chg_ing[num] += 1;
		if( val_chg_ing[num] > 0x07 )
			val_chg_ing[num] = 0x03;
		
		buff[7] = val_chg_ing[num];
	}
	else if( st & BASIC_BAT_FULL )
		buff[7] = 0x08;

	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void main_disp_led( uint8_t st, uint8_t num )
{
	uint8_t idx;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x12, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	// led的变量地址依次为0812,0912,...0F12
	buff[4] = addr_h[ num ];
	
	// 向变量地址写0，显示绿灯，写1，显示红灯
	if( st & LED_GREEN )
	    buff[7] = 0x00;
	else if(  st & LED_RED )
		buff[7] = 0x01;
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void main_disp_vout( float fvalue, uint8_t num )
{
    uint8_t idx;
	int16_t uwValue;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x19, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	buff[4] = addr_h[ num ];
	
	// 数据格式转换，整数3位，小数1位，比如屏幕需要显示56.7，那么需要发送567，把56.7转换成567，然后取高8位和低8位
	uwValue = fvalue * 10;
	
	buff[6] = (int8_t)((uwValue>>8) & 0xFF);
	buff[7] = (int8_t)(uwValue & 0xFF);
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void main_disp_bat_volt( float fvalue, uint8_t num )
{
    uint8_t idx;
	int16_t uwValue;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x26, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	buff[4] = addr_h[ num ];
	
	// 数据格式转换，整数3位，小数1位，比如屏幕需要显示56.7，那么需要发送567，把56.7转换成567，然后取高8位和低8位
	uwValue = fvalue * 10;
	
	buff[6] = (int8_t)((uwValue>>8) & 0xFF);
	buff[7] = (int8_t)(uwValue & 0xFF);
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void main_disp_iout( float fvalue, uint8_t num )
{
    uint8_t idx;
	uint16_t uwValue;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x1A, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	buff[4] = addr_h[ num ];
	
	// 数据格式转换，比如把56.7转换成567，然后取高8位和低8位
	uwValue = fvalue * 10;
	
	buff[6] = (uint8_t)((uwValue>>8) & 0xFF);
	buff[7] = (uint8_t)(uwValue & 0xFF);
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void main_disp_chgr_status( uint8_t st, uint8_t num )
{
	uint8_t idx;
    uint8_t buff[] = {0x5A, 0xA5, 0x07, 0x82, 0x08, 0x15};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	char *str1 = "正常";
	char *str2 = "异常";
	
	buff[4] = addr_h[ num ];
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	if( st == BASIC_ERR_NO )
	{
	    while( *str1 != '\0' )
			com_touch_screen->write( *str1++ );
	}
	else if( st != BASIC_ERR_NO )
	{
	    while( *str2 != '\0' )
			com_touch_screen->write( *str2++ );
	}
	
	// 发送出去
	com_touch_screen->send();
}

static void main_disp_chgr_time( uint16_t to, uint8_t num )
{
    uint8_t idx;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x1B, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	buff[4] = addr_h[ num ];
	
	buff[6] = (uint8_t)((to>>8) & 0xFF);
	buff[7] = (uint8_t)(to & 0xFF);
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void main_disp_temperature( int8_t to, uint8_t num )
{
    uint8_t idx;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x1C, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	buff[4] = addr_h[ num ];
	
//	buff[6] = (uint8_t)((to>>8) & 0xFF);
	if( to >= 127 ) to = 127;
	else if ( to <= -128) to = -128;
	
	buff[7] = (int8_t)(to & 0xFF);
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void main_disp_AH( uint16_t to, uint8_t num )
{
    uint8_t idx;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x1D, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	buff[4] = addr_h[ num ];
	
	buff[6] = (uint8_t)((to>>8) & 0xFF);
	buff[7] = (uint8_t)(to & 0xFF);
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void main_disp_DC_OK( uint8_t st, uint8_t num )
{
	uint8_t idx;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x1F};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	char *str1 = "是";
	char *str2 = "否";
	char clr[4] = {' ', ' ', ' ', ' '};
	
	buff[4] = addr_h[ num ];
	
	// 清空显示区域
	idx = 0;
	while( idx < 4)
		com_touch_screen->write( clr[idx++] );
	com_touch_screen->send();
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	if( st & bit( ERR_DC_OK ) )
	{
	    while( *str1 != '\0' )
			com_touch_screen->write( *str1++ );
	}
	else
	{
	    while( *str2 != '\0' )
			com_touch_screen->write( *str2++ );
	}
	
	// 发送出去
	com_touch_screen->send();
}

static void main_disp_mode( uint8_t to, uint8_t num )
{
	uint8_t idx = 0;
	
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x13, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	// 显示手动/自动的图标变量地址依次为0x0813,0x0913...0x0A13
	buff[4] = addr_h[ num ];
	
	// 向变量地址写1，显示手动，写0，显示自动
	if( to == MODE_MANUAL )
	    buff[7] = 0x01;
	else if( to == MODE_AUTO )
		buff[7] = 0x00;

	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );

	// 发送出去
	com_touch_screen->send();
}

static void main_disp_op( uint8_t to, uint8_t num )
{
	uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x14, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	// 显示停止/启动的图标变量地址依次为0x0814,0x0914...0x0A14
	buff[4] = addr_h[ num ];
	
	// 向变量地址写1，显示启动，写0，显示停止
	if( to == OP_DISP_START ) 
	    buff[7] = 0x01;
	else if( to == OP_DISP_STOP )  
		buff[7] = 0x00;
	else if( to == OP_DISP_NONE )  
		buff[7] = 0x02;


	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );

	// 发送出去
	com_touch_screen->send();
}

static void main_disp_starting( void )
{
	/* 显示启动中：
	 * 1.切换到启动中页面
	 * 2.更新页面id
     * 3.开启动画	
	 */
    hmi_switch_to_spec_page( HMI_MAIN_STARTING_ID );
	hmi_start_animate();
}

static void main_disp_start_success( void )
{
    /* 启动成功后:
     * 1. 显示停止
     * 2. 返回之前的页面，并更新页面id
	 * 3. 关闭动画
     */
    main_disp_op( OP_DISP_STOP, phmi_info->op.start_num );
}

static void main_disp_start_failed( void )
{
    /* 启动失败后:
	 * 1. 显示启动图标
	 * 2. 显示 启动失败 页面，并更新页面id
	 * 3. 持续2s
	 * 4. 返回之前的页面，并更新页面id
	 * 5. 关闭动画
	 */
	main_disp_op( OP_DISP_START, phmi_info->op.stop_num );
}

static void main_disp_chgr_type_switch(void)
{
	hmi_switch_to_spec_page( HMI_MAIN_TYPE_SWITCH_ID );
}

static void main_disp_chgr_type_switch_finish(void)
{
	hmi_switch_to_spec_page( phmi_info->page.id.pre );
}

static void set_enter_detail_setting( uint8_t num )
{
	// 判断该模块是否在充电，如果在，那么禁止跳转到详细参数设置页面
    if( pdev[num]->basic.chgr != BASIC_CHGR_OFF )
	{
	    hmi_switch_to_spec_page( HMI_NO_SETTING_PAGE_ID );
	}
	// 判断该模块是否处于自动充电的模式，如果是，那么禁止跳转到详细参数设置页面
    if( pdev[phmi_info->set.cur_num]->chgr.mode == MODE_AUTO )
	{
	    hmi_switch_to_spec_page( HMI_NO_SETTING_PAGE_ID );
	}
}

static void set_enter_syn_detail_settting(void)
{
	uint8_t idx = 0;
    //  判断是否有在充电的模块，如果有，那么不进入详细参数设置页面
	for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
	{
        //  判断是否有在充电的模块，如果有，那么不进入详细参数设置页面
	    if( pdev[idx]->basic.chgr != BASIC_CHGR_OFF )
		{
            hmi_switch_to_spec_page( HMI_NO_SYN_SETTING_PAGE_ID );
		}
	    // 判断是否有模块处于自动充电的模式，如果是，那么禁止跳转到详细参数设置页面
	    if( pdev[idx]->chgr.mode == MODE_AUTO )
		{
            hmi_switch_to_spec_page( HMI_NO_SYN_SETTING_PAGE_ID );
		}
	}
}

static void hmi_set( void )
{
	uint16_t addr = phmi_rx_frame->data.addr;
	uint16_t value = phmi_rx_frame->data.data[0];
	uint8_t idx = 0;
	
//	if( addr == 0x0022 ) // 说明是按键
	switch( addr )
	{
		case 0x0022:
			if( value == 0x0002 )  // 表示切换到设置，密码输入完成，准备进入设置里面，此时判断输入的是否正确
			{
			    set_enter();
			}
			else if( value == 0x0012 )
			{
				set_enter_syn_detail_settting();
			}
			else if( (value == 0x0011) )
			{
				hmi_update_page_id( HMI_SET_ENTER_ID );
			}
			else if(value == 0x0021)    // 表示切换到了：充电设置-模块1设置
			{
				phmi_info->set.cur_num = POWER_UNIT_1;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0032 )  // 表示切换到了：充电设置-模块2设置
			{
			    phmi_info->set.cur_num = POWER_UNIT_2;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0043 )  // 表示切换到了：充电设置-模块3设置
			{
			    phmi_info->set.cur_num = POWER_UNIT_3;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0054 )  // 表示切换到了：充电设置-模块4设置
			{
			    phmi_info->set.cur_num = POWER_UNIT_4;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0065 )  // 表示切换到了：充电设置-模块5设置
			{
			    phmi_info->set.cur_num = POWER_UNIT_5;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0076 )  // 表示切换到了：充电设置-模块6设置
			{
			    phmi_info->set.cur_num = POWER_UNIT_6;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0087 )  // 表示切换到了：充电设置-模块7设置
			{
			    phmi_info->set.cur_num = POWER_UNIT_7;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0098 )  // 表示切换到了：充电设置-模块8设置
			{
			    phmi_info->set.cur_num = POWER_UNIT_8;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x00A9 )  // 表示切换到了：充电设置-同步设置
			{
			    phmi_info->set.cur_num = POWER_UNIT_SY;
			}	
			else if( ( value == 0x0022 ) || \
                     ( value == 0x0033 ) || \
                     ( value == 0x0044 ) || \
                     ( value == 0x0055 ) || \
                     ( value == 0x0066 ) || \
                     ( value == 0x0077 ) || \
                     ( value == 0x0088 ) || \
                     ( value == 0x0099 ) || \
					 ( value == 0x00AA ) )  // 进入三段式曲线设置页面
			{
				phmi_info->set.chgr_type = CHGR_TYPE_THREE_SECTION;  // 标记，当前设置方式为三段式

				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // 进入的是同步模式的三段式曲线设置页面
				{
				    //  判断是否有在充电的模块，如果有，那么不进入详细参数设置页面
					for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
					{
				        //  判断是否有在充电的模块，如果有，那么不进入详细参数设置页面
					    if( pdev[idx]->basic.chgr != BASIC_CHGR_OFF )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}
					    // 判断是否有模块处于自动充电的模式，如果是，那么禁止跳转到详细参数设置页面
					    if( pdev[idx]->chgr.mode == MODE_AUTO )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}
					}

					// 对于同步模式，与模块1看齐
	                set_update_temp_curve_data( POWER_UNIT_1, pdev[ POWER_UNIT_1 ]->chgr.para.curve );
				    set_disp_chgr_set( POWER_UNIT_1 );
				}
				else  // 进入的是1-8模块，单独的参数设置
				{	
    				// 判断该模块是否在充电，如果在，那么禁止跳转到详细参数设置页面
    			    if( pdev[phmi_info->set.cur_num]->basic.chgr != BASIC_CHGR_OFF )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}
					// 判断该模块是否处于自动充电的模式，如果是，那么禁止跳转到详细参数设置页面
    			    if( pdev[phmi_info->set.cur_num]->chgr.mode == MODE_AUTO )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}
    				
    	            set_update_temp_curve_data( phmi_info->set.cur_num, pdev[ phmi_info->set.cur_num ]->chgr.para.curve );
    				set_disp_chgr_set( phmi_info->set.cur_num );
				}
			}
			else if( value == 0x00BA )  // 显示曲线一
			{
				LOG_TRACE_1("\r\n曲线一");
				set_update_temp_curve_data( phmi_info->set.cur_num, CHGR_PARA_CURVE_1 );
			    set_disp_curve( CHGR_PARA_CURVE_1 );
			}
			else if( value == 0x00CB )  // 显示曲线二
			{
				set_update_temp_curve_data( phmi_info->set.cur_num, CHGR_PARA_CURVE_2 );
			    set_disp_curve( CHGR_PARA_CURVE_2 );
			}
			else if( value == 0x00DC )  // 显示曲线三
			{
				set_update_temp_curve_data( phmi_info->set.cur_num, CHGR_PARA_CURVE_3 );
			    set_disp_curve( CHGR_PARA_CURVE_3 );
			}
			else if( value == 0x00ED )  // 显示曲线四
			{
				set_update_temp_curve_data( phmi_info->set.cur_num, CHGR_PARA_CURVE_4 );
			    set_disp_curve( CHGR_PARA_CURVE_4 );
			}
			else if( value == 0x00FE )  // 显示更改参数
			{
				set_update_temp_curve_data( phmi_info->set.cur_num, CHGR_PARA_CURVE_U );
			    set_disp_curve( CHGR_PARA_CURVE_U );
			}
			else if( ( value == 0x00BB ) || \
                     ( value == 0x00CC ) || \
                     ( value == 0x00DD ) || \
                     ( value == 0x00EE ) || \
                     ( value == 0x00FF ) ) // 表示在充电曲线页面，按了返回，那么返回到之前的页面
			{
			    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num ] );
			}
			else if( ( value == 0x00BC ) || \
				     ( value == 0x00CD ) || \
                     ( value == 0x00DE ) || \
                     ( value == 0x00EF ) || \
                     ( value == 0x0100 ) || \
                     ( value == 0x00FF ) )  // 表示已经参数设置完毕，确认生效
			{
				// 向设置任务发送设置参数通知
		        xTaskNotify( xSettingDamonHandle, NOTIFY_CHGR_PARA_SETTING, eSetBits);
			}
			else if( ( value == 0x0023 ) || \
                     ( value == 0x0034 ) || \
                     ( value == 0x0045 ) || \
                     ( value == 0x0056 ) || \
                     ( value == 0x0067 ) || \
                     ( value == 0x0078 ) || \
                     ( value == 0x0089 ) || \
                     ( value == 0x009A ) || \
                     ( value == 0x00AB ) )  // 进入了充电时长设置
			{
				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // 进入的是同步模式的设置
				{
					for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
					{
				        //  判断是否有在充电的模块，如果有，那么不进入
					    if( pdev[idx]->basic.chgr != BASIC_CHGR_OFF )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}

					    // 判断是否有模块处于自动充电的模式，如果是，那么禁止跳转到详细参数设置页面
					    if( pdev[idx]->chgr.mode == MODE_AUTO )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}
					}
				}
				else  // 进入的是1-8模块，单独的设置
				{	
    				// 判断该模块是否在充电，如果在，那么禁止进入
    			    if( pdev[phmi_info->set.cur_num]->basic.chgr != BASIC_CHGR_OFF )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}

					// 判断该模块是否处于自动充电的模式，如果是，那么禁止跳转到详细参数设置页面
    			    if( pdev[phmi_info->set.cur_num]->chgr.mode == MODE_AUTO )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}
				}

				// 更新临时参数
				set_update_temp_chgr_timing_time( phmi_info->set.cur_num );
				// 显示对应页面
                set_disp_chgr_timing_time_set( phmi_info->set.cur_num );
                // 显示对应的定时充电时间
                set_disp_chgr_timing_time( phmi_info->set.tmp_chgr_timing_time );
			}
			else if ( ( value == 0x0120 ) | \
					  ( value == 0x0122) )  // 表示按了返回，返回到对应模块的充电设置主页面 
			{
			    hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );
			}
			else if ( value == 0x011F )  // 表示开启了定时充电功能
			{
                // 显示对应的定时充电时间
                set_disp_chgr_timing_time( phmi_info->set.tmp_chgr_timing_time );
				LOG_DEBUG_APP("\r\n开启定时充电");
			}
			else if ( value == 0x0121 )  // 表示开启了定时充电时间功能，并且设定完成，确认
			{
			    // 保存设定的参数
			    set_chgr_timing_time_set_finish( phmi_info->set.cur_num );

            	// 切换到对应设置的主界面
                hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );

				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // 进入的是同步模式的设置
				    LOG_DEBUG_APP("\r\n定时充电设置完成, 同步设置, 定时时间:%d，定时充电状态：%02X", pdev[0]->chgr.timing_time, pdev[phmi_info->set.cur_num]->chgr.st_timing_time );
				else
					LOG_DEBUG_APP("\r\n定时充电设置完成, num:%d, 定时时间:%d，定时充电状态：%02X", phmi_info->set.cur_num, pdev[phmi_info->set.cur_num]->chgr.timing_time, pdev[phmi_info->set.cur_num]->chgr.st_timing_time );
			}
			else if ( value == 0x0123 )  // 表示关闭定时充电时间功能
			{
			    // 保存设定的参数
			    set_close_chgr_timing_time_set( phmi_info->set.cur_num );

				LOG_DEBUG_APP("\r\n关闭定时充电");
			}
			else if( ( value == 0x0024 ) || \
                     ( value == 0x0035 ) || \
                     ( value == 0x0046 ) || \
                     ( value == 0x0057 ) || \
                     ( value == 0x0068 ) || \
                     ( value == 0x0079 ) || \
                     ( value == 0x008A ) || \
                     ( value == 0x009B ) || \
                     ( value == 0x00AC ) )  // 进入了预约充电设置
			{
				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // 进入的是同步模式的设置
				{
				    //  判断是否有在充电的模块，如果有，那么不进入
					for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
					{
					    if( pdev[idx]->basic.chgr != BASIC_CHGR_OFF )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}
					}

					// 判断是否有模块处于自动充电的模式，如果是，那么禁止跳转到详细参数设置页面
					if( pdev[idx]->chgr.mode == MODE_AUTO )
					{
				        hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
					    return;
					}
				}
				else  // 进入的是1-8模块，单独的设置
				{	
    				// 判断该模块是否在充电，如果在，那么禁止进入
    			    if( pdev[phmi_info->set.cur_num]->basic.chgr != BASIC_CHGR_OFF )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}

					// 判断该模块是否处于自动充电的模式，如果是，那么禁止跳转到详细参数设置页面
    			    if( pdev[phmi_info->set.cur_num]->chgr.mode == MODE_AUTO )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}
				}
				// 更新临时参数
				set_update_temp_chgr_booking_time( phmi_info->set.cur_num );
				// 显示对应页面
                set_disp_chgr_booking_time_set( phmi_info->set.cur_num );
                // 显示对应的定时充电时间
                set_disp_chgr_booking_time( phmi_info->set.tmp_chgr_booking_time );
			}
			else if ( ( value == 0x0128 ) | \
					  ( value == 0x012A) )  // 表示按了返回，返回到对应模块的充电设置主页面 
			{
			    hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );
			}
			else if ( value == 0x011F )  // 表示开启了预约充电功能
			{
                // 显示对应的预约充电时间
                set_disp_chgr_booking_time( phmi_info->set.tmp_chgr_booking_time );
				LOG_DEBUG_APP("\r\n开启预约充电");
			}
			else if ( value == 0x0129 )  // 表示开启了预约充电时间功能，并且设定完成，确认
			{
			    // 保存设定的参数
			    set_chgr_booking_time_set_finish( phmi_info->set.cur_num );

            	// 切换到对应设置的主界面
                hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );

				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // 进入的是同步模式的设置
				    LOG_DEBUG_APP("\r\n预约充电设置完成, 同步设置, 定时时间:%d", pdev[0]->chgr.booking_time );
				else
					LOG_DEBUG_APP("\r\n预约充电设置完成, num:%d, 定时时间:%d", phmi_info->set.cur_num, pdev[phmi_info->set.cur_num]->chgr.booking_time );
			}
			else if ( value == 0x012B )  // 表示关闭定时充电时间功能
			{
			    // 保存设定的参数
			    set_close_chgr_booking_time_set( phmi_info->set.cur_num );

				LOG_DEBUG_APP("\r\n关闭预约充电");
			}
			else if( ( value == 0x0025 ) || \
                     ( value == 0x0036 ) || \
                     ( value == 0x0047 ) || \
                     ( value == 0x0058 ) || \
                     ( value == 0x0069 ) || \
                     ( value == 0x007A ) || \
                     ( value == 0x008B ) || \
                     ( value == 0x009C ) || \
                     ( value == 0x00AD ) )  // 进入了锂电池参数设置
			{
				phmi_info->set.chgr_type = CHGR_TYPE_PRE;  // 标记，当前设置预充电参数

				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // 进入的是同步模式的设置
				{
					for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
					{
				        //  判断是否有在充电的模块，如果有，那么不进入
					    if( pdev[idx]->basic.chgr != BASIC_CHGR_OFF )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}

					    // 判断是否有模块处于自动充电的模式，如果是，那么禁止跳转到详细参数设置页面
					    if( pdev[idx]->chgr.mode == MODE_AUTO )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}
					}
					// 对于同步模式，与模块1看齐
	                set_update_temp_curve_data( POWER_UNIT_1, pdev[ POWER_UNIT_1 ]->chgr.para.curve );
				    set_disp_chgr_set( POWER_UNIT_1 );
				}
				else  // 进入的是1-8模块，单独的设置
				{	
    				// 判断该模块是否在充电，如果在，那么禁止进入
    			    if( pdev[phmi_info->set.cur_num]->basic.chgr != BASIC_CHGR_OFF )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}

					// 判断该模块是否处于自动充电的模式，如果是，那么禁止跳转到详细参数设置页面
    			    if( pdev[phmi_info->set.cur_num]->chgr.mode == MODE_AUTO )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}

    	            set_update_temp_curve_data( phmi_info->set.cur_num, pdev[ phmi_info->set.cur_num ]->chgr.para.curve );
    		        set_disp_chgr_set( phmi_info->set.cur_num );
				}
			}
			else if ( ( value == 0x012F ) )  // 表示在锂电池参数设置里面按了返回，返回到对应模块的充电设置主页面 
			{
			    hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );
			}
			else if ( value == 0x0130 )  // 表示锂电池参数设置完成，确认生效
			{
				// 向设置任务发送设置参数通知
		        xTaskNotify( xSettingDamonHandle, NOTIFY_CHGR_PARA_SETTING, eSetBits);
			}
			else if ( value == 0x2233 ) // 进入密码设置
			{
			    set_enter_passwd_setting();
			}
			else if ( value == 0x2234 ) // 确认修改密码
			{
			    set_passwd_setting_verify();
			}
			else if ( value == 0x2235 ) // 表示修改失败
			{
			    set_enter_passwd_setting();
			}
			else if ( value == 0x4457 ) // 按了恢复出厂设置的执行
			{
				for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
				{
					// 判断是否有模块处于自动充电的模式，如果是，那么禁止跳转到确认界面
					if( pdev[idx]->chgr.mode == MODE_AUTO )
					{
				        hmi_switch_to_spec_page( HMI_SET_CHGR_RESTROE_FACTORY_ID );
					    return;
					}
				
				}

		        hmi_switch_to_spec_page( HMI_SET_CHGR_RESTROE_FACTORY_CONFIRM_ID ); // 跳到确认界面
			}
			else if ( value == 0x4456 ) // 按了确认恢复出厂设置
			{
				// 向设置任务发送设置参数通知
		        xTaskNotify( xSettingDamonHandle, NOTIFY_RESTORE_FACTORY_SETTING, eSetBits);
			}
			else if( ( value == 0x0026 ) || \
                     ( value == 0x0037 ) || \
                     ( value == 0x0048 ) || \
                     ( value == 0x0059 ) || \
                     ( value == 0x006A ) || \
                     ( value == 0x007B ) || \
                     ( value == 0x008C ) || \
                     ( value == 0x009D ) || \
                     ( value == 0x00AE ) )  // 进入了自动充电阈值设置
			{
				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // 进入的是同步模式的设置
				{
					for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
					{
				        //  判断是否有在充电的模块，如果有，那么不进入
					    if( pdev[idx]->basic.chgr != BASIC_CHGR_OFF )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}

					    // 判断是否有模块处于自动充电的模式，如果是，那么禁止跳转到详细参数设置页面
					    if( pdev[idx]->chgr.mode == MODE_AUTO )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}
					}
				}
				else  // 进入的是1-8模块，单独的设置
				{
    				// 判断该模块是否在充电，如果在，那么禁止进入
    			    if( pdev[phmi_info->set.cur_num]->basic.chgr != BASIC_CHGR_OFF )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}

					// 判断该模块是否处于自动充电的模式，如果是，那么禁止跳转到详细参数设置页面
    			    if( pdev[phmi_info->set.cur_num]->chgr.mode == MODE_AUTO )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}
				}

				// 更新临时参数
				set_update_temp_chgr_threshold( phmi_info->set.cur_num );
                // 显示对应的定时充电时间
                set_disp_chgr_threshold( phmi_info->set.tmp_chgr_threshold );
			}
			else if ( value == 0x0135 )  // 表示在阈值设置中，按了返回，返回到对应模块的充电设置主页面 
			{
			    hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );
			}
			else if ( value == 0x0136 )  // 表示在阈值设置中，确认设置完成
			{
				phmi_info->set.chgr_type = CHGR_TYPE_AUTO;  // 标记，当前设置自动充电参数
				// 向设置任务发送设置参数通知
		        xTaskNotify( xSettingDamonHandle, NOTIFY_CHGR_PARA_SETTING, eSetBits);
			}
			else if( value == 0x00AF ) // 无法同步设置页面的确认
			{
				hmi_switch_to_spec_page( HMI_SET_ENTER_ID );
			}
			else if( value == 0x010F ) // 表示在设置参数生效中的界面，点击了充电设置
			{
				if( phmi_info->flag_setting != FALSE )
				{
					LOG_DEBUG_APP("\r\nin set ing status, do not switch");
					hmi_switch_to_spec_page( HMI_SET_SET_ING_ID );
				}
			}
			else if( value == 0x4457 ) // 表示在恢复出厂设置中的界面，点击了充电设置
			{
				// 界面配置的是，点击后屏幕自动切换
				// 如果处于生效中的状态，这里就通过软件再切换到生效中的界面
				if( phmi_info->flag_setting != FALSE )
				{
					LOG_DEBUG_APP("\r\nin restore factory status, do not switch");
					hmi_switch_to_spec_page( HMI_SET_CHGR_RESTROE_FACTORY_ING_ID );
				}
			}
			break;
		case 0x1055:  // 表示输入了恒充电流
			phmi_info->set.tmp_para.data.ichg = (float)value/10; LOG_TRACE("\r\n3-section, ichg: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1057:  // 表示输入了均转电流
			phmi_info->set.tmp_para.data.itaper = (float)value/10; LOG_TRACE("\r\n3-section, itaper: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1059:  // 表示输入了恒充电压
			phmi_info->set.tmp_para.data.vbst = (float)value/10; LOG_TRACE("\r\n3-section, vbst: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x105B:  // 表示输入了浮充电压
			phmi_info->set.tmp_para.data.vfloat = (float)value/10; LOG_TRACE("\r\n3-section, vfloat: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1170:  // 表示输入了预充电流
			phmi_info->set.tmp_para.data_pre.pre_iout = (float)value/10; LOG_TRACE("\r\npre_chgr, pre_iout: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1172:  // 表示输入了预充电压
			phmi_info->set.tmp_para.data_pre.pre_vout = (float)value/10; LOG_TRACE("\r\npre_chgr, pre_vout: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1174:  // 表示输入了恒充电流
			phmi_info->set.tmp_para.data_pre.ichg = (float)value/10; LOG_TRACE("\r\npre_chgr, ichg: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1176:  // 表示输入了终止电压
			phmi_info->set.tmp_para.data_pre.vbst = (float)value/10; LOG_TRACE("\r\npre_chgr, vbst: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1178:  // 表示输入了均转电流
			phmi_info->set.tmp_para.data_pre.itaper = (float)value/10; LOG_TRACE("\r\npre_chgr, itaper: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1011: //  表示输入了密码，更新输入的密码，把输入的密码保存到临时变量里面
			set_update_tmp_input_passwd();
			break;
		case 0x115D: //  表示输入了密码设置的密码，1.更新输入的密码，把输入的密码保存到临时变量里面; 2.切换到修改的页面
			set_update_tmp_input_passwd();
			hmi_switch_to_spec_page( HMI_SET_PASSWD_SETTING_MODIFY_ID );
			break;
		case 0x1065: // 表示输入了定时充电时间 
			phmi_info->set.tmp_chgr_timing_time = value; LOG_DEBUG_APP("\r\nchgr_timing_time:%d", value);
			break;
		case 0x106D: // 表示输入了定时充电时间 
			phmi_info->set.tmp_chgr_booking_time = value; LOG_DEBUG_APP("\r\nchgr_booking_time:%d", value);
			break;
		case 0x1180: // 表示输入了自动充电的电量阈值
			phmi_info->set.tmp_chgr_threshold = value; LOG_DEBUG_APP("\r\nchgr_threshold:%d", value);
			break;
		default:
			break;
	}
}

static void set_chk_input_para( ChgrParaDef *para )
{
	uint16_t data;
	uint16_t addr;

	if( para->data.itaper - para->data.ichg > 0 )
	{
		LOG_DEBUG_APP_1( "three section, itaper > ichg, use ichg" );
		para->data.itaper = para->data.ichg;
	}
	if( para->data.vfloat - para->data.vbst > 0 )
	{
		LOG_DEBUG_APP_1( "three section, vfloat > vbst, use vbst" );
		para->data.vfloat = para->data.vbst;
	}

	if( para->data_pre.pre_iout - para->data_pre.ichg > 0 )
	{
		LOG_DEBUG_APP_1("pre_chgr, pre_iout > ichg, use ichg");
        para->data_pre.pre_iout = para->data_pre.ichg;
	}
	if( para->data_pre.itaper - para->data_pre.ichg > 0 )
	{
		LOG_DEBUG_APP_1("pre_chgr, itaper > ichg, use ichg");
        para->data_pre.itaper = para->data_pre.ichg;
	}
	if( para->data_pre.pre_vout - para->data_pre.vbst > 0 )
	{
		LOG_DEBUG_APP_1("pre_chgr, pre_vout > vbst, use ichg");
        para->data_pre.pre_vout = para->data_pre.vbst;
	}
	
	// disp data
	addr = 0x1055;
	data = para->data.ichg * 10;
	hmi_disp_para_data( addr, data );

	addr = 0x1057;
	data = para->data.itaper * 10;
	hmi_disp_para_data( addr, data );

	addr = 0x1059;
	data = para->data.vbst * 10;
	hmi_disp_para_data( addr, data );

	addr = 0x105B;
	data = para->data.vfloat * 10;
	hmi_disp_para_data( addr, data );
//
	addr = 0x1170;
	data = para->data_pre.pre_iout * 10;
	hmi_disp_para_data( addr, data );

	addr = 0x1172;
	data = para->data_pre.pre_vout * 10;
	hmi_disp_para_data( addr, data );

	addr = 0x1174;
	data = para->data_pre.ichg * 10;
	hmi_disp_para_data( addr, data );

	addr = 0x1176;
	data = para->data_pre.vbst * 10;
	hmi_disp_para_data( addr, data );

	addr = 0x1178;
	data = para->data_pre.itaper * 10;
	hmi_disp_para_data( addr, data );
}

static void set_update_tmp_input_passwd( void )
{
	uint8_t idx;
	uint8_t input_len = 0;
    uint8_t tmp[ PASSWD_LEN_SET ];

	for( idx = 0, input_len = 0; idx < phmi_rx_frame->data.len; idx ++ )
	{
		tmp[ input_len ++ ] = ( phmi_rx_frame->data.data[idx] >> 8 ) & 0xff;

	    if ( ( phmi_rx_frame->data.data[idx] & 0xff ) != 0xff )
		{
		    tmp[ input_len ++ ] = phmi_rx_frame->data.data[idx] & 0xff;
		}
		else
		{
		    tmp[ input_len ++ ] = 'f';  // 如果收到0xff，把最后一个字符设置为f，判断密码设置中，判断输入是否正确的依据
		}
	}
	
	for( idx = 0; idx < input_len; idx ++ )
	{
	    phmi_info->set.tmp_set_passwd[idx] = tmp[idx];
	}
//	memmove( phmi_info->set.tmp_set_passwd, (const *)tmp, input_len );
    phmi_info->set.tmp_set_passwd[ input_len ] = '\0';

	LOG_DEBUG_APP("\r\nupdate_tmp_set_input_passwd, input_len:%d, input passwd:%s", input_len, phmi_info->set.tmp_set_passwd );
}

static void set_enter( void )
{
    uint8_t passwd[ PASSWD_LEN_SET+1 ] = {0};	

    // 读取当前存储在eeprom中的设置密码
    set_read_passwd( passwd );
    passwd[ PASSWD_LEN_SET ] = '\0';

	// 检测输入的密码是否正确
	if( strcmp( (char *)phmi_info->set.tmp_set_passwd, (char *)passwd ) == 0 )
	{
		// 进入设置
	    hmi_switch_to_spec_page( HMI_SET_ENTER_ID );
	}
	else
	{
		// 弹出密码输入错误界面
	    hmi_switch_to_spec_page( HMI_SET_PASSWD_ERR_ID );
	}

	// 清除临时保存输入密码的缓冲区
	memset((char *)phmi_info->set.tmp_set_passwd, 0xff, PASSWD_LEN_SET+1 );
}

static void set_read_passwd( uint8_t *passwd ) 
{
    ee_ReadBytes( passwd, EE_PASSWD_SET, PASSWD_LEN_SET );

	delay_ms( EE_MS_DELAY );
}

static void set_save_passwd( uint8_t *passwd )
{
	// 更新设置的密码
    ee_WriteBytes( passwd, EE_PASSWD_SET, PASSWD_LEN_SET );
	vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
}

static void set_enter_passwd_setting( void )
{
	uint16_t addr = 0x1163;

	// 切换到密码设置页面
	hmi_switch_to_spec_page( HMI_SET_PASSWD_SETTING_ID );

	// 显示未修改状态
	hmi_disp_string( addr, (uint8_t *)"未修改", 6 );
}

static void set_passwd_setting_verify( void )
{
	uint16_t addr = 0x1163;
	uint8_t len_passwd_input = 0;

	len_passwd_input = strlen( (const char *)phmi_info->set.tmp_set_passwd );

	// 判断输入的长度是否正确
    if( phmi_info->set.tmp_set_passwd[ len_passwd_input-1 ] == 'f')
	{
		// 切换到错误提示界面
		hmi_switch_to_spec_page( HMI_SET_PASSWD_SETTING_FAIL_ID );
		LOG_DEBUG_APP("\r\nmodify passwd failed, passwd_input_len:%d, input passwd:%s", len_passwd_input, phmi_info->set.tmp_set_passwd );
	}
	else
	{
		// 保存输入的密码
		set_save_passwd( (uint8_t *)phmi_info->set.tmp_set_passwd );

		// 保存维护的密码
		maintain_save_passwd( (uint8_t *)phmi_info->set.tmp_set_passwd );
		
		LOG_DEBUG_APP("\r\nmodify passwd success, input passwd:%s", phmi_info->set.tmp_set_passwd );
	    
	    // 切换到密码设置界面，然后状态显示已修改
		hmi_switch_to_spec_page( HMI_SET_PASSWD_SETTING_ID );
	    hmi_disp_string( addr, (uint8_t *)"已修改", 6 );
	}

	// 清除临时保存输入密码的缓冲区
	memset((char *)phmi_info->set.tmp_set_passwd, 0xff, PASSWD_LEN_SET+1 );
}

static void hmi_disp_para_data( uint16_t addr, uint16_t data )
{
	uint8_t idx = 0;
    uint8_t buff[] = { 0x5A, 0xA5, 0x05, 0x82, 0x00, 0x00, 0x00, 0x00 };
	
	buff[4] = (addr>>8) & 0xff;
	buff[5] = addr & 0xff;
	
	buff[6] = (data>>8) & 0xff;
	buff[7] = data & 0xff;
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void set_disp_chgr_set( uint8_t num )
{
	uint16_t cur_page_id_tab[] = { HMI_SET_CURVE1_ID, HMI_SET_CURVE2_ID, HMI_SET_CURVE3_ID, HMI_SET_CURVE4_ID, HMI_SET_CURVE_U_ID };
	
	if( phmi_info->set.chgr_type == CHGR_TYPE_THREE_SECTION )
	{
	    // 切换到对应曲线的页面
	    hmi_switch_to_spec_page( cur_page_id_tab [pdev[num]->chgr.para.curve] );
	    LOG_DEBUG_APP("\r\n切换到页面， ID：%d", cur_page_id_tab [pdev[num]->chgr.para.curve]);
	    
	    // 显示对应的曲线
	    set_disp_curve( pdev[num]->chgr.para.curve );
	}
	else if( phmi_info->set.chgr_type == CHGR_TYPE_PRE )
	{
	    // 显示对应的参数
        set_disp_pre_chgr_para_data( &pdev[num]->chgr.para.data_pre );
	}
}

static void set_disp_curve_data( CHGR_ParaDataDef *pdata )
{
	uint16_t uwValue = 0;
	uint16_t addr[] = { 0x1055, 0x1057, 0x1059, 0x105B }; // ichg, itaper, vbst, vfloat;
	
    // 显示恒充电流
	uwValue = pdata->ichg * 10;
	hmi_disp_para_data( addr[IDX_ICHG], uwValue);
	
	// 显示均转电流
	uwValue = pdata->itaper * 10;
	hmi_disp_para_data( addr[IDX_ITAPER], uwValue);
	
	// 显示终止电压
	uwValue = pdata->vbst * 10;
	hmi_disp_para_data( addr[IDX_VBST], uwValue);
	
	// 显示浮充电压
	uwValue = pdata->vfloat * 10;
	hmi_disp_para_data( addr[IDX_VFLOAT], uwValue);
}

static void set_disp_pre_chgr_para_data( PreChgrParaDataDef *pdata )
{
	uint16_t uwValue = 0;
	uint16_t addr[] = { 0x1170, 0x1172, 0x1174, 0x1176, 0x1178 }; // pre_iout, pre_vout, ichg, vbst, itaper;
	
    // 显示预充电流
	uwValue = pdata->pre_iout * 10;
	hmi_disp_para_data( addr[IDX_PRE_CHGR_PRE_IOUT], uwValue);

    // 显示预充电压
	uwValue = pdata->pre_vout * 10;
	hmi_disp_para_data( addr[IDX_PRE_CHGR_PRE_VOUT], uwValue);

    // 显示恒充电流
	uwValue = pdata->ichg * 10;
	hmi_disp_para_data( addr[IDX_PRE_CHGR_ICHG], uwValue);
	
    // 显示恒充电压
	uwValue = pdata->vbst * 10;
	hmi_disp_para_data( addr[IDX_PRE_CHGR_VBST], uwValue);

    // 显示终止电流
	uwValue = pdata->itaper * 10;
	hmi_disp_para_data( addr[IDX_PRE_CHGR_ITAPER], uwValue);
}

static void set_disp_curve( uint8_t curve )
{
	CHGR_ParaDataDef data;
	
	// 如果是用户设置的参数，显示设置的参数，否则直接从预置曲线里面读取参数
	if( curve == CHGR_PARA_CURVE_U )
	{
        data = phmi_info->set.tmp_para.data;
#if 0		
		printf("\r\n自定义, cur_num:%d %f, %f, %f ,%f", phmi_info->set.cur_num, pdev[phmi_info->set.cur_num]->chgr.para.data.ichg, \
		                                     pdev[phmi_info->set.cur_num]->chgr.para.data.itaper, \
		                                     pdev[phmi_info->set.cur_num]->chgr.para.data.vbst, \
		                                     pdev[phmi_info->set.cur_num]->chgr.para.data.vfloat );
#endif
	}
	else
	{
	    data.ichg = default_curve_data[ curve ][IDX_ICHG];
        data.itaper = default_curve_data[ curve ][IDX_ITAPER];
	    data.vbst = default_curve_data[ curve ][IDX_VBST];
	    data.vfloat = default_curve_data[ curve ][IDX_VFLOAT];
	}
	
	// 显示曲线的数据
	set_disp_curve_data( &data );
}

static void set_disp_chgr_timing_time( uint16_t data )
{
	uint8_t idx = 0;
    uint8_t buff[] = { 0x5A, 0xA5, 0x05, 0x82, 0x10, 0x65, 0x00, 0x00 };
	
	buff[6] = (data>>8) & 0xff;
	buff[7] = data & 0xff;
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void set_disp_chgr_timing_time_set( uint8_t num )
{
	// 同步设置的参数，存在模块1里面
	if( num == POWER_UNIT_SY )
	{
		num = POWER_UNIT_1;
	}

   	if( pdev[ num ]->chgr.st_timing_time != DISABLE )
   	{
		// 切换到定时充电，开启的页面
   	    hmi_switch_to_spec_page( HMI_SET_CHGR_TIME_ON_ID ); 
   	}
	else
	{
		// 切换到定时充电，关闭的页面
   	    hmi_switch_to_spec_page( HMI_SET_CHGR_TIME_OFF_ID ); 
	}
}

static void set_update_temp_chgr_timing_time( uint8_t num )
{
	// 同步设置的参数，存在模块1里面
	if( num == POWER_UNIT_SY )
	{
		num = POWER_UNIT_1;
	}

	if( pdev[ num ]->chgr.st_timing_time != DISABLE )
	{
	    phmi_info->set.tmp_chgr_timing_time = pdev[ num ]->chgr.timing_time;
	}
	else
	{
	    phmi_info->set.tmp_chgr_timing_time = CHGR_TIMING_TIME_DEFAULT;
	}
}

static void set_chgr_timing_time_set_finish( uint8_t num )
{
	uint8_t idx = 0;

	// 如果是同步设置，那么设置所有模块
	if( num == POWER_UNIT_SY )
	{
		for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
		{
	        pdev[ idx ]->chgr.st_timing_time = ENABLE;
	        pdev[ idx ]->chgr.timing_time = phmi_info->set.tmp_chgr_timing_time;
		}
	}
	else
	{
	    pdev[ num ]->chgr.st_timing_time = ENABLE;
	    pdev[ num ]->chgr.timing_time = phmi_info->set.tmp_chgr_timing_time;
	}

	LOG_TRACE_1("\r\nst_timing_time:%d\n", pdev[ num ]->chgr.st_timing_time);
}

static void set_close_chgr_timing_time_set( uint8_t num )
{
	uint8_t idx = 0;

	// 如果是同步设置，那么设置所有模块
	if( num == POWER_UNIT_SY )
	{
		for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
		{
	        pdev[ idx ]->chgr.st_timing_time = DISABLE;
	        pdev[ idx ]->chgr.timing_time = CHGR_TIMING_TIME_DEFAULT;
		}
	}
	else
	{
	    pdev[ num ]->chgr.st_timing_time = DISABLE;
	    pdev[ num ]->chgr.timing_time = CHGR_TIMING_TIME_DEFAULT;
	}

    phmi_info->set.tmp_chgr_timing_time = CHGR_TIMING_TIME_DEFAULT;
}

static void set_disp_chgr_booking_time( uint16_t data )
{
	uint8_t idx = 0;
    uint8_t buff[] = { 0x5A, 0xA5, 0x05, 0x82, 0x10, 0x6D, 0x00, 0x00 };
	
	buff[6] = (data>>8) & 0xff;
	buff[7] = data & 0xff;
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();
}

static void set_disp_chgr_booking_time_set( uint8_t num )
{
	// 同步设置的参数，存在模块1里面
	if( num == POWER_UNIT_SY )
	{
		num = POWER_UNIT_1;
	}

   	if( pdev[ num ]->chgr.st_booking_time != DISABLE )
   	{
		// 切换到定时充电，开启的页面
   	    hmi_switch_to_spec_page( HMI_SET_CHGR_BOOKING_ON_ID ); }
	else
	{
		// 切换到定时充电，关闭的页面
   	    hmi_switch_to_spec_page( HMI_SET_CHGR_BOOKING_OFF_ID ); 
	}
}

static void set_update_temp_chgr_booking_time( uint8_t num )
{
	// 同步设置的参数，存在模块1里面
	if( num == POWER_UNIT_SY )
	{
		num = POWER_UNIT_1;
	}

	if( pdev[ num ]->chgr.st_booking_time != DISABLE )
	{
	    phmi_info->set.tmp_chgr_booking_time = pdev[ num ]->chgr.booking_time;
	}
	else
	{
	    phmi_info->set.tmp_chgr_booking_time = CHGR_BOOKING_TIME_DEFAULT;
	}
}

static void set_chgr_booking_time_set_finish( uint8_t num )
{
	uint8_t idx = 0;

	// 如果是同步设置，那么设置所有模块
	if( num == POWER_UNIT_SY )
	{
		for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
		{
	        pdev[ idx ]->chgr.st_booking_time = ENABLE;
	        pdev[ idx ]->chgr.booking_time = phmi_info->set.tmp_chgr_booking_time;
		}
	}
	else
	{
	    pdev[ num ]->chgr.st_booking_time = ENABLE;
	    pdev[ num ]->chgr.booking_time = phmi_info->set.tmp_chgr_booking_time;
	}
}

static void set_close_chgr_booking_time_set( uint8_t num )
{
	uint8_t idx = 0;

	// 如果是同步设置，那么设置所有模块
	if( num == POWER_UNIT_SY )
	{
		for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
		{
	        pdev[ idx ]->chgr.st_booking_time = DISABLE;
	        pdev[ idx ]->chgr.booking_time = CHGR_BOOKING_TIME_DEFAULT;
		}
	}
	else
	{
	    pdev[ num ]->chgr.st_booking_time = DISABLE;
	    pdev[ num ]->chgr.booking_time = CHGR_BOOKING_TIME_DEFAULT;
	}

    phmi_info->set.tmp_chgr_booking_time = CHGR_BOOKING_TIME_DEFAULT;
}

static void set_update_temp_chgr_threshold( uint8_t num )
{
	// 同步设置的参数，存在模块1里面
	if( num == POWER_UNIT_SY )
	{
		num = POWER_UNIT_1;
	}

	// 更新临时参数
	phmi_info->set.tmp_chgr_threshold = pdev[ num ]->chgr.para.data_auto.threshold;
}

static void set_disp_chgr_threshold( uint16_t data  )
{
    hmi_disp_para_data( 0x1180, data );
}

static void set_chgr_threshold_set_finish( uint8_t num )
{
	uint8_t idx = 0;
	uint16_t data = 0;
	uint16_t addr = 0;

	// 如果是同步设置，那么设置所有模块
	if( num == POWER_UNIT_SY )
	{
		for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
		{
	        pdev[ idx ]->chgr.para.data_auto.threshold = phmi_info->set.tmp_chgr_threshold;
			data = phmi_info->set.tmp_chgr_threshold;
			addr = ee_chgr_para_addr[idx] + EE_OFFSET_CHGR_THRESHOLD;
			ee_WriteBytes( ( uint8_t * )&data, addr, 2 );
		    vTaskDelay( pdMS_TO_TICKS( PMBUS_COMM_MS_DELAY ) );
			LOG_TRACE_1("\r\nnum:%d set threshold:%d", idx, data);
		}
	}
	else
	{
	    pdev[ num ]->chgr.para.data_auto.threshold = phmi_info->set.tmp_chgr_threshold;
		data = phmi_info->set.tmp_chgr_threshold;
		addr = ee_chgr_para_addr[num] + EE_OFFSET_CHGR_THRESHOLD;
		ee_WriteBytes( (uint8_t *)&data, addr, 2 );
		vTaskDelay( pdMS_TO_TICKS( PMBUS_COMM_MS_DELAY ) );
		LOG_TRACE_1("\r\nnum:%d set threshold:%d", num, data);
	}
}

static void set_restore_factory_default( void )
{
	// 表示生效中
	phmi_info->flag_setting = TRUE;

	// 开启动画
	hmi_start_animate();

	// 关闭所有模块的充电
	pub_stop_all();
	
	// 打开所有模块
	power_on_all();
	vTaskDelay( pdMS_TO_TICKS( POWER_ON_MS_DELAY ) );

	// 清除掉eeprom中保存的初始化信息，重新初始化
    erase_ee_flag();

	// 重新初始化模块信息
	init_power_unit_info();

	// 修改临时参数信息，直接调用同步模式的保存参数函数来设置充电参数
	phmi_info->set.tmp_para.curve = pdev[0]->chgr.para.curve;
	phmi_info->set.tmp_para.data = pdev[0]->chgr.para.data;
	phmi_info->set.tmp_para.data_pre = pdev[0]->chgr.para.data_pre;

	// 生效参数
	set_save_chgr_para_syn();
    set_save_pre_chgr_para_syn();

	// 关闭所有模块
	power_off_all();

	// 关闭动画
	hmi_stop_animate();

	// 切换到完成页面
	hmi_switch_to_spec_page( HMI_SET_CHGR_RESTROE_FACTORY_FINISH_ID );

	// 添加切换校验
	hmi_verify_page_switch( HMI_SET_CHGR_RESTROE_FACTORY_FINISH_ID );
	LOG_DEBUG_APP_1("\r\n开始校验页面切换，页面ID:%d", (uint16_t)HMI_SET_CHGR_RESTROE_FACTORY_FINISH_ID );

	// 表示生效完成
	phmi_info->flag_setting = FALSE;
}

static void set_update_temp_curve_data( uint8_t num, uint8_t curve )
{
	LOG_DEBUG_APP("\r\nupdate tmp curve data, ");
	// 如果是同步模式，显示模块1的参数
	if( num == POWER_UNIT_SY )
	{
	    LOG_DEBUG_APP_1("\r\nin syn mode");
	    num = POWER_UNIT_1;
	}

	// 更新临时数据，包括curve, data, !!!ee_addr（）
	// !!!一定要更新ee_addr，否则后面会导致保存参数的地址错误(20181228调试了一下午加晚上才找到是这个问题！！！)
	phmi_info->set.tmp_para = pdev[num]->chgr.para;

	LOG_DEBUG_APP_1("\r\nnum:%d, ee_addr:%04X, tmp_ee_addr:%04X", \
					num, pdev[num]->chgr.para.ee_addr, phmi_info->set.tmp_para.ee_addr );

	LOG_DEBUG_APP_1("\r\ntemp, 3-section:%f, %f, %f, %f", \
					phmi_info->set.tmp_para.data.ichg, phmi_info->set.tmp_para.data.itaper, \
					phmi_info->set.tmp_para.data.vbst, phmi_info->set.tmp_para.data.vfloat );

	LOG_DEBUG_APP_1("\r\ntemp, pre-chgr:%f, %f, %f, %f, %f", \
					phmi_info->set.tmp_para.data_pre.pre_iout, phmi_info->set.tmp_para.data_pre.pre_vout, \
					phmi_info->set.tmp_para.data_pre.ichg, phmi_info->set.tmp_para.data_pre.vbst, \
					phmi_info->set.tmp_para.data_pre.itaper	);

	if( phmi_info->set.chgr_type == CHGR_TYPE_THREE_SECTION )
	{
	    // 更新临时曲线
	    phmi_info->set.tmp_para.curve = curve;
	    
	    LOG_DEBUG_APP_1("\r\nthree-section type");
	    if( curve == CHGR_PARA_CURVE_U )
	    {
	        LOG_DEBUG_APP_1("\r\n自定义参数");
	    }
        else
	    {
	        // 如果是预置的，直接从预置曲线里面读即可
	    	LOG_DEBUG_APP_1("\r\n预置曲线参数，curve:%d, ", curve);
	        phmi_info->set.tmp_para.data.ichg = default_curve_data[ curve ][IDX_ICHG];
            phmi_info->set.tmp_para.data.itaper = default_curve_data[ curve ][IDX_ITAPER];
	        phmi_info->set.tmp_para.data.vbst = default_curve_data[ curve ][IDX_VBST];
	        phmi_info->set.tmp_para.data.vfloat = default_curve_data[ curve ][IDX_VFLOAT];	
	    }
	    
	    LOG_DEBUG_APP_1("\r\nupdate, cur_num:%d, temp para:  curve:%d, %f, %f, %f ,%f", num, \
	    	phmi_info->set.tmp_para.curve, \
	        phmi_info->set.tmp_para.data.ichg, \
	    	phmi_info->set.tmp_para.data.itaper, \
	    	phmi_info->set.tmp_para.data.vbst, \
	    	phmi_info->set.tmp_para.data.vfloat );
	}
	else if( phmi_info->set.chgr_type == CHGR_TYPE_PRE )
	{
		LOG_DEBUG_APP_1("\r\npre type");
	    LOG_DEBUG_APP_1("\r\nupdate, cur_num:%d, temp para: %f, %f, %f, %f ,%f", num, \
	    	phmi_info->set.tmp_para.data_pre.pre_iout, \
	    	phmi_info->set.tmp_para.data_pre.pre_vout, \
	        phmi_info->set.tmp_para.data_pre.ichg, \
	    	phmi_info->set.tmp_para.data_pre.vbst, \
	    	phmi_info->set.tmp_para.data_pre.itaper );
	}
}

static void set_read_chgr_para_from_ee( ChgrParaDef *para  )
{
	uint16_t addr = para->ee_addr;
	
	/****************** 读三段式充电充电参数 *********************/
	// read curve
	ee_ReadBytes( (uint8_t *)&( para->curve ), addr, 1 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
	
	// read ichg
	addr += 1;
	ee_ReadBytes( (uint8_t *)&( para->data.ichg ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
	
	// read itaper
	addr += 4;
	ee_ReadBytes( (uint8_t *)&( para->data.itaper ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
	
	// read vbst
	addr += 4;
	ee_ReadBytes( (uint8_t *)&( para->data.vbst ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	// read vfloat
	addr += 4;
	ee_ReadBytes( (uint8_t *)&( para->data.vfloat ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );


	/****************** 读预充电充电参数 *********************/
	// read pre_iout
	addr += 4;
	ee_ReadBytes( (uint8_t *)&( para->data_pre.pre_iout ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	// read pre_vout
	addr += 4;
	ee_ReadBytes( (uint8_t *)&( para->data_pre.pre_vout ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
	
	// read ichg
	addr += 4;
	ee_ReadBytes( (uint8_t *)&( para->data_pre.ichg ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	// read vbst 
	addr += 4;
	ee_ReadBytes( (uint8_t *)&( para->data_pre.vbst ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	// read itaper
	addr += 4;
	ee_ReadBytes( (uint8_t *)&( para->data_pre.itaper ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	/****************** 读自动电充电参数 *********************/
	// read threshold
	addr += 4;
	ee_ReadBytes( (uint8_t *)&( para->data_auto.threshold ), addr, 2 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	LOG_TRACE("\r\nin read, ee_addr:%04X(H)-(%d(D)), 3_period_chgr: curve:%d, %f, %f, %f, %f, pre_chgr:%f, %f, %f, %f, %f, auto_chgr:threshold:%d", \
				para->ee_addr, para->ee_addr, para->curve, para->data.ichg, para->data.itaper, para->data.vbst, para->data.vfloat,\
				para->data_pre.pre_iout, para->data_pre.pre_vout, para->data_pre.ichg, para->data_pre.vbst, \
				para->data_pre.itaper, para->data_auto.threshold );
}

static void set_write_chgr_para_to_ee( ChgrParaDef *para )
{
	uint16_t addr = para->ee_addr;
	
	/****************** 写三段式充电充电参数 *********************/
	// write curve
	ee_WriteBytes( (uint8_t *)&( para->curve ), addr, 1 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	// write ichg
	addr += 1;
	ee_WriteBytes( (uint8_t *)&( para->data.ichg ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	// write itaper
	addr += 4;
	ee_WriteBytes( (uint8_t *)&( para->data.itaper ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	// write vbst
	addr += 4;
	ee_WriteBytes( (uint8_t *)&( para->data.vbst ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	// write vfloat
	addr += 4;
	ee_WriteBytes( (uint8_t *)&( para->data.vfloat ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	/****************** 写预充电充电参数 *********************/
	// read pre_iout
	addr += 4;
	ee_WriteBytes( (uint8_t *)&( para->data_pre.pre_iout ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	// read pre_vout
	addr += 4;
	ee_WriteBytes( (uint8_t *)&( para->data_pre.pre_vout ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
	
	// read ichg
	addr += 4;
	ee_WriteBytes( (uint8_t *)&( para->data_pre.ichg ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	// read vbst 
	addr += 4;
	ee_WriteBytes( (uint8_t *)&( para->data_pre.vbst ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	// read itaper
	addr += 4;
	ee_WriteBytes( (uint8_t *)&( para->data_pre.itaper ), addr, 4 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	/****************** 写自动充电参数 *********************/
	// 
	addr += 4;
	ee_WriteBytes( (uint8_t *)&( para->data_auto.threshold ), addr, 2 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	
//	LOG_TRACE("\r\nin write, ee_addr:%04X, curve:%d, %f, %f, %f, %f", para->ee_addr, para->curve, para->data.ichg, para->data.itaper, para->data.vbst, para->data.vfloat );
	LOG_TRACE("\r\nin write, ee_addr:%04X(H)-(%d(D)), curve:%d", para->ee_addr, para->ee_addr, para->curve );
}

static void set_save_chgr_para( uint8_t num )
{
	float range = 1; // 读写误差
#ifdef __LOG_TRACE
	ChgrParaDef para;
	uint8_t flag = 0;
#endif
	
	LOG_TRACE_1("\r\nbefore save, tmp_para, dev:%d, ee_addr:%04X, curve:%d, ichg:%f, itaper:%f, vbst:%f, vfloat:%f", num, phmi_info->set.tmp_para.ee_addr, \
	                                                                      phmi_info->set.tmp_para.curve, \
	                                                                      phmi_info->set.tmp_para.data.ichg, \
	                                                                      phmi_info->set.tmp_para.data.itaper, \
		                                                                  phmi_info->set.tmp_para.data.vbst, \
	                                                                      phmi_info->set.tmp_para.data.vfloat );
	
    // 第一步： 先把临时参数赋值给实际参数(ee_addr, curve和四个data)
	// 注意：这里的赋值方式一定要和 set_update_temp_curve_data()这个函数相对应，如果上面的函数是直接全部赋值的para的，这里可以直接赋值para
	// 如果上面的函数不是直接赋值para，那么这里就要注意了，一定不能忘了ee_addr的赋值，否则会导致设置参数后，导致eeprom里面的数据错乱
	pdev[num]->chgr.para = phmi_info->set.tmp_para;
	
	LOG_TRACE_1("\r\nreal para, dev:%d, ee_addr:%04X, curve:%d, ichg:%f, itaper:%f, vbst:%f, vfloat:%f", num, pdev[num]->chgr.para.ee_addr, \
	                                                                      pdev[num]->chgr.para.curve, \
	                                                                      pdev[num]->chgr.para.data.ichg, \
	                                                                      pdev[num]->chgr.para.data.itaper, \
		                                                                  pdev[num]->chgr.para.data.vbst, \
	                                                                      pdev[num]->chgr.para.data.vfloat );
	
	// 第二步： 把实际参数写入功率模块
	if ( set_write_chgr_para_to_unit_verify( WriteCURVE_ICHG, fReadCURVE_ICHG, pdev[num]->basic.addr, \
						 ( pdev[num]->chgr.para.data.ichg ), range ) )
	{ 
		LOG_TRACE_1("\r\n write ichg success");
	}
	else
	{ 
		LOG_TRACE_1("\r\n write ichg failed");
	}
	
	if ( set_write_chgr_para_to_unit_verify( WriteCURVE_ITAPER, fReadCURVE_ITAPER, pdev[num]->basic.addr, \
						 ( pdev[num]->chgr.para.data.itaper ), range ) )
	{
		LOG_TRACE_1("\r\n write itaper success");
	}
	else
	{ 
		LOG_TRACE_1("\r\n write itaper failed");
	}
	
	if ( set_write_chgr_para_to_unit_verify( WriteCURVE_VBST, fReadCURVE_VBST, pdev[num]->basic.addr, \
						 pdev[num]->chgr.para.data.vbst, range ) )
	{
		LOG_TRACE_1("\r\n write vbst success");
	}
	else
	{ 
		LOG_TRACE_1("\r\n write vbst failed");
	}
	
	if ( set_write_chgr_para_to_unit_verify( WriteCURVE_VFLOAT, fReadCURVE_VFLOAT, pdev[num]->basic.addr, \
						 pdev[num]->chgr.para.data.vfloat, range ) )
	{
		LOG_TRACE_1("\r\n write vfloat success");
	}
	else
	{
		LOG_TRACE_1("\r\n write vfloat failed");
	}	
	// 第三步： 把参数写入eeprom保存
	LOG_TRACE_1("\r\nbefore write, ee_addr:%04X, num:%d, ichg:%f", pdev[num]->chgr.para.ee_addr, num, pdev[num]->chgr.para.data.ichg );
	set_write_chgr_para_to_ee( &( pdev[num]->chgr.para ) );
	LOG_TRACE_1("\r\nwrite ee_addr:%04X", pdev[num]->chgr.para.ee_addr);
	
#ifdef __LOG_TRACE
	para = pdev[num]->chgr.para;
	set_read_chgr_para_from_ee( &para );
	delay_ms( EE_MS_DELAY );
	ee_ReadBytes( (uint8_t *)&flag, 0, 1 );
	delay_ms( EE_MS_DELAY );

    LOG_TRACE_1("\r\nafter save, real para(read from ee), dev:%d, init_flag:%02X, curve:%d, ichg:%f, itaper:%f, vbst:%f, vfloat:%f", num, flag, para.curve, \
	                                                                      para.data.ichg, \
	                                                                      para.data.itaper, \
		                                                                  para.data.vbst, \
	                                                                      para.data.vfloat );
#endif
}

static void set_save_chgr_para_syn( void )
{
	uint8_t idx = 0;
	uint8_t num = POWER_UNIT_1;
	float range = 1; // 读写误差
#ifdef __LOG_TRACE
	ChgrParaDef para;
	uint8_t flag = 0;
#endif
	LOG_TRACE_1("\r\n同步模式设置:");
	LOG_TRACE_1("\r\nbefore save, tmp_para, dev:%d, curve:%d, ichg:%f, itaper:%f, vbst:%f, vfloat:%f", num, \
	                                                                      phmi_info->set.tmp_para.curve, \
	                                                                      phmi_info->set.tmp_para.data.ichg, \
	                                                                      phmi_info->set.tmp_para.data.itaper, \
		                                                                  phmi_info->set.tmp_para.data.vbst, \
	                                                                      phmi_info->set.tmp_para.data.vfloat );
	
    // 第一步： 先把临时参数赋值给实际参数(注意：是curve和data，不包括ee_addr!!!)
	// 注意：这里的赋值方式一定要和 set_update_temp_curve_data()这个函数相对应，如果上面的函数是直接全部赋值para的，这里必须para
	// 如果上面的函数是分别赋值的，那么这里就要注意了，需要用分别赋值，这里不需要更新ee_addr，因为同步模式指定用模块1的ee_addr
	// 依次赋值给各模块
	// 同步模式，更新充电参数即可
	for(idx = 0; idx < POWER_UNIT_NUM; idx++ )
	{
	    pdev[ idx ]->chgr.para.curve = phmi_info->set.tmp_para.curve;	
	    pdev[ idx ]->chgr.para.data = phmi_info->set.tmp_para.data;	
	    LOG_TRACE_1("\r\nreal para, dev:%d, curve:%d, ichg:%f, itaper:%f, vbst:%f, vfloat:%f", idx, \
	                                                                      pdev[idx]->chgr.para.curve, \
	                                                                      pdev[idx]->chgr.para.data.ichg, \
	                                                                      pdev[idx]->chgr.para.data.itaper, \
		                                                                  pdev[idx]->chgr.para.data.vbst, \
	                                                                      pdev[idx]->chgr.para.data.vfloat );
	}
	
	// 第二步： 把实际参数写入功率模块
	// 向各模块依次写入充电参数
	for(idx = 0; idx < POWER_UNIT_NUM; idx++ )
	{

		LOG_TRACE_1("\r\n\r\npower unit %d", idx);
    	if ( set_write_chgr_para_to_unit_verify( WriteCURVE_ICHG, fReadCURVE_ICHG, pdev[idx]->basic.addr, \
    						 ( pdev[idx]->chgr.para.data.ichg ), range ) )
    	{ 
    		LOG_TRACE_1("\r\n write ichg success");
    	}
    	else
    	{ 
    		LOG_TRACE_1("\r\n write ichg failed");
    	}
    	
    	if ( set_write_chgr_para_to_unit_verify( WriteCURVE_ITAPER, fReadCURVE_ITAPER, pdev[idx]->basic.addr, \
    						 ( pdev[idx]->chgr.para.data.itaper ), range ) )
    	{
    		LOG_TRACE_1("\r\n write itaper success");
    	}
    	else
    	{ 
    		LOG_TRACE_1("\r\n write itaper failed");
    	}
    	
    	if ( set_write_chgr_para_to_unit_verify( WriteCURVE_VBST, fReadCURVE_VBST, pdev[idx]->basic.addr, \
    						 pdev[idx]->chgr.para.data.vbst, range ) )
    	{
    		LOG_TRACE_1("\r\n write vbst success");
    	}
    	else
    	{ 
    		LOG_TRACE_1("\r\n write vbst failed");
    	}
    	
    	if ( set_write_chgr_para_to_unit_verify( WriteCURVE_VFLOAT, fReadCURVE_VFLOAT, pdev[idx]->basic.addr, \
    						 pdev[idx]->chgr.para.data.vfloat, range ) )
    	{
    		LOG_TRACE_1("\r\n write vfloat success");
    	}
    	else
    	{
    		LOG_TRACE_1("\r\n write vfloat failed");
    	}	

	    // 第三步： 把参数写入eeprom保存，分别写入各个模块的eeprom空间
	    LOG_TRACE_1("\r\nbefore write, ee_addr:%04X, num:%d, ichg:%f", pdev[idx]->chgr.para.ee_addr, idx, pdev[idx]->chgr.para.data.ichg );
	    set_write_chgr_para_to_ee( &( pdev[idx]->chgr.para ) );
	
#ifdef __LOG_TRACE
	    para = pdev[num]->chgr.para;
	    set_read_chgr_para_from_ee( &para );
//	    delay_ms( EE_MS_DELAY );
        vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
	    ee_ReadBytes( (uint8_t *)&flag, 0, 1 );
//	    delay_ms( EE_MS_DELAY );
        vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

        LOG_TRACE_1("\r\nafter save, real para(read from ee), dev:%d, init_flag:%02X, curve:%d, ichg:%f, itaper:%f, vbst:%f, vfloat:%f", num, flag, para.curve, \
	                                                                      para.data.ichg, \
	                                                                      para.data.itaper, \
		                                                                  para.data.vbst, \
	                                                                      para.data.vfloat );

#endif
    } // end of for-loop

}

static uint8_t set_write_chgr_para_to_unit_verify( void (*write)(uint8_t addr, float para), float (*read)(uint8_t addr), \
	                                 uint8_t addr, float para_wr, float range )
{
	uint8_t max_cnt = 20;
	uint8_t idx = 0;
	float para_rd= 0;

	for ( idx = 0; idx < max_cnt; idx ++ )
	{
		write( addr, para_wr );
//		delay_ms( PMBUS_COMM_MS_DELAY );
		vTaskDelay( pdMS_TO_TICKS( PMBUS_COMM_MS_DELAY ) );
		
		para_rd = read( addr );
//		delay_ms( PMBUS_COMM_MS_DELAY );
		vTaskDelay( pdMS_TO_TICKS( PMBUS_COMM_MS_DELAY ) );
		LOG_TRACE_1("\r\n------------------>para_wr:%f, para_rd:%f<-------------------------------", para_wr, para_rd);
		
		// 如果读取的数据跟写入的数据在误差范围内，则认为写入成功
		if ( (((para_wr - para_rd) >= (float)0) && ((para_wr - para_rd) < range)) || \
             (((para_rd - para_wr) >= (float)0) && ((para_rd - para_wr) < range)) )
		{
			return TRUE;
		}
		else // 否则继续写，最多写10次
		{
			LOG_TRACE_1("\r\nwrite failed!!!,wr:%f, rd:%f", para_wr, para_rd);
		}
	}
	
	return FALSE;
}

static void set_save_pre_chgr_para( uint8_t num )
{
	// 把临时参数保存到模块
	pdev[ num ]->chgr.para.data_pre = phmi_info->set.tmp_para.data_pre;	

	LOG_TRACE_1("\r\nreal para, dev:%d, pre_iout:%f, pre_vout:%f, ichg:%f, vbst:%f, itaper:%f", num, \
	                                                                      pdev[num]->chgr.para.data_pre.pre_iout, \
	                                                                      pdev[num]->chgr.para.data_pre.pre_vout, \
	                                                                      pdev[num]->chgr.para.data_pre.ichg, \
		                                                                  pdev[num]->chgr.para.data_pre.vbst, \
	                                                                      pdev[num]->chgr.para.data_pre.itaper );

	// 把参数写入eeprom保存
	LOG_TRACE_1("\r\nbefore write, ee_addr:%04X, num:%d, ichg:%f", pdev[num]->chgr.para.ee_addr, num, pdev[num]->chgr.para.data.ichg );
	set_write_chgr_para_to_ee( &( pdev[num]->chgr.para ) );
}

static void set_save_pre_chgr_para_syn( void )
{
	uint8_t idx = 0;

	// 同步模式，更新充电参数
	for(idx = 0; idx < POWER_UNIT_NUM; idx++ )
	{
	    pdev[ idx ]->chgr.para.data_pre = phmi_info->set.tmp_para.data_pre;	
	    LOG_TRACE_1("\r\nreal para, dev:%d, pre_iout:%f, pre_vout:%f, ichg:%f, vbst:%f, itaper:%f", idx, \
	                                                                      pdev[idx]->chgr.para.data_pre.pre_iout, \
	                                                                      pdev[idx]->chgr.para.data_pre.pre_vout, \
	                                                                      pdev[idx]->chgr.para.data_pre.ichg, \
		                                                                  pdev[idx]->chgr.para.data_pre.vbst, \
	                                                                      pdev[idx]->chgr.para.data_pre.itaper );
	    // 把参数写入eeprom保存，分别写入各个模块的eeprom空间
	    LOG_TRACE_1("\r\nbefore write, ee_addr:%04X, num:%d, ichg:%f", pdev[idx]->chgr.para.ee_addr, idx, pdev[idx]->chgr.para.data.ichg );
	    set_write_chgr_para_to_ee( &( pdev[idx]->chgr.para ) );
	}
}

static void hmi_event(void)
{
	uint16_t addr = phmi_rx_frame->data.addr;
	uint16_t value = phmi_rx_frame->data.data[0];

	LOG_DEBUG_APP("\r\nhmi_event, addr:0x%04X, value:0x%04X", addr, value);

	switch( addr )
	{
		case 0x0033:    // 直接切换到某个模块
			if( value == 0x0011 ) // 表示显示模块1的当前设置
			{
			    event_disp_cur_setting( POWER_UNIT_1 );
				event_disp_cur_setting_num( POWER_UNIT_1 );
			}
			else if( value == 0x0022 )  // 表示显示模块2的当前设置
			{
			    event_disp_cur_setting( POWER_UNIT_2 );
				event_disp_cur_setting_num( POWER_UNIT_2 );
			}
			else if( value == 0x0033 )  // 表示显示模块3的当前设置
			{
			    event_disp_cur_setting( POWER_UNIT_3 );
				event_disp_cur_setting_num( POWER_UNIT_3 );
			}
			else if( value == 0x0044 )  // 表示显示模块4的当前设置
			{
			    event_disp_cur_setting( POWER_UNIT_4 );
				event_disp_cur_setting_num( POWER_UNIT_4 );
			}
			else if( value == 0x0055 )  // 表示显示模块5的当前设置
			{
			    event_disp_cur_setting( POWER_UNIT_5 );
				event_disp_cur_setting_num( POWER_UNIT_5 );
			}
			else if( value == 0x0066 )  // 表示显示模块6的当前设置
			{
			    event_disp_cur_setting( POWER_UNIT_6 );
				event_disp_cur_setting_num( POWER_UNIT_6 );
			}
			else if( value == 0x0077 )  // 表示显示模块7的当前设置
			{
			    event_disp_cur_setting( POWER_UNIT_7 );
				event_disp_cur_setting_num( POWER_UNIT_7 );
			}
			else if( value == 0x0088 )  // 表示显示模块8的当前设置
			{
			    event_disp_cur_setting( POWER_UNIT_8 );
				event_disp_cur_setting_num( POWER_UNIT_8 );
			}
			else if( value == 0x0099 ) // 表示显示模块1的历史事件
			{
				phmi_info->event.cur_num = POWER_UNIT_1;

				event_disp_cur_history_num( POWER_UNIT_1 );
			    event_disp_cur_history( POWER_UNIT_1, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x00AA )  // 表示显示模块2的历史事件
			{
				phmi_info->event.cur_num = POWER_UNIT_2;

				event_disp_cur_history_num( POWER_UNIT_2 );
			    event_disp_cur_history( POWER_UNIT_2, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x00BB )  // 表示显示模块3的历史事件
			{
				phmi_info->event.cur_num = POWER_UNIT_3;

				event_disp_cur_history_num( POWER_UNIT_3 );
			    event_disp_cur_history( POWER_UNIT_3, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x00CC )  // 表示显示模块4的历史事件
			{
				phmi_info->event.cur_num = POWER_UNIT_4;

				event_disp_cur_history_num( POWER_UNIT_4 );
			    event_disp_cur_history( POWER_UNIT_4, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x00DD )  // 表示显示模块5的历史事件
			{
				phmi_info->event.cur_num = POWER_UNIT_5;

				event_disp_cur_history_num( POWER_UNIT_5 );
			    event_disp_cur_history( POWER_UNIT_5, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x00EE )  // 表示显示模块6的历史事件
			{
				phmi_info->event.cur_num = POWER_UNIT_6;

				event_disp_cur_history_num( POWER_UNIT_6 );
			    event_disp_cur_history( POWER_UNIT_6, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x00FF )  // 表示显示模块7的历史事件
			{
				phmi_info->event.cur_num = POWER_UNIT_7;

				event_disp_cur_history_num( POWER_UNIT_7 );
			    event_disp_cur_history( POWER_UNIT_7, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x0100 )  // 表示显示模块8的历史事件
			{
				phmi_info->event.cur_num = POWER_UNIT_8;

				event_disp_cur_history_num( POWER_UNIT_8 );
			    event_disp_cur_history( POWER_UNIT_8, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x0121 )  // 左翻 
			{
                event_history_sw_page( phmi_info->event.cur_num, SW_TO_LEFT );
			}
			else if( value == 0x0122 )  // 右翻 
			{
                event_history_sw_page( phmi_info->event.cur_num, SW_TO_RIGHT );
			}
			break;
		case 0x1818: // 通过按钮左右移动切换到某个模块，显示对应的当前设置 
			event_disp_cur_setting( (value - 1) );
			break;
		case 0x189B: // 通过按钮左右移动切换到某个模块，显示对应的历史事件 
		    phmi_info->event.cur_num = value - 1; 
            phmi_info->event.page_num = 0;
			event_disp_cur_history( (value - 1), 0 );
			break;
	    default:
	    	break;
	}
}

static void event_disp_cur_setting( uint8_t num )
{
	uint16_t var_addr_list[] = { 0x1811, 0x1812, 0x1813, 0x1814, 0x1815, 0x1816 };
	uint16_t ver_addr_list_pre[] = { 0x189B, 0x189C, 0x189D, 0x189E, 0x189F };
	uint16_t addr;
	uint16_t data;
	
	LOG_DEBUG_APP("\r\nnum-%d, cur set: %f, %f, %f, %f, ", num, \
			                                               	 	pdev[num]->chgr.para.data.ichg, \
			                                               	 	pdev[num]->chgr.para.data.itaper, \
			                                               	 	pdev[num]->chgr.para.data.vbst, \
			                                               	 	pdev[num]->chgr.para.data.vfloat );

	// 显示均充电流
	addr = var_addr_list[0];  // 变量地址
	data = (uint16_t)(pdev[num]->chgr.para.data.ichg * 10) & 0xffff;  // 把浮点数转换成16进制数，保留一位小数
	hmi_disp_para_data( addr, data );
	// 显示均转电流
	addr = var_addr_list[1];  // 变量地址
	data = (uint16_t)(pdev[num]->chgr.para.data.itaper * 10) & 0xffff;  // 把浮点数转换成16进制数，保留一位小数
	hmi_disp_para_data( addr, data );
	// 显示终止电压
	addr = var_addr_list[2];  // 变量地址
	data = (uint16_t)(pdev[num]->chgr.para.data.vbst * 10) & 0xffff;  // 把浮点数转换成16进制数，保留一位小数
	hmi_disp_para_data( addr, data );
	// 显示浮充电压
	addr = var_addr_list[3];  // 变量地址
	data = (uint16_t)(pdev[num]->chgr.para.data.vfloat * 10) & 0xffff;  // 把浮点数转换成16进制数，保留一位小数
	hmi_disp_para_data( addr, data );


	// 显示预充电流
	addr = ver_addr_list_pre[0];  // 变量地址
	data = (uint16_t)(pdev[num]->chgr.para.data_pre.pre_iout * 10) & 0xffff;  // 把浮点数转换成16进制数，保留一位小数
	hmi_disp_para_data( addr, data );
	// 显示预充电压
	addr = ver_addr_list_pre[1];  // 变量地址
	data = (uint16_t)(pdev[num]->chgr.para.data_pre.pre_vout * 10) & 0xffff;  // 把浮点数转换成16进制数，保留一位小数
	hmi_disp_para_data( addr, data );
	// 显示恒充电流
	addr = ver_addr_list_pre[2];  // 变量地址
	data = (uint16_t)(pdev[num]->chgr.para.data_pre.ichg * 10) & 0xffff;  // 把浮点数转换成16进制数，保留一位小数
	hmi_disp_para_data( addr, data );
	// 显示恒充电压
	addr = ver_addr_list_pre[3];  // 变量地址
	data = (uint16_t)(pdev[num]->chgr.para.data_pre.vbst * 10) & 0xffff;  // 把浮点数转换成16进制数，保留一位小数
	hmi_disp_para_data( addr, data );
	// 显示终止电流
	addr = ver_addr_list_pre[4];  // 变量地址
	data = (uint16_t)(pdev[num]->chgr.para.data_pre.itaper * 10) & 0xffff;  // 把浮点数转换成16进制数，保留一位小数
	hmi_disp_para_data( addr, data );



	// 显示定时充电时间
	addr = var_addr_list[4];
	if( pdev[num]->chgr.st_timing_time != DISABLE )
		data = pdev[num]->chgr.timing_time;
	else
		data = 0;
	hmi_disp_para_data( addr, data );
	LOG_DEBUG_APP_1("timing time:%d, ", data);
	
	// 显示预约充电时间	
	addr = var_addr_list[5];
	if( pdev[num]->chgr.st_booking_time != DISABLE )
		data = pdev[num]->chgr.booking_time;
	else
		data = 0;
	hmi_disp_para_data( addr, data );

	LOG_DEBUG_APP_1("booking time:%d", data);
}

static void event_disp_cur_setting_num( uint8_t num )
{
    uint16_t addr = 0x1818;
	uint16_t data = num + 1;

	hmi_disp_para_data( addr, data);
}

static void event_disp_cur_history_num( uint8_t num )
{
    uint16_t addr = 0x189B;
	uint16_t data = num + 1;

	hmi_disp_para_data( addr, data);
}

static void event_history_sw_page( uint8_t num, uint8_t sw_dir )
{
    if( sw_dir == SW_TO_RIGHT )
	{
		LOG_DEBUG_APP("\r\nevent, sw page, to right, ");
		phmi_info->event.page_num ++;
	}
	else if( sw_dir == SW_TO_LEFT )
	{
		LOG_DEBUG_APP("\r\nevent, sw page, to left, ");
		phmi_info->event.page_num --;
	}

	if( phmi_info->event.page_num > EVENT_MAX_PAGE_NUM - 1 )
		phmi_info->event.page_num = 0;
	else if( phmi_info->event.page_num < 0  )
		phmi_info->event.page_num = EVENT_MAX_PAGE_NUM - 1;

	// debug
	if( (phmi_info->event.page_num < 0) || (phmi_info->event.page_num > EVENT_MAX_PAGE_NUM ) )
	{
	    LOG_DEBUG_APP_1("debug------------>num:%d, page:%d", num, phmi_info->event.page_num );
	    return;
	}

	LOG_DEBUG_APP_1("num:%d, page:%d", num, phmi_info->event.page_num );

    event_disp_cur_history( num, phmi_info->event.page_num );
}

//BOOL read_event( PU_InfoDef *pdev, EE_SysEventDef *pevent, uint8_t page_num, uint8_t num_to_read, uint8_t *p_num_actual_read )
static void event_disp_cur_history( uint8_t num, uint8_t page_num )
{
	uint8_t idx = 0;
	EE_SysEventDef event[EVENT_EACH_PAGE_NUM];
	uint8_t num_actual_read;
	uint8_t line_base_num = 0;

    //  每页的第一个序号
	line_base_num = page_num * EVENT_EACH_PAGE_NUM; 

	// 显示前，需要清除当前页面残留的数据，无论该页是否有历史记录
	for( idx = 0; idx < EVENT_EACH_PAGE_NUM; idx++ )
	{
        event_disp_clear_one_line( idx, (line_base_num++) );
	}

	// 如果有历史记录，显示 
	if( read_event( pdev[num], event, page_num, EVENT_EACH_PAGE_NUM, &num_actual_read ) )
	{
	    line_base_num = page_num * EVENT_EACH_PAGE_NUM; 

	    for( idx = 0; idx < num_actual_read; idx++ )
		{
			// 显示历史事件
		    event_disp_one_line( idx, (line_base_num++), event[idx] );
		}
	}

	LOG_DEBUG_APP("\r\nnum: %d, page_num:%d, num_actual_read:%d", num, page_num, num_actual_read);
}

static void event_disp_clear_one_line( uint8_t line, uint8_t num )
{
	uint16_t num_addr_list[] = { 0x181d, 0x181e, 0x181f, 0x1820 };
	uint16_t event_addr_list[] = { 0x1821, 0x1830, 0x183f, 0x184e };
	uint16_t time_addr_list[] = { 0x185d, 0x186c, 0x187b, 0x188a };

	uint8_t temp_str[20] = {'a'};
	memset(temp_str, ' ', 20);

	// 显示序号，清空时，序号仍然正常显示
	LOG_DEBUG_APP("\r\nclear event, line:%d, disp num:%d", line, num);
	hmi_disp_para_data( num_addr_list[ line ], (uint16_t)num );

	// 清空事件
	hmi_disp_string( event_addr_list[ line ], temp_str, sizeof( temp_str ) );

	// 清空时间
	hmi_disp_string( time_addr_list[ line ], temp_str, sizeof( temp_str ) );
}

static void event_disp_one_line( uint8_t line, uint8_t num, EE_SysEventDef event )
{
	uint16_t num_addr_list[] = { 0x181d, 0x181e, 0x181f, 0x1820 };
	uint16_t event_addr_list[] = { 0x1821, 0x1830, 0x183f, 0x184e };
	uint16_t time_addr_list[] = { 0x185d, 0x186c, 0x187b, 0x188a };
	uint8_t temp_str[20] = {0};
	uint8_t temp_str_idx = 0;
	uint8_t len = 0;
	uint8_t idx = 0;
	uint8_t event_code = 0;
	uint8_t mode = 0;
	uint8_t para = 0;

	// 显示前，先清除该行，防止残留
    event_disp_clear_one_line( line, num );


	// 显示序号
	hmi_disp_para_data( num_addr_list[ line ], num );

	// 显示事件
	event_code = (event.code >> 8) & 0xFF;

	len = strlen( Event_history_tab[ event_code ] );
	for( idx = 0; idx < len; idx ++)
	{
	    temp_str[ temp_str_idx++ ] = Event_history_tab[ event_code ][idx];
	}

	LOG_DEBUG_APP("\r\nevent code:0x%04X", event.code);

	if( event_code == EVENT_START )
	{
	    mode = ( ((uint8_t)event.code) & 0xF0 ) >> 4;
	    para = ((uint8_t)event.code) & 0x0F;
	    LOG_DEBUG_APP("\r\nmode:%d, para:%d", event_code, para);

	    temp_str[ temp_str_idx++ ] = '(';

		// 添加模式
		len = strlen( Event_chgr_mode_tab[ mode ] );
		for( idx = 0; idx < len; idx ++ )
		{
		    temp_str[ temp_str_idx++ ]  = Event_chgr_mode_tab[mode][idx];
		}
		temp_str[ temp_str_idx++ ] = '-';

		// 添加参数
		len = strlen( Event_chgr_para_tab[ para ] );
		LOG_DEBUG_APP("\r\npara len:%d, para:%s", len, Event_chgr_para_tab[ para ] );
		for( idx = 0; idx < len; idx ++ )
		{
		    temp_str[ temp_str_idx++ ]  = Event_chgr_para_tab[para][idx];
		}
		temp_str[ temp_str_idx++ ] = ')';
		temp_str[ temp_str_idx ] = '\0';

	}
	else
	{
		temp_str[ temp_str_idx ] = '\0';
	}

	hmi_disp_string( event_addr_list[ line ], temp_str, temp_str_idx );

	// 显示时间
	temp_str[0] = '2';
	temp_str[1] = '0';
	temp_str_idx = 2;
	sprintf( (char *)&temp_str[temp_str_idx], "%02x", event.sys_time.year);
	temp_str_idx += 2;
    temp_str[ temp_str_idx++ ] = '-';

	sprintf( (char *)&temp_str[temp_str_idx], "%02x", event.sys_time.month);
	temp_str_idx += 2;
    temp_str[ temp_str_idx++ ] = '-';
	
	sprintf( (char *)&temp_str[temp_str_idx], "%02x", event.sys_time.day);
	temp_str_idx += 2;
    temp_str[ temp_str_idx++ ] = ' ';

	sprintf( (char *)&temp_str[temp_str_idx], "%02x", event.sys_time.hour);
	temp_str_idx += 2;
    temp_str[ temp_str_idx++ ] = ':';

	sprintf( (char *)&temp_str[temp_str_idx], "%02x", event.sys_time.min);
	temp_str_idx += 2;
    temp_str[ temp_str_idx++ ] = ':';

	sprintf( (char *)&temp_str[temp_str_idx], "%02x", event.sys_time.sec);
	temp_str_idx += 2;

    temp_str[ temp_str_idx ] = '\0';

	hmi_disp_string( time_addr_list[ line ], temp_str, temp_str_idx );
}

static void hmi_disp_string( uint16_t addr, uint8_t *str, uint8_t len )
{
	uint8_t idx = 0;
    uint8_t buff[] = { 0x5A, 0xA5, 0x00, 0x82, 0x00, 0x00 };
	
	buff[2] = len + 3;

	buff[4] = (addr>>8) & 0xff;
	buff[5] = addr & 0xff;
		
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );

	while( *str != '\0' )
	{
	    com_touch_screen->write( *str++ );
	}
	
	// 发送出去
	com_touch_screen->send();

}

static void hmi_maintain()
{
	uint16_t addr = phmi_rx_frame->data.addr;
	uint16_t value = phmi_rx_frame->data.data[0];


	switch( addr )
	{
		case 0x0044:
			if( value == 0x0002 )  // 表示切换到维护，密码输入完成，准备进入维护里面，此时判断输入的密码是否正确
			{
			    maintain_enter();
				hmi_update_page_id ( HMI_MAINTAIN_ENTER_ID );
	            phmi_info->maintain.cur_num = POWER_UNIT_1;
			}
			else if ( value == 0x0011 )  // 表示显示模块1的维护信息
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_1;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_1_ID );

			    maintain_disp_detail( POWER_UNIT_1 );
			}
			else if ( value == 0x0022 )  // 表示显示模块2的维护信息
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_2;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_2_ID );

			    maintain_disp_detail( POWER_UNIT_2 );
			}
			else if ( value == 0x0033 )  // 表示显示模块3的维护信息
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_3;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_3_ID );

			    maintain_disp_detail( POWER_UNIT_3 );
			}
			else if ( value == 0x0044 )  // 表示显示模块4的维护信息
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_4;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_4_ID );

			    maintain_disp_detail( POWER_UNIT_4 );
			}
			else if ( value == 0x0055 )  // 表示显示模块5的维护信息
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_5;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_5_ID );

			    maintain_disp_detail( POWER_UNIT_5 );
			}
			else if ( value == 0x0066 )  // 表示显示模块6的维护信息
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_6;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_6_ID );

			    maintain_disp_detail( POWER_UNIT_6 );
			}
			else if ( value == 0x0077 )  // 表示显示模块7的维护信息
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_7;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_7_ID );

			    maintain_disp_detail( POWER_UNIT_7 );
			}
			else if ( value == 0x0088 )  // 表示显示模块8的维护信息
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_8;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_8_ID );

			    maintain_disp_detail( POWER_UNIT_8 );
			}
			break;
		case 0x2011: //  表示输入了密码，更新输入的密码，把输入的密码保存到临时变量里面
			maintain_update_tmp_input_passwd();
			break;
		default:
			break;
	}

}

static void maintain_update_tmp_input_passwd( void )
{
	uint8_t idx;
	uint8_t input_len = 0;
    uint8_t tmp[ PASSWD_LEN_MAINTAIN ];

	for( idx = 0, input_len = 0; idx < phmi_rx_frame->data.len; idx ++ )
	{
		tmp[ input_len ++ ] = ( phmi_rx_frame->data.data[idx] >> 8 ) & 0xff;

	    if ( ( phmi_rx_frame->data.data[idx] & 0xff ) != 0xff )
		{
		    tmp[ input_len ++ ] = phmi_rx_frame->data.data[idx] & 0xff;
		}
	}
	
	for( idx = 0; idx < input_len; idx ++ )
	{
		phmi_info->set.tmp_maintain_passwd[idx] = tmp[idx];	    
	}
//	memmove( phmi_info->set.tmp_maintain_passwd, tmp, input_len );
    phmi_info->set.tmp_maintain_passwd[ input_len ] = '\0';

	LOG_DEBUG_APP("\r\nupdate_tmp_maintain_input_passwd, input_len:%d, input passwd:%s", input_len, phmi_info->set.tmp_maintain_passwd );
}

static void maintain_enter( void )
{
    uint8_t passwd[ PASSWD_LEN_MAINTAIN+1 ] = {0};	

    // 读取当前存储在eeprom中的设置密码
    maintain_read_passwd( passwd );
    passwd[ PASSWD_LEN_MAINTAIN ] = '\0';

	// 检测输入的密码是否正确
	if( strcmp( (char *)phmi_info->set.tmp_maintain_passwd, (char *)passwd ) == 0 )
	{
		// 进入设置
	    hmi_switch_to_spec_page( HMI_MAINTAIN_ENTER_ID );
	}
	else
	{
		// 弹出密码输入错误界面
	    hmi_switch_to_spec_page( HMI_MAINTAIN_PASSWD_ERR_ID );
	}

	// 清除临时保存输入密码的缓冲区
	memset((char *)phmi_info->set.tmp_maintain_passwd, 0xff, PASSWD_LEN_MAINTAIN+1 );
}

static void maintain_read_passwd( uint8_t *passwd ) 
{
    ee_ReadBytes( passwd, EE_PASSWD_MAINTAIN, PASSWD_LEN_MAINTAIN );

	delay_ms( EE_MS_DELAY );
}

static void maintain_save_passwd( uint8_t *passwd )
{
    ee_WriteBytes( passwd, EE_PASSWD_MAINTAIN, PASSWD_LEN_MAINTAIN );

	delay_ms( EE_MS_DELAY );
}

static void maintain_disp_detail( uint8_t num )
{
	uint16_t disp_data = 0;
	uint16_t disp_addr = 0;
	
	// 充饱电状态, 定电流状态，定电压状态，浮充状态，电池连接，定电流阶段超时，定电压阶段超时，定浮充阶段超时
	uint16_t chgr_status_disp_addr[]={0x2041, 0x2045, 0x2049, 0x204D, 0x2051, 0x2055, 0x2059, 0x205D};
	uint8_t idx;
	
	// 显示输入电压
	disp_addr = 0x2031;
	if( pdev[num]->basic.flag_ac_power != BASIC_POWER_OFF )
	    disp_data = ReadVin( pdev[num]->basic.addr );
	else
	    disp_data = 0;
	hmi_disp_para_data( disp_addr, disp_data );
	LOG_DEBUG_APP("\r\nvin:%d", disp_data);

	// 显示输出电压
	disp_addr = 0x2033;
	if( pdev[num]->basic.flag_ac_power != BASIC_POWER_OFF )
	    disp_data = pdev[num]->chgr.vout * 10;  // 保留一位小数，所以显示的时候乘以10
	else
	    disp_data = 0; 
	hmi_disp_para_data( disp_addr, disp_data );
	LOG_DEBUG_APP_1( "vout: %f, ", pdev[num]->chgr.vout );

	// 显示风扇转速, 暂定为显示风扇1的转速
	disp_addr = 0x2035;
	if( pdev[num]->basic.flag_ac_power != BASIC_POWER_OFF )
//	    disp_data = ReadFanSpeed_1( pdev[num]->basic.addr );
	    disp_data = pdev[num]->basic.fan_speed;
	else
	    disp_data = 0; 
	hmi_disp_para_data( disp_addr, disp_data );
	LOG_DEBUG_APP_1( "fan_speed_1: %d, ", disp_data );

	// 模块温度
	disp_addr = 0x2037;
	if( pdev[num]->basic.flag_ac_power != BASIC_POWER_OFF )
//	    disp_data = ReadTemperature( pdev[num]->basic.addr );
	    disp_data = pdev[num]->basic.temp;
	else
	    disp_data = 0;
	hmi_disp_para_data( disp_addr, disp_data );
	LOG_DEBUG_APP_1( "temp: %d, ", disp_data );

	// 输入过压
//	disp_addr = 0x2039;
//	maintain_disp_chgr_status( disp_addr, "否" );

	// 输出过压
//	disp_addr = 0x203B;
//	maintain_disp_chgr_status( disp_addr, "否" );

	// 模块过温
	disp_addr = 0x203D;
	if( pdev[num]->basic.err & bit(ERR_T_ALARM) )
	    maintain_disp_chgr_status( disp_addr, "是" );
	else
	    maintain_disp_chgr_status( disp_addr, "否" );	

	// 输入异常
	//disp_addr = 0x203F;
	//maintain_disp_chgr_status( disp_addr, "否" );

	// 显示充电状态
	if( pdev[num]->basic.chgr == BASIC_CHGR_OFF )    // 未处于充电状态，全部显示否
	{
	    for( idx = 0; idx < sizeof(chgr_status_disp_addr)/sizeof(uint16_t); idx ++ )
	        maintain_disp_chgr_status( chgr_status_disp_addr[idx], "否" ); 
	}
	else if( pdev[num]->basic.bat == BASIC_BAT_CONN )    // 如果电池连接，则电池连接显示：是
	{
	    maintain_disp_chgr_status( chgr_status_disp_addr[4], "是" );
	}
	else    // 处于充电状态中
	{
		// 是否充饱
	    if( pdev[num]->chgr.status & FULLM )
	        maintain_disp_chgr_status( chgr_status_disp_addr[0], "是" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[0], "否" );

		// 定电流状态
	    if( pdev[num]->chgr.status & CCM )
	        maintain_disp_chgr_status( chgr_status_disp_addr[1], "是" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[1], "否" );

		// 定电压状态
	    if( pdev[num]->chgr.status & CVM )
	        maintain_disp_chgr_status( chgr_status_disp_addr[2], "是" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[2], "否" );

		// 浮充状态
	    if( pdev[num]->chgr.status & FVM )
	        maintain_disp_chgr_status( chgr_status_disp_addr[3], "是" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[3], "否" );

		// 电池连接状态
	    if( pdev[num]->chgr.status & BTNC )
	        maintain_disp_chgr_status( chgr_status_disp_addr[4], "是" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[4], "否" );

		// 定电流阶段超时
	    if( pdev[num]->chgr.status & CCTOF )
	        maintain_disp_chgr_status( chgr_status_disp_addr[5], "是" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[5], "否" );

		// 定电压阶段超时
	    if( pdev[num]->chgr.status & CVTOF )
	        maintain_disp_chgr_status( chgr_status_disp_addr[6], "是" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[6], "否" );

		// 定浮充阶段超时
	    if( pdev[num]->chgr.status & FVTOF )
	        maintain_disp_chgr_status( chgr_status_disp_addr[7], "是" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[7], "否" );
	}
	
    LOG_DEBUG_APP("\r\ncur_num:%d, page_id:%d", num, phmi_info->page.id.cur );
}

static void maintain_update_detail(void)
{
	if( ( phmi_info->page.id.cur >= HMI_MAINTAIN_DETAIL_1_ID ) && \
		( phmi_info->page.id.cur <= HMI_MAINTAIN_DETAIL_8_ID ) )
	{
        maintain_disp_detail( phmi_info->maintain.cur_num );
	}
}

static void maintain_disp_chgr_status( uint16_t addr, char *str )
{
	uint8_t idx = 0;
    uint8_t buff[] = { 0x5A, 0xA5, 0x05, 0x82, 0x00, 0x00 };
	
	buff[4] = (addr>>8) & 0xff;
	buff[5] = addr & 0xff;
	
	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	while (*str != '\0')
	    com_touch_screen->write( *str ++ );
	
	// 发送出去
	com_touch_screen->send();
}

static void hmi_help(void)
{
	uint16_t addr = phmi_rx_frame->data.addr;
	uint16_t value = phmi_rx_frame->data.data[0];
	char hmi_ver[] = VER_HMI;
	char sw_ver[] = VER_SW;
    
	switch( addr )
	{
	    case 0x0055:
			if( ( value == 0x0002 ) || ( value == 0x0001 ) )
			{
			    hmi_disp_string( 0x2811, (uint8_t *)hmi_ver, strlen(hmi_ver) );
			    hmi_disp_string( 0x2822, (uint8_t *)sw_ver, strlen(sw_ver) );
			    hmi_disp_string( 0x2844, (uint8_t *)phmi_info->serial_num, EE_SERIAL_NUM_LEN );
			    hmi_disp_string( 0x2833, (uint8_t *)phmi_info->product_code, EE_PRODUCT_CODE_LEN );
			}
			break;
		default:
			break;
	}
}

static DevListDef *create_dev_list( uint8_t num )
{
    DevListDef *phead = NULL;
	DevListDef *pnew = NULL;
	uint32_t mem_size;	

	if( __LOG_DEBUG_APP )
	{
	    mem_size = xPortGetFreeHeapSize();
	    LOG_DEBUG_FILE_APP("\r\n系统当前内存大小为：%ld 字节", mem_size);
	}
	
	LOG_DEBUG_APP("\r\ncreate_devicelist, node_size:%d, node_whole_num:%d", sizeof(DevListDef), num);
	if( NULL == (phead = (DevListDef *)pvPortMalloc(sizeof(DevListDef))) ) { LOG_DEBUG_FILE_APP("\r\nget mem failed"); return NULL; }
	
	// 创建设备链表，采用头插法
	phead->next = NULL;
	while( num -- )
	{
	    if( NULL != (pnew = (DevListDef *)pvPortMalloc(sizeof(DevListDef))) )
		{
		    pnew->next = phead->next;
			phead->next = pnew;
		}
		else
		{
			return NULL;
		}
	}

	if( __LOG_DEBUG_APP )
	{
	    mem_size = xPortGetFreeHeapSize();
	    LOG_DEBUG_FILE_APP("\r\n系统剩余内存大小为：%ld 字节", mem_size );
	}
	
	return phead;
}

static void init_device_list(void)
{ uint8_t idx = 0;
	DevListDef *pnode;

	// 创建设备列表
	if( NULL == ( phead_device_list = create_dev_list( POWER_UNIT_NUM ) ) )
	{
		LOG_DEBUG_APP("\r\n设备列表创建失败");
        return;
	}

	// 初始化指向这些设备的指针数值
	pnode = phead_device_list;
	for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
	{
		pnode = pnode->next;
		pdev[idx] = &pnode->dev;
	}
}

static void clear_history_event(void)
{
	uint8_t idx = 0;

	uint8_t flag_full = 0;
	uint16_t flag_full_addr = EE_EVENT_IS_FULL_ADDR;

	uint8_t latest_idx = EVENT_MAX_STORE_NUM - 1; // 最新序号从最大值开始
	uint16_t latest_idx_base_addr = EE_EVENT_LATEST_IDX_BASE_ADDR;

    // 清除事件相关
    ee_WriteBytes( &flag_full, flag_full_addr, EE_EVENT_IS_FULL_LEN);
    delay_ms( EE_MS_DELAY );

	for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
	{
	    ee_WriteBytes( &latest_idx, latest_idx_base_addr + idx*EE_EVENT_LATEST_IDX_LEN, EE_EVENT_LATEST_IDX_LEN);
	    delay_ms( EE_MS_DELAY );
	}
}

static BOOL init_smp_calibrated( uint8_t num )
{
	uint8_t flag = 0;
	uint16_t  rd_calib_val = 0;
	float calib = 0;

#if 0
	// 屏蔽校验
	pdev[num]->basic.smp_bat_calibrate_value = 0;
	return TRUE;
#endif

	// 读取是否执行过校准的标志
	ee_ReadBytes( &flag, (ee_chgr_para_addr[num] + EE_OFFSET_SMP_CALIBRATED), 1 );
	delay_ms( EE_MS_DELAY );

	if( flag == TRUE )
	{
		// 表示执行过采样校准
		pdev[num]->basic.flag_smp_calibrated = TRUE; 

		// 读取校准值
	    ee_ReadBytes( (uint8_t *)&rd_calib_val, (ee_chgr_para_addr[num]+ EE_OFFSET_SMP_CALIBRATE_VAL), LEN_SMP_CALIBRATE_VAL );
	    delay_ms( EE_MS_DELAY );
		calib = (float)(int16_t)rd_calib_val/100;
		LOG_DEBUG_APP("\r\n执行过电压采样校准, 校准值为:%.2f", calib);
	}
	else
	{
        // 表示没有执行过采样校准
		pdev[num]->basic.flag_smp_calibrated = FALSE; 

		calib = 0;
		LOG_DEBUG_APP("\r\n没有执行过电压采样校准, 设置校准值为0" );
	}

	// 过滤非法的校准值
	if( (calib > 10) || (calib + 10 < 0) )
	{
		pdev[num]->basic.flag_smp_calibrated = FALSE; 
		calib = 0;
		LOG_DEBUG_APP("\r\n校准值=%.2f，为非法值，设置为0", calib );
	}

	pdev[num]->basic.smp_bat_calibrate_value = calib;
}

static void init_power_unit_info(void)
{
	uint8_t idx = 0;
	DevListDef *pnode;

	for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
	{
		pdev[idx]->basic.num = idx; // 0,1,2,3,4,5,6,7
    	pdev[idx]->basic.err = BASIC_ERR_NO;
		pdev[idx]->basic.warn = BASIC_WARN_NO;
    	pdev[idx]->basic.chgr = BASIC_CHGR_OFF;
		pdev[idx]->basic.restart = BASIC_RESTART_NO;
    	pdev[idx]->basic.bat = BASIC_BAT_DISCONN;
    	pdev[idx]->basic.addr = addr_list[idx];
    	pdev[idx]->basic.temp = 25;
    	pdev[idx]->basic.temp_before = 25;
    	pdev[idx]->basic.fan_speed = 0;
    	pdev[idx]->basic.bat_volt = 0;
    	pdev[idx]->basic.flag_ac_power = BASIC_POWER_OFF;
    	pdev[idx]->basic.flag_delay_close_ac = OFF;
		pdev[idx]->basic.flag_bms_conn = OFF;
		pdev[idx]->basic.flag_mode_switch = BASIC_MODE_NONE_SWITCH;


		// 初始化过电压采样校准
        init_smp_calibrated( idx );

		// 初始化充电参数的存储地址
		// 一定要放在初始化充电参数之前
		pdev[idx]->chgr.para.ee_addr = ee_chgr_para_addr[idx]; 
		
		// 初始化充电参数的上限和下限，一定要放在初始化充电参数之前，因为is_been_init里面要用
		    // 三段式曲线充电参数 
		pdev[idx]->chgr.data_max.ichg   = curve_data_mm[CHGR_PARA_DATA_MAX][IDX_ICHG];
		pdev[idx]->chgr.data_max.itaper = curve_data_mm[CHGR_PARA_DATA_MAX][IDX_ITAPER];
		pdev[idx]->chgr.data_max.vbst   = curve_data_mm[CHGR_PARA_DATA_MAX][IDX_VBST];
		pdev[idx]->chgr.data_max.vfloat = curve_data_mm[CHGR_PARA_DATA_MAX][IDX_VFLOAT];
		pdev[idx]->chgr.data_min.ichg   = curve_data_mm[CHGR_PARA_DATA_MIN][IDX_ICHG];
		pdev[idx]->chgr.data_min.itaper = curve_data_mm[CHGR_PARA_DATA_MIN][IDX_ITAPER];
		pdev[idx]->chgr.data_min.vbst   = curve_data_mm[CHGR_PARA_DATA_MIN][IDX_VBST];
		pdev[idx]->chgr.data_min.vfloat = curve_data_mm[CHGR_PARA_DATA_MIN][IDX_VFLOAT];	
		
		    // 预充电充电参数
		pdev[idx]->chgr.pre_data_max.pre_iout = pre_chgr_data_mm[CHGR_PARA_DATA_MAX][ IDX_PRE_CHGR_PRE_IOUT ];
		pdev[idx]->chgr.pre_data_max.pre_vout = pre_chgr_data_mm[CHGR_PARA_DATA_MAX][ IDX_PRE_CHGR_PRE_VOUT ];
		pdev[idx]->chgr.pre_data_max.ichg = pre_chgr_data_mm[CHGR_PARA_DATA_MAX][ IDX_PRE_CHGR_ICHG ];
		pdev[idx]->chgr.pre_data_max.vbst = pre_chgr_data_mm[CHGR_PARA_DATA_MAX][ IDX_PRE_CHGR_VBST ];
		pdev[idx]->chgr.pre_data_max.itaper = pre_chgr_data_mm[CHGR_PARA_DATA_MAX][ IDX_PRE_CHGR_ITAPER ];
		
		pdev[idx]->chgr.pre_data_min.pre_iout = pre_chgr_data_mm[CHGR_PARA_DATA_MIN][ IDX_PRE_CHGR_PRE_IOUT ];
		pdev[idx]->chgr.pre_data_min.pre_vout = pre_chgr_data_mm[CHGR_PARA_DATA_MIN][ IDX_PRE_CHGR_PRE_VOUT ];
		pdev[idx]->chgr.pre_data_min.ichg = pre_chgr_data_mm[CHGR_PARA_DATA_MIN][ IDX_PRE_CHGR_ICHG ];
		pdev[idx]->chgr.pre_data_min.vbst = pre_chgr_data_mm[CHGR_PARA_DATA_MIN][ IDX_PRE_CHGR_VBST ];
		pdev[idx]->chgr.pre_data_min.itaper = pre_chgr_data_mm[CHGR_PARA_DATA_MIN][ IDX_PRE_CHGR_ITAPER ];

		/* 初始化充电参数
		 */
		// 如果初始化过，直接从eeprom里面读取参数
		if( is_been_init( idx ) )
		{
			pdev[idx]->chgr.para.curve = 0;
			pdev[idx]->chgr.para.data.ichg = 0;
			pdev[idx]->chgr.para.data.itaper = 0;
			pdev[idx]->chgr.para.data.vbst = 0;
			pdev[idx]->chgr.para.data.vfloat = 0;
			
			set_read_chgr_para_from_ee( &( pdev[idx]->chgr.para) );
			
			LOG_TRACE("\r\nhave been init,cur_num:%d , curve:%d, ee_addr:%04X, %f, %f, %f ,%f",  pdev[idx]->basic.num, pdev[idx]->chgr.para.curve, pdev[idx]->chgr.para.ee_addr, \
			pdev[idx]->chgr.para.data.ichg, \
		    pdev[idx]->chgr.para.data.itaper, \
		    pdev[idx]->chgr.para.data.vbst, \
		    pdev[idx]->chgr.para.data.vfloat );
		}
		else // 如果没有初始化过，执行以下操作
		{
		    // 初始化为默认充电参数
			pdev[idx]->chgr.para.curve = CHGR_PARA_CURVE_U;
			pdev[idx]->chgr.para.data.ichg = default_curve_data[CHGR_PARA_CURVE_U][IDX_ICHG];
			pdev[idx]->chgr.para.data.itaper = default_curve_data[CHGR_PARA_CURVE_U][IDX_ITAPER];
			pdev[idx]->chgr.para.data.vbst = default_curve_data[CHGR_PARA_CURVE_U][IDX_VBST];
			pdev[idx]->chgr.para.data.vfloat = default_curve_data[CHGR_PARA_CURVE_U][IDX_VFLOAT];

			    // 预充电参数
			pdev[idx]->chgr.para.data_pre.pre_iout = default_pre_chgr_para_data[ IDX_PRE_CHGR_PRE_IOUT ];
			pdev[idx]->chgr.para.data_pre.pre_vout = default_pre_chgr_para_data[ IDX_PRE_CHGR_PRE_VOUT ];
			pdev[idx]->chgr.para.data_pre.ichg = default_pre_chgr_para_data[ IDX_PRE_CHGR_ICHG ];
			pdev[idx]->chgr.para.data_pre.vbst = default_pre_chgr_para_data[ IDX_PRE_CHGR_VBST ];
			pdev[idx]->chgr.para.data_pre.itaper = default_pre_chgr_para_data[ IDX_PRE_CHGR_ITAPER ];
			
			    // 自动充电参数
			pdev[idx]->chgr.para.data_auto.threshold = 100;
			
			// 把参数写入eeprom
			set_write_chgr_para_to_ee( &( pdev[idx]->chgr.para) );
			
			// 把默认进入设置的密码写入eeprom
            set_save_passwd( set_passwd_default );

			// 把默认进入维护的密码写入eeprom
            maintain_save_passwd( maintain_passwd_default );

			// 设置已经初始化的标志位
			set_been_init( idx );
			
            LOG_TRACE("\r\nfirst init, cur_num:%d , curve:%d, %f, %f, %f ,%f", pdev[idx]->basic.num, pdev[idx]->chgr.para.curve, pdev[idx]->chgr.para.data.ichg, \
		    pdev[idx]->chgr.para.data.itaper, \
		    pdev[idx]->chgr.para.data.vbst, \
		    pdev[idx]->chgr.para.data.vfloat );

			// 清除事件标志位
			clear_history_event();
		}
		
		// 电池类型
//		pdev[idx]->chgr.bat_type = CHGR_BAT_TYPE_LEAD_ACID;
		pdev[idx]->chgr.bat_type = CHGR_BAT_TYPE_LI;

		// 充电类型，默认三段式
//		pdev[idx]->chgr.type = CHGR_TYPE_THREE_SECTION;
		pdev[idx]->chgr.type = CHGR_TYPE_PRE;
		pdev[idx]->chgr.flag_mode_switch_directly = FALSE;


		pdev[idx]->chgr.flag_stop = FALSE;
		pdev[idx]->chgr.flag_full = FALSE;
        pdev[idx]->chgr.st_timing_time = DISABLE;
        pdev[idx]->chgr.st_booking_time = DISABLE;

		pdev[idx]->chgr.booking_time = CHGR_BOOKING_TIME_DEFAULT;
		pdev[idx]->chgr.timing_time = CHGR_TIMING_TIME_DEFAULT;
        pdev[idx]->chgr.mode = MODE_MANUAL;
    	pdev[idx]->chgr.op = op_stop;
    	pdev[idx]->chgr.vout = 0;
    	pdev[idx]->chgr.iout = 0;
		pdev[idx]->chgr.time = 0;
		pdev[idx]->chgr.AH = 0;
		pdev[idx]->chgr.step = CHGR_STEP_IDLE; // 空闲，表示未充电
		memset( pdev[idx]->chgr.cnt_box, 0, ( sizeof(pdev[idx]->chgr.cnt_box) ) ); // 所有计数器初始化为0
		pdev[idx]->chgr.notify = CHGR_NOTIFY_NONE;

		// 自动充电部分
		pdev[idx]->chgr.auto_limit_iout = 0;
		pdev[idx]->chgr.auto_bat_volt = 0;
		pdev[idx]->chgr.auto_bat_iout = 0;
		pdev[idx]->chgr.auto_bat_cap = 0;  
		pdev[idx]->chgr.auto_bat_default_cap = 30; 
		pdev[idx]->chgr.auto_bat_temp = 0;
		
		pdev[idx]->chgr.auto_vout_step = 0;

        pdev[idx]->chgr.auto_flag_full_bms = FALSE;
        pdev[idx]->chgr.auto_flag_full_mcu = FALSE;
        pdev[idx]->chgr.auto_flag_full_threshold = FALSE;
        pdev[idx]->chgr.auto_flag_bms_start_query = FALSE;
		
		pdev[idx]->event = EVENT_NULL;
		memset( &pdev[idx]->event_cache, 0, sizeof( PU_EventCacheDef ) );

        pdev[idx]->flag_bat_disconn_event_cached = FALSE;

		// 清除从bms获取的电池信息
		memset( &pdev[idx]->bat_info_bms, 0, sizeof( BatInfoDefFromBMS ) );
		pdev[idx]->bat_info_bms.flag_full_volt = FALSE;
		pdev[idx]->bat_info_bms.full_volt = 576; // 默认值设定为57.6V

		pdev[idx]->chgr.iout_queue.head = 0;
//		pdev[idx]->chgr.iout_queue.tail = 0;
		pdev[idx]->chgr.iout_queue.full_flag = FALSE;

		pdev[idx]->chgr.flag_timing = FALSE;
		pdev[idx]->chgr.flag_booking = FALSE;
		pdev[idx]->chgr.flag_bms_timeout = FALSE;

		pdev[idx]->chgr.flag_in_chgr = FALSE;
		
		// 设置模式
		if( pdev[idx]->chgr.type == CHGR_TYPE_THREE_SECTION )
		{
		    chmod_select( idx, MODE_CURVE );
		}
		else if( pdev[idx]->chgr.type == CHGR_TYPE_PRE )
		{
		    chmod_select( idx, MODE_PMBUS );
		}
	}

    // for debug
	pnode = phead_device_list->next;
	while(pnode != NULL)
	{
	    LOG_DEBUG_APP("\r\nin pu init dev_list, num:%d, dev_addr:%02X", pnode->dev.basic.num, pnode->dev.basic.addr);
		pnode = pnode->next; 
	}

	LOG_TRACE("\r\npower unit info init done");
}

/**********************************************************/
// global funtions
/**********************************************************/
BOOL is_been_init( uint8_t num )
{
	uint8_t flag = 0;
	
	// 读取初始化的标志位
	ee_ReadBytes( &flag, EE_ADDR_BEEN_INIT_FLAG, 1 );
	delay_ms( EE_MS_DELAY );
	
	// 先判断eeprom中初始化的标志位
	// 如果num模块对应的位没有初始化过，就直接返回FALSE
	if( ( (flag>>num) & FLAG_BENN_INIT ) != FLAG_BENN_INIT )
	{
		LOG_TRACE("\r\ninit flag:%02x, 模块%d未初始化过", flag,  num);
	    return FALSE;
	}
	
	// 再判断参数是否正确，主要判断充电参数
	memset( &pdev[num]->chgr.para.data, 0, sizeof(pdev[num]->chgr.para.data) );
	set_read_chgr_para_from_ee( &( pdev[num]->chgr.para) );
	if( ( ( pdev[num]->chgr.para.data.ichg - pdev[num]->chgr.data_max.ichg ) > 0 ) || \
        ( ( pdev[num]->chgr.para.data.ichg - pdev[num]->chgr.data_min.ichg ) < 0 )	)
		goto cmd_fail;
	if( ( ( pdev[num]->chgr.para.data.itaper - pdev[num]->chgr.data_max.itaper ) > 0 ) || \
        ( ( pdev[num]->chgr.para.data.itaper - pdev[num]->chgr.data_min.itaper ) < 0 )	)
		goto cmd_fail;
	if( ( ( pdev[num]->chgr.para.data.vbst - pdev[num]->chgr.data_max.vbst ) > 0 ) || \
        ( ( pdev[num]->chgr.para.data.vbst - pdev[num]->chgr.data_min.vbst ) < 0 )	)
		goto cmd_fail;
	if( ( ( pdev[num]->chgr.para.data.vfloat - pdev[num]->chgr.data_max.vfloat ) > 0 ) || \
        ( ( pdev[num]->chgr.para.data.vfloat - pdev[num]->chgr.data_min.vfloat ) < 0 )	)
		goto cmd_fail;


	// 判断预充电参数
	if( ( ( pdev[num]->chgr.para.data_pre.pre_iout - pdev[num]->chgr.pre_data_max.pre_iout ) > 0 ) || \
        ( ( pdev[num]->chgr.para.data_pre.pre_iout - pdev[num]->chgr.pre_data_min.pre_iout ) < 0 )	)
		goto cmd_fail;
	if( ( ( pdev[num]->chgr.para.data_pre.pre_vout - pdev[num]->chgr.pre_data_max.pre_vout ) > 0 ) || \
        ( ( pdev[num]->chgr.para.data_pre.pre_vout - pdev[num]->chgr.pre_data_min.pre_vout ) < 0 )	)
		goto cmd_fail;
	if( ( ( pdev[num]->chgr.para.data_pre.ichg - pdev[num]->chgr.pre_data_max.ichg ) > 0 ) || \
        ( ( pdev[num]->chgr.para.data_pre.ichg - pdev[num]->chgr.pre_data_min.ichg ) < 0 )	)
		goto cmd_fail;
	if( ( ( pdev[num]->chgr.para.data_pre.vbst - pdev[num]->chgr.pre_data_max.vbst ) > 0 ) || \
        ( ( pdev[num]->chgr.para.data_pre.vbst - pdev[num]->chgr.pre_data_min.vbst ) < 0 )	)
		goto cmd_fail;
	if( ( ( pdev[num]->chgr.para.data_pre.itaper - pdev[num]->chgr.pre_data_max.itaper ) > 0 ) || \
        ( ( pdev[num]->chgr.para.data_pre.itaper - pdev[num]->chgr.pre_data_min.itaper ) < 0 )	)
		goto cmd_fail;

	// 判断自动充电参数
	if( ( ( pdev[num]->chgr.para.data_auto.threshold - 0 ) < 0 ) || \
        ( ( pdev[num]->chgr.para.data_auto.threshold - 100 )> 0 )	)
		goto cmd_fail;

	return TRUE;

cmd_fail:
	LOG_TRACE("\r\ninit flag is true, but para err, num:%d", num);
	return FALSE;
}

void set_been_init( uint8_t num )
{
	uint8_t flag = 0;
	
	// 读取初始化的标志位
	ee_ReadBytes( &flag, EE_ADDR_BEEN_INIT_FLAG, 1 );
	delay_ms( EE_MS_DELAY );
    LOG_TRACE("\r\nset been init, init flag:%02X", flag);
    // 设置对应num模块的初始化位为 FLAG_BENN_INIT
    flag = flag | (FLAG_BENN_INIT << num );	
		
	// 写入到eeprom
	ee_WriteBytes( &flag, EE_ADDR_BEEN_INIT_FLAG, 1 );
	delay_ms( EE_MS_DELAY );
}

void hmi_init(void)
{
BaseType_t xReturn = pdPASS;

	// 所有模块显示“手动”
	main_disp_mode( MODE_MANUAL, POWER_UNIT_1 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_2 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_3 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_4 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_5 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_6 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_7 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_8 );
	
	// 所有模块显示“启动”
	main_disp_op( OP_DISP_START, POWER_UNIT_1 );
	main_disp_op( OP_DISP_START, POWER_UNIT_2 );
	main_disp_op( OP_DISP_START, POWER_UNIT_3 );
	main_disp_op( OP_DISP_START, POWER_UNIT_4 );
	main_disp_op( OP_DISP_START, POWER_UNIT_5 );
	main_disp_op( OP_DISP_START, POWER_UNIT_6 );
	main_disp_op( OP_DISP_START, POWER_UNIT_7 );
	main_disp_op( OP_DISP_START, POWER_UNIT_8 );

	// 读取产品编码和生产序号
	ee_ReadBytes( phmi_info->serial_num, EE_SERIAL_NUM_ADDR, EE_SERIAL_NUM_LEN );
	ee_ReadBytes( phmi_info->product_code, EE_PRODUCT_CODE_ADDR, EE_PRODUCT_CODE_LEN );

    // 记录屏幕超时的简单计数器
    phmi_info->timeout = 0;

	// 表示是否处于设置生效中
    phmi_info->flag_setting = FALSE;

	phmi_info->page.id.pre = HMI_SCREEN_SAVE_PAGE_ID;
	phmi_info->page.id.cur = HMI_SCREEN_SAVE_PAGE_ID;
	phmi_info->page.id.cur_rd = HMI_SCREEN_SAVE_PAGE_ID;

	// 存储校验的信息
    phmi_info->verify.page_id = 0;
    phmi_info->verify.flag = FALSE;

	// 重置屏幕
	hmi_reset();

	// create damon task
	xReturn = xTaskCreate( (TaskFunction_t) prvTouchScreenDamonTask,
		                   (const char *) "touch_screen_damon",
		                   (unsigned short) TOUCH_SCREEN_DAMON_STACK,
		                   (void *) NULL,
		                   (UBaseType_t) TOUCH_SCREEN_DAMON_PRIO,
		                   (TaskHandle_t *) &xTouchScreenDamonHandle );
	if( xReturn != pdPASS ) { printf("\r\ncreate touch screen damon failed"); return; }

	// create setting damon task
	xReturn = xTaskCreate( (TaskFunction_t) prvSettingDamonTask,
		                   (const char *) "setting_damon",
		                   (unsigned short) SETTING_DAMON_STACK,
		                   (void *) NULL,
		                   (UBaseType_t) SETTING_DAMON_PRIO,
		                   (TaskHandle_t *) &xSettingDamonHandle );
	if( xReturn != pdPASS ) { printf("\r\ncreate setting damon failed"); return; }

    // get sys time at startup
    hmi_read_sys_time();	

}

void pu_info_init(void)
{
	// 初始化设备列表
    init_device_list();

	// 初始化设备信息
	init_power_unit_info();
}

DevListDef *get_device_list( void )
{
    return phead_device_list;
}

PU_InfoDef *get_device( uint8_t num )
{
	return pdev[num];
}

HMI_InfoDef *get_hmi_info( void )
{
    return phmi_info;
}

void process_hmi_rx_frame(void)
{
	if( !frame_parse() )
	{
		LOG_TRACE("\r\nframe_parse failed, reset rx buff\n");
		com_touch_screen->reset_rx();
		return;
	}
	if( !frame_chk() )
		return;
	
	phmi_info->timeout = 0; // 清零屏保超时

	// 处于屏保等系统页面
	if( phmi_rx_frame->data.addr == HMI_KEY_ADDR_START_SYSTEM )
	{
		LOG_DEBUG_APP("\r\n系统");
		hmi_update_page_id( HMI_SCREEN_SAVE_PAGE_ID );
//		hmi_screen_save();
	}
	// 在主页页面
    else if( (phmi_rx_frame->data.addr >= HMI_KEY_ADDR_START_MAIN) &&\
        (phmi_rx_frame->data.addr <= HMI_KEY_ADDR_END_MAIN) )
	{
		LOG_DEBUG_APP("\r\n主页");
//		hmi_update_page_id( HMI_MAIN_PAGE_ID );
		hmi_main();
	}
	// 在设置页面
	else if( (phmi_rx_frame->data.addr == HMI_KEY_ADDR_SET) || \
             ((phmi_rx_frame->data.addr >= 0x1011) && (phmi_rx_frame->data.addr <= 0x1810)) )
	{
		LOG_DEBUG_APP("\r\n设置");
		hmi_update_page_id( HMI_SET_PAGE_ID );
		hmi_set();
	}
	else if( (phmi_rx_frame->data.addr == HMI_KEY_ADDR_EVENT) || \
			 ( (phmi_rx_frame->data.addr >= HMI_VAR_ADDR_START_EVENT) &&  (phmi_rx_frame->data.addr <= HMI_VAR_ADDR_END_EVENT) ) )
	{
		LOG_DEBUG_APP("\r\n事件");
		hmi_update_page_id( HMI_EVENT_PAGE_ID );
	    hmi_event();
	}
	else if( (phmi_rx_frame->data.addr == HMI_KEY_ADDR_MAINTAIN) || \
			 ( (phmi_rx_frame->data.addr >= HMI_VAR_ADDR_START_MAINTAIN) &&  (phmi_rx_frame->data.addr <= HMI_VAR_ADDR_END_MAINTAIN) ) )
	{
		LOG_DEBUG_APP("\r\n维护");
		hmi_update_page_id( HMI_MAINTAIN_PAGE_ID );
	    hmi_maintain();
	}
	else if( (phmi_rx_frame->data.addr == HMI_KEY_ADDR_HELP) || \
			 ( (phmi_rx_frame->data.addr >= HMI_VAR_ADDR_START_HELP) &&  (phmi_rx_frame->data.addr <= HMI_VAR_ADDR_END_HELP) ) )
	{
		LOG_DEBUG_APP("\r\n帮助");
		hmi_update_page_id( HMI_HELP_PAGE_ID );
        hmi_help();
	}
}

static BOOL frame_parse(void)
{
    uint16_t rx_len = 0;
	uint8_t rx_buffer[50];
	uint8_t data = 0;
	
	uint8_t idx = 0;
	uint8_t len = 0;
	uint8_t data_idx = 0;
	
	if ( phmi_rx_frame == NULL ) { return FALSE; }
	// 初始化存储帧的位置
	memset( phmi_rx_frame, 0, sizeof( *phmi_rx_frame ));

// for debug
//    uart_debug( com_touch_screen, 50 );

	// 处理数据帧
	rx_len = com_touch_screen->get_rx_cnt();
	len = rx_len;
	LOG_DEBUG_APP("\r\ncom_ts rx_len=%d, rx_data: ", rx_len);
	if( rx_len >= 50 )
		return FALSE;

	while( len -- )
	{
		data = com_touch_screen->read();
		LOG_DEBUG_APP_1("%02X ", data);
	    rx_buffer[idx++] = data;
	}

	idx = 0;
    // get header
	phmi_rx_frame->header = (rx_buffer[idx++] << 8 ) & 0xFF00;
	phmi_rx_frame->header |= (rx_buffer[idx++] & 0x00FF);
    // get frame len
	phmi_rx_frame->len = rx_buffer[idx++];
    // get instructions
	phmi_rx_frame->ins = rx_buffer[idx++];

    // get read addr and read-data len
    if( phmi_rx_frame->ins == CMD_RD_VAR )  // 如果是读变量
	{
	    // 读变量地址
	    phmi_rx_frame->data.addr = (rx_buffer[idx++] << 8 ) & 0xFF00;
	    phmi_rx_frame->data.addr |= (rx_buffer[idx++] & 0x00FF);
		LOG_DEBUG_APP_1("\r\ndata.addr:%04X ", phmi_rx_frame->data.addr);

        // 获取完地址，获取实际的数据长度
    	phmi_rx_frame->data.len = rx_buffer[idx++];
		LOG_DEBUG_APP_1("data len:%d ", phmi_rx_frame->data.len);

		// 获取实际的数据值
		LOG_DEBUG_APP_1("data: ");
		len = phmi_rx_frame->data.len;
		while( len -- )
		{
		    phmi_rx_frame->data.data[data_idx] = (rx_buffer[idx++] << 8) & 0xFF00;
		    phmi_rx_frame->data.data[data_idx++] |= (rx_buffer[idx++] & 0x00FF);
		    LOG_DEBUG_APP_1("%04X ", phmi_rx_frame->data.data[data_idx-1]);
		}
	}
	else if( phmi_rx_frame->ins == CMD_RD_REG ) // 如果是读寄存器
	{
		// 读寄存器地址
	    phmi_rx_frame->data_reg.addr = rx_buffer[idx++];
		LOG_DEBUG_APP_1("\r\ndata_reg.addr:%02X ", phmi_rx_frame->data_reg.addr);

	    // 获取完地址，获取实际的数据长度
		phmi_rx_frame->data_reg.len = rx_buffer[idx++];
		LOG_DEBUG_APP_1("data len:%d ", phmi_rx_frame->data_reg.len);

		// 获取实际的数据值
		LOG_DEBUG_APP_1("data: ");
		len = phmi_rx_frame->data_reg.len;
		while( len -- )
		{
			phmi_rx_frame->data_reg.data[data_idx++] = rx_buffer[idx++];
		    LOG_DEBUG_APP_1("%02X ", phmi_rx_frame->data_reg.data[data_idx-1]);
		}
	}

	return TRUE;
}

static BOOL frame_chk(void)
{
	if( phmi_rx_frame->header != FRAME_HEADER )
		return FALSE;
	// 读当前页面id，寄存器地址为0x03，数据长度为2个字节	
	if( phmi_rx_frame->ins == CMD_RD_REG ) 
	{
		if( phmi_rx_frame->data_reg.addr == 0x03 )
		{
		    phmi_info->page.id.cur_rd = ((phmi_rx_frame->data_reg.data[0] << 8) | (phmi_rx_frame->data_reg.data[1]) );

	        LOG_DEBUG_APP("\r\ncur page id:%d", ((phmi_rx_frame->data_reg.data[0] << 8) | (phmi_rx_frame->data_reg.data[1]) ) );

	        // 清空缓冲区
	        memset( phmi_rx_frame, 0, sizeof(hmi_rx_frame) );

		    return FALSE; // return false，表示结束了，不需要进行下一步
		}
		else if( phmi_rx_frame->data_reg.addr == 0x20 ) // 读时间
		{
	        // 读时间
	        phmi_info->sys_time.year = phmi_rx_frame->data_reg.data[0];
	        phmi_info->sys_time.month = phmi_rx_frame->data_reg.data[1];
	        phmi_info->sys_time.day = phmi_rx_frame->data_reg.data[2];
	        phmi_info->sys_time.hour = phmi_rx_frame->data_reg.data[4];
	        phmi_info->sys_time.min = phmi_rx_frame->data_reg.data[5];
	        phmi_info->sys_time.sec = phmi_rx_frame->data_reg.data[6];

#if 0
            LOG_DEBUG_FILE_APP("\r\nread sys time:20%02x-%02x-%02x %02x:%02x:%02x\r\n", phmi_info->sys_time.year, phmi_info->sys_time.month, phmi_info->sys_time.day, \
									                              phmi_info->sys_time.hour, phmi_info->sys_time.min, phmi_info->sys_time.sec );
#endif
			 
	        // 清空缓冲区
	        memset( phmi_rx_frame, 0, sizeof(hmi_rx_frame) );

		    return FALSE; // return false，表示结束了，不需要进行下一步
		}
	}
//	if ( phmi_rx_frame->ins != CMD_RD_VAR )
//		return FALSE;
	
	return TRUE;
}

static void dbg_print_hmi_frame(void)
{
	uint8_t len = phmi_rx_frame->data.len;
	uint8_t idx = 0;
	
	kprintf("\r\n\r\nheader:0x%04X  \r\nlen:%d(D) \r\nins:0x%X", phmi_rx_frame->header, phmi_rx_frame->len, phmi_rx_frame->ins );
	kprintf("\r\ndata.addr:0x%04X \r\ndata.len:%d", phmi_rx_frame->data.addr, phmi_rx_frame->data.len );
	
	kprintf("\r\ndata.data:");
	while ( len -- )
	{
	    kprintf( "%02X %02X", (phmi_rx_frame->data.data[idx] >> 8), (phmi_rx_frame->data.data[idx]&0xff) );
		idx ++;
	}
}

BOOL is_wifi_conn(void)
{
	uint8_t st;

	st = GPIO_READ( EXT_WIFI_LINK_GPIO_PORT, EXT_WIFI_LINK_GPIO_PIN );

    if( st == LOW )
	{
		LOG_DEBUG_APP("\r\nwifi connected!!!");
		return TRUE;
	}
	else
	{
//		LOG_DEBUG_APP("\r\nwifi not connected!!!");
		return FALSE;
	}
}

BOOL is_exec_smp_calibrate( uint8_t num )
{
float bat_volt_bms, bat_volt_smp, calib;
uint8_t flag;
uint16_t calib_save;

	if( pdev[num]->basic.flag_smp_calibrated != TRUE )
	{
	    LOG_DEBUG_APP("\r\n执行校准:");
		bat_volt_smp = pdev[num]->basic.smp_bat_volt_ori;
		bat_volt_bms = pdev[num]->chgr.auto_bat_volt;
	    LOG_DEBUG_APP_1("采样电压：%f, 从bms读取电压:%f", bat_volt_smp, bat_volt_bms);
		calib = bat_volt_bms - bat_volt_smp;
		// 过滤校准值
		if( (calib-10>0) || (calib+10<0) )
		{
			LOG_DEBUG_APP_1("\r\ncalib=%.1f, 非法校准值, stop calibrate", calib);
			return FALSE;
		}

		flag = TRUE;
		pdev[num]->basic.smp_bat_calibrate_value = calib;

		// 保存校准状态
		pdev[num]->basic.flag_smp_calibrated = flag;
	    ee_WriteBytes( &flag, (ee_chgr_para_addr[num]+EE_OFFSET_SMP_CALIBRATED), 1 );
	    delay_ms( EE_MS_DELAY );

		// 保存校准值
		calib_save = calib * 100;
	    ee_WriteBytes( (uint8_t *)&calib_save, (ee_chgr_para_addr[num]+EE_OFFSET_SMP_CALIBRATE_VAL), LEN_SMP_CALIBRATE_VAL );
	    delay_ms( EE_MS_DELAY );
		LOG_DEBUG_APP_1("\r\n校准值为%.2f", calib);

		return TRUE;
	}

	return FALSE;
}

void erase_smp_calibrate( void )
{
	uint8_t idx = 0, flag = FALSE;

	for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
	{
		LOG_DEBUG_APP_1("\r\nerase unit:%d smp calib", idx);

		pdev[idx]->basic.flag_smp_calibrated = FALSE;
		pdev[idx]->basic.smp_bat_calibrate_value = 0;

	    ee_WriteBytes( &flag, (ee_chgr_para_addr[idx]+EE_OFFSET_SMP_CALIBRATED), 1 );
	    delay_ms( EE_MS_DELAY );
	}
}

void suspend_hmi( void )
{
	// 挂起显示相关
	if( eSuspended != eTaskGetState( h_touch_screen_entry ) )
	    vTaskSuspend( h_touch_screen_entry );
	
//	if( eSuspended != eTaskGetState( h_cmd_entry ) )
//	    vTaskSuspend( h_cmd_entry );

#ifdef __DEBUG	
	if( eSuspended != eTaskGetState( h_printf_entry ) )
	    vTaskSuspend( h_printf_entry );
#endif
	
	if( eSuspended != eTaskGetState( xTouchScreenDamonHandle ) )
	    vTaskSuspend( xTouchScreenDamonHandle );
}

void resume_hmi( void )
{
    // 恢复显示相关
	if( eSuspended == eTaskGetState( h_touch_screen_entry ) )
		vTaskResume( h_touch_screen_entry );
	
//	if( eSuspended == eTaskGetState( h_cmd_entry ) )
//		vTaskResume( h_cmd_entry );
#ifdef __DEBUG	
	if( eSuspended == eTaskGetState( h_printf_entry ) )
		vTaskResume( h_printf_entry );
#endif
	
	if( eSuspended == eTaskGetState( xSettingDamonHandle ) )
		vTaskResume( xSettingDamonHandle );
}


