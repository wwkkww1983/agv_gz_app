/************************************************************************
note.
20181226，完成基本的充电启动和停止功能开发调试；
        , 修复stop时，导致hard fault的问题；
20181227，添加参数显示的过滤，即读取的iout，vout等参数，必须在对应型号的参数范围内，才会去更新该参数并显示，否则不更新
        ，修复检测到电池断开后，stop时，界面仍然会显示vout的问题，因为stop后，没有加return
20190109, 完成急停处理，使用按键扫描的方式检测急停，然后通过任务通知进行急停和恢复消息的传递，传递顺序如下：
          prvKeyDamonTask -> charge_entry -> touch_screen_entry
20190114, 同步设置调试通过
          SET_NOTIFY_TO_RESTART_ALL, ->touch_screen_entry->charge_entry
          SET_NOTIFY_RESTART_FINISHED, restart_dev_all -> touch_screen_entry
20190124, 完成历史事件查询功能
          - 事件的缓存
		  - 事件的保存，即同步到eeprom中
		  - 事件的读取 
20190213, 添加电池检测功能，包括反接，电池类型，和对应的事件记录
20190216, 添加电池是否连接功能, 和对应的事件记录
20190225, 添加空闲状态，检测电池连接状态功能
20190228, 修改dc_relay电平控制，低电平关，高电平开


备忘：
1. 记得添加参数设置的上下限制!
2. 模式切换已经屏蔽，后期需要记得打开
*************************************************************************/

#include "applications.h"
#include "bsp_rpb1600_48.h"
#include "gpio.h"
#include "key.h"
#include "eeprom.h"
#include "adc.h"
#include "limits.h"
#include "stdlib.h"
#include "string.h"
//#include "Ext_Interface.h"


#define		IOUT_THRESHOLD 				1.5f
#define		IOUT_THRESHOLD_PRE_CHGR		0.5f
#define		VOUT_ACCURACY_RANGE			0.36f
#define		IOUT_ACCURACY_RANGE			1.34f
#define     READ_IOUT_ACCURACY_RANGE    0.67f


#define     RANG_OVER_OUTPUT            1.5f

#define     THRESHOLD_VOLT_IN_IOUT_DIS    53.5

#define     VOUT_SUCCESS_THRESHOLD      36
#define     OPEN_ON_OFF_DELAY           1000

#define    FILTER_IOUT_DIS_IN_CHARGING    4   // period=100ms
#define    FILTER_PMBUS_COMM      5
#define    FILTER_T_ALARM         3
#define    FILTER_OVER_OUTPUT     3   // 过压连续检测次数为3次
#define    FILTER_DC_OK           10   // 输出异常测次数为10次
#define    FILTER_VOUT_DISPEAR    3   // 自动模式充电中，检测电池突然关闭输出

#define    FILTER_ENTER_CCM       3
#define    FILTER_ENTER_CVM       3
#define    FILTER_ENTER_FVM       3

#define    FILTER_ENTER_CCM_3_SECTION    20
#define    FILTER_ENTER_CVM_3_SECTION    20
#define    FILTER_ENTER_FVM_3_SECTION    20

#define    FILTER_CHK_BAT_REVERSE    2
#define    FILTER_CHK_BAT_CONN       2
#define    FILTER_CHK_BAT_TYPE       2

#define    TIMEOUT_MS_BMS         8000 // 8s
#define    TIMEOUT_CLOSE_AC       60   // 60s
#define    TIMEOUT_CHK_VOUT_IN_STARTING    10000 // 10s
#define    TIMEOUT_CHK_IOUT_IN_STARTING    15000 // 15s

#define    TIMEOUT_MIN_AUTO_CHGR    240 // 4h, 240min


#define    SMP_BAT_VOLT_FILTER_CNT    100 // 采样100次，取平均值
#define    AVE_IOUT_FILTER_CNT        4 // 采样4次，取平均值

#define    FILTER_CLOSE_AC    3
#define    THRESHOLD_TEMP_CLOSE_AC    ( (int8_t)45 )

#define    __START    0x01
#define    __END      0x02
static __IO uint8_t __chgr_damon_step = __START; 

static void write_iout_to_queue ( PU_InfoDef *pdev, float iout );
static void calc_iout_ave( PU_InfoDef *pdev );

static void chk_bat_conn_st( PU_InfoDef *pdev );

static void deal_with_delay_close_ac( PU_InfoDef *pdev );
static void deal_with_smp_bat_volt( PU_InfoDef *pdev );
static void deal_with_bat_disconn_in_stop_by_host( PU_InfoDef *pdev );

static void start( PU_InfoDef *pdev );
static void stop( PU_InfoDef *pdev ); static void charging( PU_InfoDef *pdev );
static void chgr_auto( PU_InfoDef *pdev );
static void chgr_manual_with_3_section( PU_InfoDef *pdev );
static void chgr_manual_with_pre( PU_InfoDef *pdev );
static void start_success( PU_InfoDef *pdev );
static void start_failed( PU_InfoDef *pdev );

static void restart_dev( PU_InfoDef *pdev );
static void restart_dev_all( void );
static void open_output( uint8_t num );
static void close_output( uint8_t num );
void close_ac_input( uint8_t num );
void close_ac_input_all( void );
void open_ac_input_all( void );
static BOOL is_iout_disappear_in_charge( PU_InfoDef *pdev, float iout );
static BOOL is_t_alarm_in_charge( PU_InfoDef *pdev );
static BOOL is_vout_dispear_in_auto_mode( PU_InfoDef *pdev );
	
static BOOL isBatReverse( float volt );
//static BOOL isBatConn( float volt );

static BOOL isDcOutPutErr( PU_InfoDef *pdev );
static uint8_t read_dc_ok( uint8_t num );

static BOOL chk_pmbus_comm( uint32_t *pcnt, uint8_t addr );

static void chk_bat_conn_in_charging( PU_InfoDef *pdev );
static void chgr_emergency_stop(void);
static void chgr_start_from_emergency_stop(void);

static void chgr_mode_switch( PU_InfoDef *pdev );

static void chgr_control_little_iout( PU_InfoDef *pdev, float vout, float iout, float limit_iout );
static void chgr_control_little_iout_in_starting( PU_InfoDef *pdev, float vout, float iout, float limit_iout );
static BOOL chk_dc_output( PU_InfoDef *pdev );
static BOOL chk_temperature( PU_InfoDef *pdev );	

#define    CHGR_DAMON_STACK    384
#define    CHGR_DAMON_PRIO     5
//#define    CHGR_DAMON_PERIOD   500    // 500ms
#define    CHGR_DAMON_PERIOD   100    // 100ms
static TaskHandle_t xChargeDamonHandle = NULL;
static void prvChargeDamonTask( void *pvParameter );
static void suspend_charge_damon(void);
static void resume_charge_damon(void);

#if 0
#define    CHK_BAT_DAMON_STACK    256
#define    CHK_BAT_DAMON_PRIO     3
#define    CHK_BAT_DAMON_PERIOD   500    // 500ms
static TaskHandle_t xChkBatDamonHandle = NULL;
static void prvChkBatDamonTask( void *pvParameter );
#endif

#if 0
#define    KEY_DAMON_STACK    220 // 针对单独一个按键，不能低于200，否则会stack error
#define    KEY_DAMON_PRIO     5
TaskHandle_t xKeyDamonHandle = NULL;
static void prvKeyDamonTask( void *pvParameter );
#endif

#define    EVENT_DAEMON_WRITE_STACK    256
#define    EVENT_DAEMON_WRITE_PRIO     3
TaskHandle_t xEventDaemonWriteHandle = NULL;
static void prvEventDaemonWriteTask( void *pvParameter );

#define    CHGR_TIME_CTRL_DAEMON_STACK    256
#define    CHGR_TIME_CTRL_DAEMON_PRIO     3
TaskHandle_t xChgrtimeCtrlHandle = NULL;
static void prvChgrTimeCtrlDaemonTask( void *pvParameter );

#define    CHGR_SMP_VOLT_DAEMON_STACK    256
#define    CHGR_SMP_VOLT_DAEMON_PRIO     3
TaskHandle_t xChgrSmpVoltHandle = NULL;
static void prvChgrSmpVoltDaemonTask( void *pvParameter );

static void save_event( PU_InfoDef *pdev, uint8_t code );
BOOL is_ee_event_full( uint8_t num );
static void cache_event( PU_InfoDef *pdev, uint8_t code );

SemaphoreHandle_t xSemEventRW = NULL;

static DevListDef *pdev_list = NULL;
static ChgrListDef *pchgr_list = NULL;
static ChgrListDef *init_chgr_list(void);
static void add_to_chgr_list( PU_InfoDef * const  node );
static void remove_from_chgr_list( const uint8_t num );

#define    chgrMS_TO_CNT( xTimeInMs ) ( ((uint32_t)xTimeInMs) / CHGR_DAMON_PERIOD )
#define    chgrCNT_TO_MS( xCnt ) ( ((uint32_t)xCnt * CHGR_DAMON_PERIOD) )


#if 0
void charge_entry( void * pvParameters )
{
TickType_t portDelayTime = portMAX_DELAY;  // 阻塞任务
BaseType_t xResult = pdTRUE;
uint32_t ulNotifiedValue;
DevListDef *pnode;

	for( ;; )
	{
		/* Wait to be notified. */
        xResult = xTaskNotifyWait( (uint32_t)   pdFALSE, /* Don't clear bits on entry. */
                                   (uint32_t)   ULONG_MAX, /* Clear all bits on exit. */
                                   (uint32_t *) &ulNotifiedValue, /* Stores the notified value. */
                                   (TickType_t) portDelayTime ); /* block */
        if( xResult == pdPASS )
        {
            /* A notification was received. See which bits were set. */
			if( ulNotifiedValue & CHGR_NOTIFY_TO_START )
			{
				pnode = pdev_list->next;
			    while( pnode != NULL)
				{
				   if( pnode->dev.chgr.notify == CHGR_NOTIFY_TO_START )
				   {
					   LOG_DEBUG_APP_1("charg_entry rec start, num:%d .... ", pnode->dev.basic.num);
					   start( &pnode->dev );
				   }
				   pnode = pnode->next;
				}
			}
            else if( ulNotifiedValue & CHGR_NOTIFY_TO_STOP )
			{
				pnode = pdev_list->next;
			    while( pnode != NULL)
				{
				   if( pnode->dev.chgr.notify == CHGR_NOTIFY_TO_STOP )
				   {
					   LOG_TRACE("\r\nstop num:%d", pnode->dev.basic.num);
					   stop( &pnode->dev );
				   }
				   
				   pnode = pnode->next;
				}
			}
			else if( ulNotifiedValue & SET_NOTIFY_TO_RESTART )
			{
				LOG_TRACE("\r\ncharge_entry rec restart... ...");
				pnode = pdev_list->next;
			    while( pnode != NULL)
				{
				   if( pnode->dev.basic.restart == BASIC_RESTART_YES )
				   {
					   LOG_TRACE_1("find restart num:%d", pnode->dev.basic.num);
					   // 执行重启操作
					   restart_dev( &pnode->dev );
				   }

				   pnode = pnode->next;
				}
			}
			else if( ulNotifiedValue & SET_NOTIFY_TO_RESTART_ALL )
			{
				LOG_TRACE("\r\ncharge_entry rec restart all... ...");
				// 重启所有模块
//			    restart_dev_all();
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_EMERGENCY_STOP )
			{
				    // 处理急停 
				    LOG_DEBUG_APP("\r\nchgr, stop");
//			        chgr_emergency_stop();
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_START_FROM_EMERGENCY_STOP )
			{
				    // 处理从急停状态恢复
				    LOG_DEBUG_APP("\r\nchgr, resume");
//			        chgr_start_from_emergency_stop();
			}
		}
		else
		{
		    printf("\r\nerror");
		}
	}   
}
#endif

static void prvChgrTimeCtrlDaemonTask( void *pvParameter )
{
DevListDef *pnode;
uint8_t *local_bat_info = NULL;
uint8_t *p_single_bat_volt_info_fifo;
uint8_t *p_bms_info_fifo;
uint8_t *p_bat_id_num_fifo;

	for( ;; )
	{
        pnode = pdev_list->next;
        while( pnode != NULL)
	    {
		    // 每隔1s告诉wdg_daemon我还活着
	        xTaskNotify( h_wdg_daemon, WDG_BIT_CHGR_TIME_CTRL, eSetBits );
			
			if( pnode->dev.chgr.flag_booking == TRUE )
			{
			    pnode->dev.chgr.flag_booking = FALSE;
				LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d booking finish, start chgr", pnode->dev.chgr.cnt_box[ IDX_CNT_BOOKING_TIME ], pnode->dev.basic.num );
				// 执行启动操作
				pub_start( START_TYPE_BOOKING, pnode->dev.basic.num );
			}
			if( pnode->dev.chgr.flag_timing == TRUE )
			{
			    pnode->dev.chgr.flag_timing = FALSE;
				LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d  timing finish，stop chgr", pnode->dev.chgr.cnt_box[ IDX_CNT_TIMING_TIME ], pnode->dev.basic.num );
				// 执行停止操作
			    pub_stop( STOP_TYPE_BOOKING, pnode->dev.basic.num );
			}
			if( pnode->dev.chgr.flag_bms_timeout == TRUE )
			{
			    pnode->dev.chgr.flag_bms_timeout = FALSE;

                LOG_DEBUG_APP("\r\nbms timeout ");
			    if( pnode->dev.basic.chgr != BASIC_CHGR_OFF )
			    {
			    	LOG_DEBUG_APP_1("in chgring, stop ");
			        pub_stop( STOP_TYPE_BMS, pnode->dev.basic.num );
			    }
			    // 更新BMS没有连接的信息
			    local_bat_info = get_local_bat_info();
		        reset_bit(local_bat_info[ pnode->dev.basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BMS_CONN);
				// 清空对应的电池信息
				memset( &local_bat_info[ pnode->dev.basic.num * SIZE_SINGLE_BAT_INFO + 1 ], 0, ( SIZE_SINGLE_BAT_INFO - 1 ) );
				pnode->dev.bat_info_bms.volt_single_14 = 57.6; // 默认57.6
				pnode->dev.bat_info_bms.volt_single_16 = 57.6; // 默认57.6

				// 清空从bms获取的电池信息
		        memset( &pnode->dev.bat_info_bms, 0, sizeof( BatInfoDefFromBMS ) );
		        pnode->dev.bat_info_bms.flag_full_volt = FALSE;
		        pnode->dev.bat_info_bms.full_volt = 576; // 默认值设定为57.6V

				// 如果bms断开，那么清除非法停止状态
                pnode->dev.basic.warn = BASIC_WARN_NO;

				// 如果bms断开，设置单体电压的信息长度为0，清空对应的缓存
				pnode->dev.bat_info_bms.single_bat_volt_len = 0;
				if( NULL != ( p_single_bat_volt_info_fifo = getSingleBatVoltInfoFifo() ) )
				    memset( (p_single_bat_volt_info_fifo + pnode->dev.basic.num * SIZE_SINGLE_BAT_VOLT_INFO), 0, SIZE_SINGLE_BAT_VOLT_INFO );

				// 如果bms断开，清空bms版本信息
				memset(pnode->dev.bat_info_bms.bms_ver, 0, SIZE_BMS_VER);
				if( NULL != ( p_bms_info_fifo = getBatBmsVerFifo() ) )
				    memset( (p_bms_info_fifo+pnode->dev.basic.num*SIZE_BMS_VER), 0, SIZE_BMS_VER );

				// 如果bms断开，清空ID串号信息
				memset(pnode->dev.bat_info_bms.id_num, 0, SIZE_BAT_ID_NUM);
				if( NULL != ( p_bat_id_num_fifo = getBatIdNumFifo() ) )
				    memset( (p_bat_id_num_fifo+pnode->dev.basic.num*SIZE_BAT_ID_NUM), 0, SIZE_BAT_ID_NUM );

				// 如果bms断开，清除查询标志位
				pnode->dev.chgr.auto_flag_bms_start_query = FALSE;
				pnode->dev.chgr.cnt_box[ IDX_CNT_DELAY_BMS_START_QUERY ] = 0;

				// bms断开时，如果电池MOS关闭的告警信息仍然存在，清除告警信息，20191030添加
	            if( !( pnode->dev.bat_info_bms.warn_4 & bit( 6 ) ) )
					reset_bit( pnode->dev.basic.err, ERR_BAT_CHGR_MOS_CLOSE );

				// 清除是否需要cache电池断开事件的标志位
				pnode->dev.flag_bat_disconn_event_cached = FALSE;

			    LOG_DEBUG_APP_1("switch to manual mode");
			    pnode->dev.basic.flag_mode_switch = BASIC_MODE_AUTO_TO_MANUAL; 
			}

			/*
			 * 处理延时关闭AC输入
			 */
            deal_with_delay_close_ac( &pnode->dev );
			
			// 处理模式切换
			if( pnode->dev.basic.flag_mode_switch != BASIC_MODE_NONE_SWITCH )
			    chgr_mode_switch( &pnode->dev );

			// 更新充电站状态信息，上传给上位机
           update_local_chgr_station_info( &pnode->dev );

		   // 进入被动停止状态后，检测电池是否断开
//		   deal_with_bat_disconn_in_stop_by_host( &pnode->dev );
			
	       pnode = pnode->next;
	    }
	
		vTaskDelay( pdMS_TO_TICKS(1000) );
	}
}

static void clear_chgr_info( PU_InfoDef *pdev )
{
	// 处理数据刷新
    if( ( pdev->basic.chgr == BASIC_CHGR_OFF ) || ( pdev->basic.chgr == BASIC_CHGR_STOP_BY_HOST ) )
	{
		pdev->basic.temp = 25;
		
        pdev->chgr.status = (uint16_t)0;

		// 清空对应模块的数据
		pdev->chgr.vout = 0;
		pdev->chgr.iout = 0;
		pdev->chgr.time = 0;
		
	    // 自动充电部分
		if( pdev->basic.chgr != BASIC_CHGR_STOP_BY_HOST )
		{
		    pdev->chgr.auto_bat_cap = 0;  
		    pdev->chgr.auto_bat_temp = 0;
		}
		pdev->chgr.auto_limit_iout = 0;
		pdev->chgr.auto_bat_iout = 0;
		pdev->chgr.auto_vout_step = 0;
        pdev->chgr.auto_flag_full_bms = FALSE;
        pdev->chgr.auto_flag_full_mcu = FALSE;
        pdev->chgr.auto_flag_full_threshold = FALSE;

		//
        pdev->chgr.iout_ori = 0;
        pdev->chgr.iout_disp = 0;
        memset( &pdev->chgr.iout_queue, 0, sizeof( Queue_Float ) );
	}
	else if( pdev->basic.chgr == BASIC_CHGR_FINISH )
	{
	    pdev->chgr.vout = 0;
		pdev->chgr.iout = 0;

        pdev->chgr.iout_ori = 0;
        pdev->chgr.iout_disp = 0;
        memset( &pdev->chgr.iout_queue, 0, sizeof( Queue_Float ) );
	}
}

static void prvChgrSmpVoltDaemonTask( void *pvParameter )
{
DevListDef *pnode;

	for( ;; )
	{
	    xTaskNotify( h_wdg_daemon, WDG_BIT_SMP_VOLT, eSetBits );

        pnode = pdev_list->next;
        while( pnode != NULL)
		{
           deal_with_smp_bat_volt( &pnode->dev );

		   chk_bat_conn_st( &pnode->dev );

		   deal_with_bat_disconn_in_stop_by_host( &pnode->dev );

	       pnode = pnode->next;

		}
		vTaskDelay( pdMS_TO_TICKS(20) );
	}
}

static void deal_with_smp_bat_volt( PU_InfoDef *pdev )
{
/*
 * 100us采一次，连续采100次，取个平均值，作为实际的电池电压值
 */
	uint16_t idx = 0;
	__IO uint32_t sum = 0;
	uint16_t ave = 0;
	float tmp_volt = 0;
	float calib_volt = 0;

	for( idx = 0; idx < SMP_BAT_VOLT_FILTER_CNT; idx ++ )
	{
		sum += adc_get_convert_value_single( pdev->basic.num );
		delay_us(100);
	}
	ave = sum / SMP_BAT_VOLT_FILTER_CNT;

    tmp_volt = ( (float)ave / 4096 ) * 199 - 100;
    pdev->basic.smp_bat_volt_ori = tmp_volt;

	// 加上校准值
	calib_volt = tmp_volt + pdev->basic.smp_bat_calibrate_value;
	if( (calib_volt - 10 <= 0) && ( calib_volt + 10 >= 0 ) )
		calib_volt = 0;
	pdev->basic.bat_volt = calib_volt;
}

static void deal_with_bat_disconn_in_stop_by_host( PU_InfoDef *pdev )
{
    if( pdev->basic.chgr == BASIC_CHGR_STOP_BY_HOST )
	{
	    if( !isBatConn( pdev->basic.bat_volt ) )
		{
			LOG_DEBUG_APP("\r\nin stop by host, bat disconn, volt:%f", pdev->basic.bat_volt);
		    pdev->basic.chgr = BASIC_CHGR_OFF;
		}
	}
}

static void chgr_mode_switch( PU_InfoDef *pdev )
{
	if( pdev->basic.flag_mode_switch != BASIC_MODE_NONE_SWITCH )
	{
		// 1. 先关闭模块	
		// 如果模块是开机的状态，那么就关掉，然后延时8s，切换模式
		if( pdev->basic.flag_ac_power == BASIC_POWER_ON )
		{
			LOG_DEBUG_APP("\r\n模式切换时，模块是开机状态，直接关掉, num(0-base):%d", pdev->basic.num);
			power_off( pdev->basic.num );
			pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] = 0;
		}
		// 2. 延时8s，等待模块关闭完成
		pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] ++;

		// 3. 延时时间到，切换模式
		if( pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] > 8 )
		{
			if( pdev->basic.flag_mode_switch == BASIC_MODE_AUTO_TO_MANUAL )
			{
			    pdev->basic.flag_mode_switch = BASIC_MODE_NONE_SWITCH;

			    LOG_DEBUG_APP("\r\n模式切换延时时间到，切换到手动模式, num(0-base):%d", pdev->basic.num);
		        pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] = 0;
			    
				if( pdev->chgr.type == CHGR_TYPE_PRE )
				{
					LOG_DEBUG_APP_1("\r\n充电方式为预充电方式");
			        chmod_select( pdev->basic.num, MODE_PMBUS );
				}
				else if ( pdev->chgr.type == CHGR_TYPE_THREE_SECTION )
				{
					LOG_DEBUG_APP_1("\r\n充电方式为三段式充电曲线方式");
			        chmod_select( pdev->basic.num, MODE_CURVE );
				}
						
	            pdev->chgr.mode = MODE_MANUAL;  
			}
			else if( pdev->basic.flag_mode_switch == BASIC_MODE_MANUAL_TO_AUTO )
			{
			    pdev->basic.flag_mode_switch = BASIC_MODE_NONE_SWITCH;

			    LOG_DEBUG_APP("\r\n模式切换延时时间到，切换到自动模式, num(0-base):%d", pdev->basic.num);
		        pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] = 0;
			    
			    chmod_select( pdev->basic.num, MODE_PMBUS );
	            pdev->chgr.mode = MODE_AUTO;  
			}
			else if( pdev->basic.flag_mode_switch == BASIC_MODE_PRE_TO_THREE_SECTION )
			{
			    pdev->basic.flag_mode_switch = BASIC_MODE_NONE_SWITCH;

			    LOG_DEBUG_APP("\r\n充电方式切换延时时间到，切换到三段式曲线方式, num(0-base):%d", pdev->basic.num);
		        pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] = 0;
			    
			    chmod_select( pdev->basic.num, MODE_CURVE );

				LOG_DEBUG_APP_1("\r\n设置电池类型为铅酸电池");
				pdev->chgr.bat_type = CHGR_BAT_TYPE_LEAD_ACID;
				pdev->chgr.type = CHGR_TYPE_THREE_SECTION;

			    // 发送给touch_screen_entry通知，显示切换完成
	            xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_TYPE_SWITCH_FINISH, eSetBits );
			}
			else if( pdev->basic.flag_mode_switch == BASIC_MODE_THREE_SECTION_TO_PRE )
			{
			    pdev->basic.flag_mode_switch = BASIC_MODE_NONE_SWITCH;

			    LOG_DEBUG_APP("\r\n充电方式切换延时时间到，切换到预充电方式, num(0-base):%d", pdev->basic.num);
		        pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] = 0;
			    
			    chmod_select( pdev->basic.num, MODE_PMBUS );

				LOG_DEBUG_APP_1("\r\n设置电池类型为锂电池");
				pdev->chgr.bat_type = CHGR_BAT_TYPE_LI;
				pdev->chgr.type = CHGR_TYPE_PRE;

			    // 发送给touch_screen_entry通知，显示切换完成
	            xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_TYPE_SWITCH_FINISH, eSetBits );
			}

			// 如果需要延时关闭，那么就重新打开
			if( pdev->basic.flag_delay_close_ac != OFF )
			{
			    LOG_DEBUG_APP("\r\n模式切换完成，但是需要延时关闭，重新打开模块, num(0-base):%d", pdev->basic.num); 
				power_on( pdev->basic.num );
				pdev->chgr.cnt_box[IDX_CNT_DELAY_CLOSE_AC] = 0;
			}
		}
	}
}

static void prvChargeDamonTask( void *pvParameter ) 
{
ChgrListDef *pnode = NULL;
	
    for( ;; )	
	{
		if( pchgr_list == NULL ) { continue; };

		// 每隔100ms告诉wdg_daemon我还活着
	    xTaskNotify( h_wdg_daemon, WDG_BIT_CHGR_DAEMON, eSetBits );
		
		pnode = pchgr_list->next;
		while( pnode != NULL )
		{
			__chgr_damon_step = __START;
			charging( pnode->pdev );
			__chgr_damon_step = __END; 
			
		    pnode = pnode->next;

		}

		vTaskDelay( pdMS_TO_TICKS( CHGR_DAMON_PERIOD ) );
	}
}

static void suspend_charge_damon(void)
{
    if( eSuspended != eTaskGetState( xChargeDamonHandle ) )
		vTaskSuspend( xChargeDamonHandle );
}

static void resume_charge_damon(void)
{
    if( eSuspended == eTaskGetState( xChargeDamonHandle ) )
		vTaskResume( xChargeDamonHandle );
}

#if 0
static void prvChkBatDamonTask( void *pvParameter )
{
DevListDef *pnode = NULL;

    for( ;; )
	{
		if( pdev_list == NULL ) { LOG_DEBUG_APP("\r\nin chk bat damon, pdev_list is NULL"); return; };
		
		pnode = pdev_list->next;
		while( pnode != NULL )
		{
//			chk_bat_conn_st( &pnode->dev );
			
		    pnode = pnode->next;
		}

		vTaskDelay( pdMS_TO_TICKS( CHK_BAT_DAMON_PERIOD ) );
	}
}
#endif

#if 0
static void prvKeyDamonTask( void *pvParameter )
{
	uint8_t _step_chk_press = 0x00;
	uint8_t _step_chk_release = 0x01;
	uint8_t _step = _step_chk_press; //0x00-check key press, 0x01-check key release

    KeyTypeDef key_emergency_stop = { STOP_GPIO_PORT, STOP_GPIO_PIN, 0, 0xff, 0 };

    for( ;; )
	{
    	if( _step == _step_chk_press )
    	{
		    // 检测按键是否按下
    		if( key_detect( &key_emergency_stop ) == KEY_ON )
    		{
				// 通知充电入口函数，执行了紧急停机
                xTaskNotify( h_charge_entry, CHGR_NOTIFY_EMERGENCY_STOP, eSetBits );
    		    _step = _step_chk_release;
    		}
    	}
    	else if( _step == _step_chk_release )
    	{
			// 检测按键是否释放
    		if( key_detect( &key_emergency_stop ) == KEY_OFF )
    		{
                xTaskNotify( h_charge_entry, CHGR_NOTIFY_START_FROM_EMERGENCY_STOP, eSetBits );
    		    _step = _step_chk_press;
    		}
    	}

		// 每10ms检测一次
        vTaskDelay( pdMS_TO_TICKS( KEY_SCAN_INTERVAL ) );
	}
}
#endif
static void prvEventDaemonWriteTask( void *pvParameter )
{
uint8_t idx;
DevListDef *pnode;
BaseType_t xReturn = pdPASS;

    for( ;; )
	{
		// 遍历所有模块
		pnode = pdev_list->next;

		while( pnode != NULL)
		{
		    // 每隔500ms告诉wdg_daemon我还活着
	        xTaskNotify( h_wdg_daemon, WDG_BIT_EVENT_WRITE, eSetBits );

			// 查询该模块的事件缓存
			for( idx = 0; idx < EVENT_MAX_NUM; idx ++ )
			{
			    if( pnode->dev.event_cache.code & bit( idx ) )
				{
					LOG_DEBUG_APP("\r\n-------------start save event, num:%d(0-base), code:%d .... ", pnode->dev.basic.num, idx );

					xReturn = xSemaphoreTake( xSemEventRW, pdMS_TO_TICKS(500) );
	                if( pdTRUE != xReturn )
					{
	                    LOG_DEBUG_APP_1("\r\nwrite event, get mutex failed");
						break;
					}
					else
	                    LOG_DEBUG_APP_1("\r\nwrite event, get mutex succss");

					// 保存对应的事件
				    save_event( &pnode->dev, idx );

					// 清空对应的事件
					reset_bit( pnode->dev.event_cache.code, idx );

	                // 给出互斥量
					xReturn = xSemaphoreGive( xSemEventRW );
	                if( pdTRUE != xReturn )
					{
	                    LOG_DEBUG_APP_1("\r\nwrite event, give mutex failed");
						break;
					}
					else
	                    LOG_DEBUG_APP_1("\r\nwrite event, give mutex succss");

					LOG_DEBUG_APP_1("\r\n... save event finish--------------------end");
				}
			}

			pnode = pnode->next;
		}

		vTaskDelay( pdMS_TO_TICKS(500) );
	}
}

static void save_event( PU_InfoDef *pdev, uint8_t event_code )
{
	uint8_t num = 0;          // 模块序号
	uint16_t latest_idx = 0;  // 最新存储事件的序号
	uint32_t ee_wr_addr = 0;  // 写eeprom的地址
    EE_SysEventDef event;     // 待存储的事件
	uint16_t flag_full = 0;   // 用来记录事件是否已经存满

    LOG_DEBUG_APP_1("\r\nsave_event, num:%d, event_code:%d,time:20%02x-%02x-%02x %02x:%02x:%02x", pdev->basic.num, event_code, \
					  pdev->event_cache.time[ event_code ].year, pdev->event_cache.time[ event_code ].month, pdev->event_cache.time[ event_code ].day, \
					  pdev->event_cache.time[ event_code ].hour, pdev->event_cache.time[ event_code ].min, pdev->event_cache.time[ event_code ].sec );

	// 获取模块序号
	num = pdev->basic.num;

	// 读取最新的序号latest_idx, 为最新写入的地址
    ee_ReadBytes( (uint8_t *)&latest_idx, EE_EVENT_LATEST_IDX_BASE_ADDR + ( num * EE_EVENT_LATEST_IDX_LEN ), EE_EVENT_LATEST_IDX_LEN );
	vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	LOG_DEBUG_APP_1("\r\nsave event, before save, latest_idx:%d", latest_idx );

	// 写入事件
	/* 计算eeprom的写入地址:
	 * 1. 先获取该模块 存储事件 的基地址，
	 * 2. 获取写入的地址，即: 基地址 + latest_idx * sizeof(EE_SysEventDef)
	 * 采用从大到小写，（便于的读的时候从小到大读，可以支持eeprom的page_read）
	 */
	ee_wr_addr = EE_EVENT_STORE_BASE_ADDR + num * EVENT_MAX_STORE_NUM * sizeof( EE_SysEventDef ) + \
				 (latest_idx) * sizeof( EE_SysEventDef );

	// 构建事件存储变量
	event.sys_time = pdev->event_cache.time[ event_code ]; // 获取对应事件发生的时间
	event.code = (uint16_t)( ( event_code & 0xFF ) << 8); // 高八位存储事件代码
	// 如果是启动事件，需要记录模式和参数
	if( event_code == EVENT_START )
	{
		event.code |= ( ( ( ( (uint8_t)event.code ) & 0x0F ) >> 4 ) | ( pdev->chgr.mode & 0x0F) ) << 4;    // 低八位的高四位是模式
        event.code |= ( ( ( (uint8_t)event.code ) & 0xF0 ) | ( pdev->chgr.para.curve & 0x0F ) );    // 低八位的低四位是参数
	}

	ee_WriteBytes( (uint8_t *)&event, ee_wr_addr, sizeof( EE_SysEventDef ) );
	vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	// 更新最新事件序号
	if( latest_idx != 0 )
	{
	    latest_idx --;
	}
	else if( latest_idx == 0 )  // 如果最新的写入序号已经是0了，那么循环，从最大序号重新覆盖写
	{	
	    latest_idx = EVENT_MAX_STORE_NUM - 1;	

		// 此时，如果之前事件没有满，那么设置事件已经满了
		if( !is_ee_event_full( num ) )
		{
	    	// 记录已经事件已经存满
            ee_ReadBytes((uint8_t *)&flag_full, EE_EVENT_IS_FULL_ADDR, EE_EVENT_IS_FULL_LEN );
	        vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	    	set_bit( flag_full, num );
	    	
            ee_WriteBytes((uint8_t *)&flag_full, EE_EVENT_IS_FULL_ADDR, EE_EVENT_IS_FULL_LEN );
	        vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
		}
	}
	// 把最新事件序号保存到eeprom
    ee_WriteBytes( (uint8_t *)&latest_idx, EE_EVENT_LATEST_IDX_BASE_ADDR + ( num * EE_EVENT_LATEST_IDX_LEN ), EE_EVENT_LATEST_IDX_LEN );
	vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	LOG_DEBUG_APP_1("\r\nsave event, save finish, latest_idx:%d", latest_idx );
}

BOOL read_event( PU_InfoDef *pdev, EE_SysEventDef *pevent, uint8_t page_num, uint8_t num_to_read, uint8_t *p_num_actual_read )
{
    uint8_t num = 0;
	uint16_t part_1_num = 0;
	uint16_t part_2_num = 0;
	uint16_t latest_idx = 0;
	uint16_t latest_read_idx = 0;
	uint16_t read_idx = 0; 
	uint16_t num_actual_read = 0;
	uint32_t ee_base_addr = 0;    //  存储当前模块，历史事件的基地址
	uint32_t ee_read_addr = 0;    //  读历史事件的地址

	// 模块序号
	num = pdev->basic.num;

	LOG_DEBUG_APP("\r\n-------------start read_event, num:%d ....", num);

	// 获取互斥量
	if( pdTRUE != xSemaphoreTake( xSemEventRW, pdMS_TO_TICKS(500) ) )
	{
	    LOG_DEBUG_APP_1("\r\nread event, get mutex failed");
	    LOG_DEBUG_APP_1("\r\n... read event finish--------------------end");
		return FALSE;
	}
	else
	    LOG_DEBUG_APP_1("\r\nread event, get mutex success");

	// 执行读的操作	
	    // 读取最新的写入序号
    ee_ReadBytes( (uint8_t *)&latest_idx, EE_EVENT_LATEST_IDX_BASE_ADDR + ( num * EE_EVENT_LATEST_IDX_LEN ), EE_EVENT_LATEST_IDX_LEN );
	vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
	LOG_DEBUG_APP_1("\r\nread_event, latest_idx:%d", latest_idx);

    ee_base_addr = EE_EVENT_STORE_BASE_ADDR + num * EVENT_MAX_STORE_NUM * sizeof( EE_SysEventDef ); 
	
	// 先确定最新的读序号
	latest_read_idx = latest_idx + 1;
	if( latest_read_idx > ( EVENT_MAX_STORE_NUM - 1 ) )
		latest_read_idx = 0;	

    // 根据页面序号，确定实际读的序号
	read_idx = latest_read_idx + num_to_read * page_num;
	LOG_DEBUG_APP_1("\r\nread_idx:%d", read_idx );

	// 处理历史记录数量满了的情况，如果满了, 循环读取
	if( is_ee_event_full( num ) )
	{
		// 如果读取序号超出了最大序号，那么循环从最小序号开始计算读取的序号
		if( read_idx > ( EVENT_MAX_STORE_NUM - 1 ) )
			read_idx = read_idx - EVENT_MAX_STORE_NUM;

		// 根据实际读的序号，确定eeprom实际读的地址
	    ee_read_addr = ee_base_addr + read_idx * sizeof( EE_SysEventDef );

		// 如果四条记录不能一次读完
		if( (read_idx + num_to_read - 1) > (EVENT_MAX_STORE_NUM - 1) )
		{
            // 先读第一部分
            part_1_num = EVENT_MAX_STORE_NUM - read_idx;
			num_actual_read = part_1_num;

		    ee_ReadBytes((uint8_t*)pevent, ee_read_addr, ( num_actual_read * sizeof( EE_SysEventDef ) ) );
	        vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

            // 再读第二部分
			part_2_num = num_to_read - part_1_num;
			num_actual_read = part_2_num;
            ee_read_addr = ee_base_addr;  // 从基地址开始读
		    ee_ReadBytes((uint8_t*)&pevent[ part_1_num ], ee_read_addr, ( num_actual_read * sizeof( EE_SysEventDef ) ) );
	        vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
		}
		else
		{
			num_actual_read = num_to_read;

		    ee_ReadBytes((uint8_t*)pevent, ee_read_addr, ( num_actual_read * sizeof( EE_SysEventDef ) ) );
	        vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
		}

		*p_num_actual_read = num_to_read;
	}
	// 处理历史记录数量没有满的情况
	else  
	{
		// 如果读取序号超出了最大序号或者最新序号为最大序号，说明该页数据为空，返回FALSE
		if( ( read_idx > ( EVENT_MAX_STORE_NUM - 1 ) ) || ( latest_idx == ( EVENT_MAX_STORE_NUM - 1 )  ) )
		{
		    *p_num_actual_read = 0;

	        LOG_DEBUG_APP_1("\r\nin event is not full, read_idx:%d, page:%d event is empty, return false", read_idx, page_num );
	        // 给出互斥量
	        if( pdTRUE != xSemaphoreGive( xSemEventRW ) )
	        {
	            LOG_DEBUG_APP_1("\r\nread event, give mutex failed");	
	        }

			return FALSE;
		}

		// 根据实际读的序号，确定eeprom实际读的地址
	    ee_read_addr = ee_base_addr + read_idx * sizeof( EE_SysEventDef );

		// 如果该页的历史记录数量不满num_to_read，那么就按实际存了多少个读
		if( (read_idx + num_to_read - 1) > (EVENT_MAX_STORE_NUM - 1) )
		{            
            num_actual_read = EVENT_MAX_STORE_NUM - read_idx;
		}
		else
		{
            num_actual_read = num_to_read;	
		}

		ee_ReadBytes((uint8_t*)pevent, ee_read_addr, ( num_actual_read * sizeof( EE_SysEventDef ) ) );
	    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

		// 更新实际读取的数量
	    *p_num_actual_read = num_actual_read;
	}

	// 给出互斥量
	if( pdTRUE != xSemaphoreGive( xSemEventRW ) )
	{
	    LOG_DEBUG_APP_1("\r\nread event, give mutex failed");	
	}
	else
	{
	    LOG_DEBUG_APP_1("\r\nread event, give mutex success");	 
	}

	LOG_DEBUG_APP_1("\r\n... read event finish--------------------end");

	return TRUE;
}

BOOL is_ee_event_full( uint8_t num )
{
	uint16_t flag_full = 0;

    ee_ReadBytes((uint8_t *)&flag_full, EE_EVENT_IS_FULL_ADDR, EE_EVENT_IS_FULL_LEN );
	vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	if( flag_full & bit(num) )
	{
		LOG_DEBUG_APP_1("\r\nis_ee_event_full, unit %d ee event is full, flag:0x%04X", num, flag_full);
		return TRUE;
	}
	else
	{
		LOG_DEBUG_APP_1("\r\nis_ee_event_full, unit %d ee event not full, flag:0x%04X", num, flag_full);
		return FALSE;
	}
}

static void cache_event( PU_InfoDef *pdev, uint8_t event_code )
{
	Sys_TimeDef time;

	LOG_TRACE("\r\n--------------- start cache_event, num:%d, event_code:%d ......", pdev->basic.num, event_code );

	// 读取当前时间
	get_sys_time( &time );

	pdev->event_cache.time[event_code] = time;

    LOG_TRACE_1("\r\nread time:20%02x-%02x-%02x %02x:%02x:%02x", \
					  time.year, time.month, time.day, \
					  time.hour, time.min, time.sec );

    LOG_TRACE_1("\r\ncache time:20%02x-%02x-%02x %02x:%02x:%02x", \
					  pdev->event_cache.time[ event_code ].year, pdev->event_cache.time[ event_code ].month, pdev->event_cache.time[ event_code ].day, \
					  pdev->event_cache.time[ event_code ].hour, pdev->event_cache.time[ event_code ].min, pdev->event_cache.time[ event_code ].sec );

	// 记录设置对应的事件位
	set_bit( pdev->event_cache.code, event_code );

	LOG_TRACE_1("\r\n... cache_event finish---------------end" );
}

void deal_with_chgr_booking( PU_InfoDef *pdev )
{
	// 未充电
	if( pdev->basic.chgr == BASIC_CHGR_OFF )
	{
		// 判断预约充电
		if( pdev->chgr.st_booking_time == ENABLE )
		{
			pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ] ++;
//			LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d 预约时间，计时中", pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ], pdev->basic.num );

			// 如果预约时间到
			if( pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ] >= ( pdev->chgr.booking_time * 60 ) )
//			if( pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ] >= ( pdev->chgr.booking_time ) )
			{
//				LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d 预约时间到，启动充电", pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ], pdev->basic.num );
//				LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d booking finish, start chgr", pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ], pdev->basic.num );

				// 关闭预约充电
				pdev->chgr.st_booking_time = DISABLE;

				// 清零计数器
				pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ] = 0;

				// 表示预约时间到了
				pdev->chgr.flag_booking = TRUE;

				// 执行启动操作
//				pub_start( START_TYPE_BOOKING, pdev->basic.num );
		    }
        }
	}
}

void deal_with_chgr_timing( PU_InfoDef *pdev )
{
  	// 在充电
  	if( ( pdev->basic.chgr == BASIC_CHGR_ING ) || ( pdev->basic.chgr == BASIC_CHGR_FINISH ) )
  	{
  		// 判断定时充电
  	    if( pdev->chgr.st_timing_time == ENABLE )
  		{
  		    pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ] ++;
//  			LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d  定时时间计时中", pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ], pdev->basic.num );

  			// 如果定时时间到
  		    if( pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ] >= ( pdev->chgr.timing_time * 60 ) )
//    		if( pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ] >= ( pdev->chgr.timing_time ) )
  			{
//				LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d  定时时间到，停止充电", pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ], pdev->basic.num );
//				LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d  timing finish，stop chgr", pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ], pdev->basic.num );

				// 关闭定时充电
                pdev->chgr.st_timing_time = DISABLE;

               // 清空定时计数器
                pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ] = 0;

				// 表示定时时间到了
				pdev->chgr.flag_timing = TRUE;
  			}
  		}
   	}

}

static void deal_with_delay_close_ac( PU_InfoDef *pdev )
{
	int8_t temp = 0;

	if( pdev->basic.flag_delay_close_ac != OFF )
	{
		pdev->chgr.cnt_box[IDX_CNT_DELAY_CLOSE_AC] ++;
				
		// 手动模式下，如果该模块曾经充满过，那么就关闭模块，然后延时8秒后，再打开
		if( pdev->chgr.flag_full == TRUE )
		{
			LOG_DEBUG_APP("\r\n曾经充满过，关闭ac输入, num:%d", pdev->basic.num);
		    power_off( pdev->basic.num );
			if( pdev->chgr.cnt_box[IDX_CNT_DELAY_CLOSE_AC] >= 9 )
			{
				pdev->basic.flag_delay_close_ac = OFF;

				pdev->chgr.flag_full = FALSE;
				LOG_DEBUG_APP("\r\n重新打开AC输入用来散热，num:%d", pdev->basic.num);
			    power_on( pdev->basic.num );
				pdev->chgr.cnt_box[IDX_CNT_DELAY_CLOSE_AC] = 0;
			}
		}

		// 读取模块温度
		temp = ReadTemperature( pdev->basic.addr );
		if( temp < THRESHOLD_TEMP_CLOSE_AC )
		{
			pdev->chgr.cnt_box[ IDX_CNT_FILTER_CLOSE_AC ] ++;
			if( pdev->chgr.cnt_box[ IDX_CNT_FILTER_CLOSE_AC ] >= FILTER_CLOSE_AC )
			{
			    pdev->chgr.cnt_box[ IDX_CNT_FILTER_CLOSE_AC ] = 0;
			    LOG_DEBUG_APP("\r\nunit %d temp %d < %d, close ac, time cost about: %ds", pdev->basic.num, temp, THRESHOLD_TEMP_CLOSE_AC, pdev->chgr.cnt_box[IDX_CNT_DELAY_CLOSE_AC] );

			    power_off( pdev->basic.num );

			    pdev->chgr.cnt_box[IDX_CNT_DELAY_CLOSE_AC] = 0;
			    pdev->basic.flag_delay_close_ac = OFF;
			}
		}
		else
		{
			pdev->chgr.cnt_box[ IDX_CNT_FILTER_CLOSE_AC ] = 0;
		}

#if 0
		if( pdev->chgr.cnt_box[IDX_CNT_DELAY_CLOSE_AC] >= TIMEOUT_CLOSE_AC )
		{
			LOG_DEBUG_APP("\r\n延时关闭模块时间到，关闭模块AC输入, num:%d", pdev->basic.num);
			power_off( pdev->basic.num );
			pdev->chgr.cnt_box[IDX_CNT_DELAY_CLOSE_AC] = 0;
			pdev->basic.flag_delay_close_ac = OFF;
		}
#endif
	}
}

void deal_with_bms_timeout( PU_InfoDef *pdev )
{

	    pdev->chgr.cnt_box[ IDX_CNT_BMS_TIMEOUT ] ++;
		if( (pdev->basic.flag_bms_conn == ON) && (pdev->chgr.cnt_box[ IDX_CNT_BMS_TIMEOUT ] >= TIMEOUT_MS_BMS/1000) )
		{
            pdev->chgr.cnt_box[ IDX_CNT_BMS_TIMEOUT ] = 0;

			pdev->basic.flag_bms_conn = OFF;
		    pdev->chgr.flag_bms_timeout = TRUE;
		}
}

static void start( PU_InfoDef *pdev )
{
	// 打开输出
    open_output( pdev->basic.num );

	// 清空计数器
	memset( pdev->chgr.cnt_box, 0, sizeof(pdev->chgr.cnt_box ) );
	
	// 设置延时关闭AC为OFF，防止打开输出过程中关闭模块
	pdev->basic.flag_delay_close_ac = OFF;
	
	// 清除充满标志位
	pdev->chgr.flag_full = FALSE;

	// 设置充电状态为启动中
	pdev->basic.chgr = BASIC_CHGR_STARTING;
	
	// 进入open output 阶段
    pdev->chgr.step = CHGR_STEP_OPEN_OUTPUT;
	
    add_to_chgr_list( pdev );

	// ...
	pdev->chgr.flag_in_chgr = TRUE;
}

static void start_success( PU_InfoDef *pdev )
{
	uint8_t *local_bat_info = NULL;
	
	// 设置模块状态为：没有异常
	pdev->basic.err = BASIC_ERR_NO;
	pdev->basic.warn = BASIC_WARN_NO;
	// 设置模块标志位，表示电池在充电中
	pdev->basic.bat = BASIC_BAT_CHGR_ING;
	// 设置模块标志位，表示充电中
//	pdev->basic.chgr = BASIC_CHGR_ING;
	// 设置延时关闭AC为OFF
	pdev->basic.flag_delay_close_ac = OFF;
	
	// 设置当前op状态为 start状态
	pdev->chgr.op = op_start;

    // 更新电池缓存信息，表示充电中
	local_bat_info = get_local_bat_info();
	set_bit( local_bat_info[pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_CHGRING );
	
	// 添加启动事件
	cache_event( pdev, EVENT_START );

	// 清除错误标志
    reset_bit( pdev->basic.err, ERR_DC_OK );
	
	LOG_DEBUG_APP_1( " ....start num:%d success", pdev->basic.num );
	
	// 如果是界面启动，发送给touch_screen_entry通知，启动成功
	if( pdev->chgr.start_type == START_TYPE_HMI )
	{
	    xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_START_SUCCESS, eSetBits );
	}
}

static void start_failed( PU_InfoDef *pdev )
{
	// 清空计数器
	memset( pdev->chgr.cnt_box, 0, sizeof(pdev->chgr.cnt_box ) );
		
	// stop掉启动失败的设备
	stop( pdev );
	
	// 启动失败，直接关闭ac输入
	power_off( pdev->basic.num );

	// 设置延时关闭AC为OFF
	pdev->basic.flag_delay_close_ac = OFF;
	
	// 如果是界面启动的，发送给touch_screen_entry通知，启动失败
	if( pdev->chgr.start_type == START_TYPE_HMI )
	{
	    xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_START_FAILED, eSetBits );
	}
	
	LOG_DEBUG_APP_1( " ....start num:%d failed", pdev->basic.num );
}

static void stop( PU_InfoDef *pdev )
{	
	uint8_t *local_bat_info = NULL;
	
	// 关闭对应模块的DC输出
	close_output( pdev->basic.num );
	
	// 设置模块标志位，表示未充电
	pdev->basic.chgr = BASIC_CHGR_OFF;
    // 设置风扇转速为0，因为此时模块已经关闭了
	pdev->basic.fan_speed = 0;
	// 设置延时关闭AC为ON，开始计时
	pdev->basic.flag_delay_close_ac = ON;
	pdev->basic.temp = 25;

	// 设置充电步骤为空闲，表示未充电
    pdev->chgr.step = CHGR_STEP_IDLE;
	// 设置模块状态为:0
    pdev->chgr.status = (uint16_t)0;
	
	// 设置当前op状态为 stop状态
	pdev->chgr.op = op_stop;

    // 更新电池缓存信息，表示未充电
	local_bat_info = get_local_bat_info();
	reset_bit( local_bat_info[pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_CHGRING );
	
	// 清空对应模块的数据
    pdev->chgr.vout = 0;
	pdev->chgr.iout = 0;
	pdev->chgr.time = 0;
	memset( pdev->chgr.cnt_box, 0, sizeof(pdev->chgr.cnt_box ) );

	pdev->chgr.iout_ori = 0;
	pdev->chgr.iout_disp = 0;
	memset( &pdev->chgr.iout_queue, 0, sizeof( Queue_Float ) );
	
	   // 自动充电部分
	pdev->chgr.auto_limit_iout = 0;
	pdev->chgr.auto_bat_volt = 0;
	pdev->chgr.auto_bat_iout = 0;
    pdev->chgr.auto_bat_cap = 0;  
//	pdev->chgr.auto_bat_default_cap = 0; 
	pdev->chgr.auto_bat_temp = 0;
	pdev->chgr.auto_vout_step = 0;
    pdev->chgr.auto_flag_full_bms = FALSE;
    pdev->chgr.auto_flag_full_mcu = FALSE;
    pdev->chgr.auto_flag_full_threshold = FALSE;

	// 清除是否需要cache电池断开事件的标志位
	pdev->flag_bat_disconn_event_cached = FALSE;

	/* 停止处理：
	 * 1.直接移除即可，因为调用stop之后，不会继续执行该设备的充电操作( 调用时需要确保这一点 )
	 */
	LOG_DEBUG_APP_1("\r\nremove unit:%d from chgr list", pdev->basic.num);
	remove_from_chgr_list( pdev->basic.num );

	// ...
	pdev->chgr.flag_in_chgr = FALSE;

	// 添加停止事件
	cache_event( pdev, EVENT_STOP );

	LOG_DEBUG_APP_1(" ....num:%d stop finish", pdev->basic.num );
}

static void restart_dev( PU_InfoDef *pdev )
{
	uint32_t timer = 0;
    // 等待充电任务的执行完毕，每10ms查询一次，最长时间等待4s
	while( (__chgr_damon_step != __END) || (timer > 400) ) { timer ++; vTaskDelay( pdMS_TO_TICKS(10) ); };
	
	// 挂起充电任务
	if( eSuspended != eTaskGetState( xChargeDamonHandle ) )
		vTaskSuspend( xChargeDamonHandle );
	
	// 关闭AC输入
	close_ac_input( pdev->basic.num );
	
	// 延时 10s钟
	vTaskDelay( pdMS_TO_TICKS(10000) );
	
	// 打开ac输入
	open_ac_input( pdev->basic.num );
	
	// 延时3s，等待模块稳定
	vTaskDelay( pdMS_TO_TICKS(3000) );
	
	if( eSuspended == eTaskGetState( xChargeDamonHandle ) )
	{
	    vTaskResume( xChargeDamonHandle );
	}
	
    // 发送给touch_screen_entry通知，重启完成	
	LOG_TRACE("\r\nsend restart finish to touch screen.... ");
	xTaskNotify( h_touch_screen_entry, SET_NOTIFY_RESTART_FINISHED, eSetBits );
}

static void restart_dev_all( void )
{
	// 关闭所有模块的AC输入
	close_ac_input_all();

	// 延时 10s钟
	vTaskDelay( pdMS_TO_TICKS(10000) );

	// 打开所有模块的AC输入
    open_ac_input_all();

	// 延时3s，等待模块稳定
	vTaskDelay( pdMS_TO_TICKS(3000) );

    // 发送给touch_screen_entry通知，重启完成
	LOG_TRACE("\r\nsend restart all finish to touch screen.... ");
	xTaskNotify( h_touch_screen_entry, SET_NOTIFY_RESTART_FINISHED, eSetBits );
}

static void chk_bat_conn_st( PU_InfoDef *pdev )
{
	float volt = 0;
	uint8_t *p_local_bat_info = NULL;

	// 获取本地电池信息
	p_local_bat_info = get_local_bat_info();
	
	// 读取采样的电池电压
    volt = pdev->basic.bat_volt;
	
	// 如果处于打开输出阶段或者充电中，获取采样电压值后，不用进行下面的检测
	if( (pdev->basic.chgr == BASIC_CHGR_STARTING) || \
	    (pdev->basic.chgr == BASIC_CHGR_ING) || \
	    (pdev->basic.chgr == BASIC_CHGR_FINISH) )
		return;
	if( pdev->chgr.step == CHGR_STEP_IDLE )
        pdev->chgr.step = CHGR_STEP_CHK_BAT_REVERSE;
	
    if( pdev->chgr.step == CHGR_STEP_CHK_BAT_REVERSE )
    {
    	// 检测电池是否反接
    	if( isBatReverse( volt ) )
        {
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] --; 
    	}
    	else
    	{
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] ++; 
    	}
    
    	// 如果检测通过 
    	if( pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] >= (int32_t)FILTER_CHK_BAT_REVERSE )
    	{
    		// 接下来检测电池有没有连接
            pdev->chgr.step = CHGR_STEP_CHK_BAT_CONN;
    
    		// 清除错误标志
            reset_bit( pdev->basic.err, ERR_BAT_REVERSE );
    		
    		// 更新本地电池信息缓存中，电池反接信息
            reset_bit(p_local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_REVERSE);
    
    		// 清零计数器
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] = 0;
    	}
    	// 如果检测未通过 
    	else if( pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] <= (int32_t)( FILTER_CHK_BAT_REVERSE * (-1) ) )
    	{
    		// 设置电池反接标志
    		pdev->basic.bat = BASIC_BAT_REVERSE;
    
    		// 设置错误标志
            set_bit( pdev->basic.err, ERR_BAT_REVERSE );
    
    		// 更新本地电池信息缓存中，电池反接信息
            set_bit(p_local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_REVERSE);
    		if( !( pdev->basic.err & bit( ERR_BAT_REVERSE ) ) )
    		{
    		    // 记录电池反接事件
    		    cache_event( pdev, EVENT_BAT_REVERSE );
    		}
    		
    		// 返回空闲，继续检测
    	    pdev->chgr.step = CHGR_STEP_IDLE;
    
    		// 清零计数器
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] = 0;
    	}
    }
    
    if( pdev->chgr.step == CHGR_STEP_CHK_BAT_CONN )
    {
    	// 检测电池是否连接
    	if( isBatConn( volt ) )
    	{
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] ++; 
    	}
    	else
    	{
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] --; 
    	}
    
    	// 如果检测通过 
    	if( pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] >= (int32_t)FILTER_CHK_BAT_CONN )
    	{
    		// 设置电池连接标志
    		pdev->basic.bat = BASIC_BAT_CONN;
    
    		// 更新本地电池信息缓存中，电池连接信息
            set_bit(p_local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_CONN);
    		
            // 获取电池电压,更新电池电压，供主页显示 
            pdev->basic.bat_volt = volt;
    
    		// 清零计数器
            pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] = 0;
    
    		// 返回空闲状态，重新检测
    	    pdev->chgr.step = CHGR_STEP_IDLE;
    	}
    	// 如果检测未通过 
    	else if( pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] <= (int32_t)( FILTER_CHK_BAT_CONN * (-1) ) )
    	{
    		// 设置电池未连接标志
    		pdev->basic.bat = BASIC_BAT_DISCONN;
    		
    		// 更新本地电池信息缓存中，电池连接信息
            reset_bit(p_local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_CONN);
    
    		// 清零计数器
            pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] = 0;
    
    		// 返回空闲状态，重新检测
    	    pdev->chgr.step = CHGR_STEP_IDLE;
    	}
    }
}

static BOOL chk_vout_in_starting( PU_InfoDef *pdev )
{
	float vout, volt, set_vout;
	StatusTypeDef st_pmbus;

	// 判断检测是否超时
    if( pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] >= chgrMS_TO_CNT( TIMEOUT_CHK_VOUT_IN_STARTING ) )
	{
	    LOG_WARNING_APP("%dms has been passed, chk vout failed in starting, stop",(uint16_t)TIMEOUT_CHK_VOUT_IN_STARTING );
        pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] = 0;
		stop( pdev );
		return FALSE;
	}

	// 读电池电压
	volt = pdev->basic.bat_volt;
	LOG_DEBUG_APP_1("\r\nvolt:%f", volt);

	// 读模块输出电压
	ReadVout_St( pdev->basic.addr, &vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
	LOG_DEBUG_APP_1("\r\nread_vout:%f ", vout );
	disp_pmbus_comm_status( st_pmbus );
	if( ( vout < 0 ) || ( vout - pdev->chgr.data_max.vbst > 1 ) )
	{
			LOG_WARNING_APP("\r\nread vout err, return" );
			return FALSE;
	}

	// 判断输出电压是否正常
	if( ( vout - VOUT_SUCCESS_THRESHOLD ) >= 0 )
	{
        LOG_INFO_APP("vout:%f, >= %f, chk vout success, cost:%.1fs, to next step...", vout, (float)VOUT_SUCCESS_THRESHOLD, (float)(pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ]*CHGR_DAMON_PERIOD) / 1000 );
        pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] = 0;
		// 接下来检测输出电流
	    pdev->chgr.step = CHGR_STEP_CHECK_OUTPUT_IOUT;
   	    // 更新基准电池信息
   	    pdev->chgr.volt_base = volt;
		/* 如果是预充电模式，根据电池电压判断预充电还是恒流，
		 * 从而确定接下来判断电流过程中，调节电流时的限定值
		 */
		if( pdev->chgr.type == CHGR_TYPE_PRE )
		{
			if( volt - pdev->chgr.para.data_pre.pre_vout <= 0 )
                pdev->chgr.limit_iout = pdev->chgr.para.data_pre.pre_iout;
			else
                pdev->chgr.limit_iout = pdev->chgr.para.data_pre.ichg;
		}

		return TRUE;
	}

	// 针对自动和预充需要设置输出电压，三段式不需要
    if( pdev->chgr.mode == MODE_MANUAL )
	{
		if( pdev->chgr.type == CHGR_TYPE_THREE_SECTION )
			return FALSE;
		else if( pdev->chgr.type == CHGR_TYPE_PRE )
			set_vout = volt;
	}
	else if( pdev->chgr.mode == MODE_AUTO )
	{
		set_vout = volt;
	}
    SetVout_St( pdev->basic.addr, set_vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
    LOG_DEBUG_APP_1("\r\nset_vout:%f ", set_vout );
    disp_pmbus_comm_status( st_pmbus );

	return FALSE;
}

static BOOL chk_iout_in_starting( PU_InfoDef *pdev )
{
	float volt, iout, limit_iout, vout, set_vout;
    uint8_t *local_bat_info = NULL;
	StatusTypeDef st_pmbus;

	// 读电池电压
    volt = pdev->basic.bat_volt;
	LOG_DEBUG_APP_1("\r\nvolt:%f", volt);

	// 判断检测是否超时
    if( pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] >= chgrMS_TO_CNT( TIMEOUT_CHK_IOUT_IN_STARTING ) )
	{
        pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] = 0;
	    LOG_WARNING_APP("%dms has been passed, chk iout failed in starting, close output, and chk bat conn",(uint16_t)TIMEOUT_CHK_IOUT_IN_STARTING );
	    // 关闭输出
	    close_output( pdev->basic.num );
		pdev->chgr.step = CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING;
		pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;

		return FALSE;
	}

	// 判断电流是否正常
	    // 读取充电电流
	ReadIout( pdev->basic.addr, &iout );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
	LOG_DEBUG_APP_1("\r\nread iout:%f", iout);
	if( (iout > pdev->chgr.data_max.ichg + 1) || ( iout < 0 ) )
	{
		LOG_WARNING_APP("read iout err, return");
		return FALSE;
	}
	if( iout > 0 )
	{
	    pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT_SUS ] ++;
		if( pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT_SUS ] >= 3 )
		{
			pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT_SUS ] = 0;
		    LOG_INFO_APP("iout:%f > 0A 3-times, chk iout success, cost:%.1fs, to next step...", iout, (float)(pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ]*CHGR_DAMON_PERIOD) / 1000 );
			pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] = 0;
			// 设置当前通道处于充电中
			pdev->basic.chgr = BASIC_CHGR_ING;
			/* 接下来检测处于充电中的哪个状态
			 * 对于自动充电，直接进入cvm步骤
			 */
			if( pdev->chgr.mode != MODE_AUTO )
				pdev->chgr.step = CHGR_STEP_CHECK_STAGE;
			else 
				pdev->chgr.step = CHGR_STEP_CVM;

			return TRUE;
		}
	}
	else
	{
	    pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT_SUS ] = 0;
	}
	
	// 调节电流
	// 对于三段式不需要调节，自动和预充需要调节
	if( pdev->chgr.mode == MODE_MANUAL  )
	{
		if( pdev->chgr.type == CHGR_TYPE_THREE_SECTION )
			return FALSE;
		else if( pdev->chgr.type == CHGR_TYPE_PRE )
			limit_iout = pdev->chgr.limit_iout;
	}
	else if( pdev->chgr.mode == MODE_AUTO  )
	{
        // 使用读取的限制电流
		limit_iout = pdev->chgr.auto_limit_iout; 
	}

	LOG_DEBUG_APP_1("\r\n<-----iout control----->");
	if( limit_iout - 5.5 >= 0 )  // ioc >= 5.5，使用B46指令设置限制电流即可
	{
		LOG_DEBUG_APP_1("\r\nlimit >= 5.5, use B46");
	    // 写入限压值
	    if( pdev->chgr.mode == MODE_AUTO  )
			set_vout = (float)pdev->bat_info_bms.full_volt/10;
		else if( pdev->chgr.type == CHGR_TYPE_PRE )
            set_vout = pdev->chgr.para.data_pre.vbst;

        SetVout_St( pdev->basic.addr, set_vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
	    LOG_DEBUG_APP_1("\r\nset_vout:%f ", set_vout );
        disp_pmbus_comm_status( st_pmbus );

	    // 写入限流值
	    LOG_DEBUG_APP_1("\r\nset limit_iout:%f ", limit_iout);
        SetIoutOC_FaultLimit_St( pdev->basic.addr, limit_iout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
        disp_pmbus_comm_status( st_pmbus );
	}
	else
	{
		LOG_DEBUG_APP_1("\r\nlimit < 5.5, use mcu control");

        // 读取充电电压
        ReadVout_St( pdev->basic.addr, &vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
        disp_pmbus_comm_status( st_pmbus );
		LOG_DEBUG_APP_1("\r\nread_vout:%f", vout );
		if( ( vout < 0 ) || ( vout - pdev->chgr.data_max.vbst > 1 ) )
        {
			LOG_WARNING_APP("read vout err, return" );
			return FALSE;
		}
	    if( pdev->chgr.mode == MODE_AUTO  )
		    chgr_control_little_iout( pdev, vout, iout, limit_iout );
		else
		    chgr_control_little_iout_in_starting( pdev, vout, iout, limit_iout );
	}

	return FALSE;
}

static BOOL chgr_starting( PU_InfoDef *pdev )
{
	StatusTypeDef st_pmbus;
	
	// 计数器计时
    pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] ++;
    LOG_DEBUG_APP_1("\r\ncnt:%u", pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ]);

	switch( pdev->chgr.step )
	{
		case CHGR_STEP_OPEN_OUTPUT:
	        // 打开on/off输出后，需要延时1.5s，否则会导致SetVout失败
			if( pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] == chgrMS_TO_CNT( 200 ) )
			{
                start_success( pdev );
				break;
			}
			else if( pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] == chgrMS_TO_CNT( OPEN_ON_OFF_DELAY ) )
			{
	            // 打开模块输出
	            on_off_control( pdev->basic.num, ON );
				LOG_INFO_APP("enbale on/off");
				break;
			}
			else if( pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] >= chgrMS_TO_CNT(1500) )
			{
				if( ( pdev->chgr.mode == MODE_AUTO ) || ( pdev->chgr.type == CHGR_TYPE_PRE ) )
				{
					// 设置限流值，防止超过限制值（limit_iout <5.5，实际值为5.5）
					SetIoutOC_FaultLimit_St( pdev->basic.addr, pdev->chgr.limit_iout, &st_pmbus );		
				}

				// 清零计数器，防止影响下一个阶段的检测
	            pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] = 0;

				// 接下来检测输出
				pdev->chgr.step = CHGR_STEP_CHECK_OUTPUT; 

				LOG_INFO_APP("1.5s has been passwd, to check output");
			}
			else
                return FALSE;
		case CHGR_STEP_CHECK_OUTPUT:
		    pdev->chgr.step = CHGR_STEP_CHECK_OUTPUT_VOUT;
		case CHGR_STEP_CHECK_OUTPUT_VOUT:
			if( !chk_vout_in_starting( pdev ) )
				return FALSE;
		case CHGR_STEP_CHECK_OUTPUT_IOUT:
            return chk_iout_in_starting( pdev );
		case CHGR_STEP_RESTART:
			LOG_INFO_APP("restart");
			open_output( pdev->basic.num );
			on_off_control( pdev->basic.num, ON );
			pdev->chgr.step = CHGR_STEP_CHECK_OUTPUT;
	        pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] = 0;
			break;
		case CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING:
			chk_bat_conn_in_charging( pdev );
			break;
		default:
		    break;
	}

    return FALSE;
}

static void charging( PU_InfoDef *pdev )
{
	assert_app( pdev == NULL );
	if( pdev == NULL ) return;

	LOG_DEBUG_APP_1("\r\n\r\n[uint-->%d]", pdev->basic.num);
	LOG_DEBUG_APP_1("\r\nin chgring, chgr mode: ");

    if( pdev->chgr.mode == MODE_MANUAL )
	{
		LOG_DEBUG_APP_1("manul_mode\r\nchgr type: ");

		if( pdev->chgr.type == CHGR_TYPE_THREE_SECTION )
		{
		    LOG_DEBUG_APP_1("3-section");
	        chgr_manual_with_3_section( pdev );
		}
		else if( pdev->chgr.type == CHGR_TYPE_PRE )
		{
		    // 测试预充电，把模式切换到PMBus控制模式
		    LOG_DEBUG_APP_1("pre-chgr");
		    chgr_manual_with_pre( pdev );
		}
	}
	else if( pdev->chgr.mode == MODE_AUTO )
	{
		LOG_DEBUG_APP_1("auto_mode");
	    chgr_auto( pdev );
	}

    clear_chgr_info( pdev );
}

static void chgr_auto( PU_InfoDef *pdev )
{
	uint16_t status = 0;
	float iout = 0;
	float vout = 0;
	float set_vout = 0;
	float limit_iout = 0;
	float ave_iout = 0;
	float volt = 0;
    uint8_t *local_bat_info = NULL;
	int8_t temperature;
	StatusTypeDef st_pmbus;

	LOG_DEBUG_APP_1("\r\nchgr_time:%l s", pdev->chgr.time);
	if( pdev->basic.chgr == BASIC_CHGR_STARTING )
	{
		// 检测是否充满
		if( is_auto_chgr_bat_full( pdev ) )
	    {
	        LOG_DEBUG_APP("num:%d 已经充满，关闭输出", pdev->basic.num);

	        // 充满后，关闭输出
	        close_output( pdev->basic.num );
            pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] = 0;
	    	
	    	// 设置充电完成
	    	pdev->basic.chgr = BASIC_CHGR_FINISH;
			pdev->basic.bat = BASIC_BAT_FULL;
	    	
	    	// 关闭后，设置输出电压和输出电流为0
	    	pdev->chgr.vout = 0;
	    	pdev->chgr.iout = 0;

			// 设置步骤为cvm，锂电池充电，全程电压保持不变，所以这里定义为cvm
		    pdev->chgr.step = CHGR_STEP_CVM;
	    	
	    	return;
	    }

		if( !chgr_starting( pdev ) )
			return;
	}

	// 读取电池电压
	volt = pdev->basic.bat_volt;
	LOG_DEBUG_APP_1("\r\nvolt:%f", volt);

	// 检测过温异常
	if( !chk_temperature( pdev ) )
		return;
		
	// 检测充电过程中，电池是否断开
	if( is_vout_dispear_in_auto_mode( pdev ) )
	{
		LOG_DEBUG_APP("\r\n采样电池电压消失，关闭充电 ");
	    stop( pdev );
		return;
	}
	
	// 充电中
	if( pdev->basic.chgr == BASIC_CHGR_ING )
	{
		// 检测充电超时
		// 自动充电，4h超时计时，从启动充电开始计时，如果超过4h，满电标志仍然没有置位，认为充满
		if( pdev->chgr.time >= (uint32_t)TIMEOUT_MIN_AUTO_CHGR )
		{
			pdev->chgr.auto_flag_full_mcu = TRUE;
			LOG_DEBUG_APP_1("\r\n4h timeout, bms full_flag not set, then set chgr_station enter chgr finish");

		    // 充满后，关闭输出
		    close_output( pdev->basic.num );
			
			// 设置充电完成
			pdev->basic.chgr = BASIC_CHGR_FINISH;
			pdev->basic.bat = BASIC_BAT_FULL;
			
			// 关闭后，设置输出电压和输出电流为0
			pdev->chgr.vout = 0;
			pdev->chgr.iout = 0;
	        pdev->chgr.iout_ori = 0;
	        pdev->chgr.iout_disp = 0;
	        pdev->chgr.auto_vout_step = 0;

			return;
		}

	    // 检测输出异常
	    if( !chk_dc_output( pdev ) )
	    	return;

		// 判断电池是否已经充满
		if( is_auto_chgr_bat_full( pdev ) )
		{
		    LOG_DEBUG_APP("num:%d 已经充满，关闭输出", pdev->basic.num);

		    // 充满后，关闭输出
		    close_output( pdev->basic.num );
			
			// 设置充电完成
			pdev->basic.chgr = BASIC_CHGR_FINISH;
			pdev->basic.bat = BASIC_BAT_FULL;
			
			// 关闭后，设置输出电压和输出电流为0
			pdev->chgr.vout = 0;
			pdev->chgr.iout = 0;
	        pdev->chgr.iout_ori = 0;
	        pdev->chgr.iout_disp = 0;
	        pdev->chgr.auto_vout_step = 0;
			
			return;
		}

	    // 处理充电过程中，已经检测到输出电流为0的情况
	    if( pdev->chgr.step == CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING )
	    {
			chk_bat_conn_in_charging( pdev );
	    	return;
	    }
	
		LOG_DEBUG_APP_1("\r\n\r\nnum:%d, limit iout:%f, bat volt:%f, bat_iout:%f", pdev->basic.num, pdev->chgr.auto_limit_iout, pdev->chgr.auto_bat_volt, pdev->chgr.auto_bat_iout);

		// 读取充电电流
		ReadIout( pdev->basic.addr, &iout );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif

		LOG_DEBUG_APP_1("\r\nread iout:%f", iout);
		if( (iout > pdev->chgr.data_max.ichg + 1) || ( iout < 0 ) ) { LOG_DEBUG_APP_1("\r\nread iout err, return"); return; }

		// 写入iout queue，并计算平均值
		write_iout_to_queue( pdev, iout );
		calc_iout_ave( pdev );
		ave_iout = pdev->chgr.iout;
		LOG_DEBUG_APP_1("\r\nave_iout:%f", ave_iout);

		// 读取充电电压
		ReadVout_St( pdev->basic.addr, &vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
		disp_pmbus_comm_status( st_pmbus );
		LOG_DEBUG_APP_1("\r\nread_vout:%f", vout );
		if( ( vout < 0 ) || ( vout - pdev->chgr.data_max.vbst > 1 ) )
		{
			LOG_DEBUG_APP_1("\r\nread vout err, return" );
			return;
		}
		
	    // 如果充电过程中，输出电流为0，进入检测电池是否断开的步骤
        if( is_iout_disappear_in_charge( pdev, iout ) )
	    {
#if 0
			// 20190926添加，如果满电标志没有置起来，检测到电流为0仍然继续充
			if( pdev->chgr.auto_flag_full_bms != TRUE )
			{
			    LOG_DEBUG_APP_1("\r\nin auto chgr, iout disappear, but full_flag is false, chgring continue...");
	    	    pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;
			}
			else
#endif
			{
		        LOG_DEBUG_APP_1("\r\nin auto chgr, iout disappear, close output, enter next step->chk bat_diss_in_charging");

                close_output( pdev->basic.num );
	    	    pdev->chgr.step = CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING;
	    	    pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;
	    	    return;
			}
	    }

/************************* 控制部分 ****************************/
		limit_iout = pdev->chgr.auto_limit_iout; // 使用读取的限制电流

		if( limit_iout - 5.5 >= 0 )  // ioc >= 5.5，使用B46指令设置限制电流即可
		{
			LOG_DEBUG_APP_1("\r\nlimit >= 5.5, use B46");
			// 写入限压值
			set_vout = (float)pdev->bat_info_bms.full_volt/10;
			SetVout_St( pdev->basic.addr, set_vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
			disp_pmbus_comm_status( st_pmbus );
			LOG_DEBUG_APP_1("\r\nset_vout:%f", set_vout );

			// 写入限流值
			LOG_DEBUG_APP_1("\r\nset limit_iout:%f", limit_iout);
			SetIoutOC_FaultLimit_St( pdev->basic.addr, limit_iout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
			disp_pmbus_comm_status( st_pmbus );
//			ReadIoutOC_FaultLimit_St( pdev->basic.addr, &limit_read, &st_pmbus );
//			LOG_DEBUG_APP_1("\r\nread limit_iout:%f", limit_read);

		    // 更新电压基准值，处理当电流从>5.5慢慢降到<5.5时，由于基准值过低导致输出电流为0的问题
   	        pdev->chgr.volt_base = volt;
		    pdev->chgr.auto_vout_step = 0;
		}
		else
		{
			LOG_DEBUG_APP_1("\r\nlimit < 5.5, use mcu control");
	//        LOG_DEBUG_APP_1("\r\niout: 1:%f, 2:%f, 3:%f, 4:%f", pdev->chgr.iout_queue.buff[0], pdev->chgr.iout_queue.buff[1], pdev->chgr.iout_queue.buff[2], pdev->chgr.iout_queue.buff[3]);
	//		chgr_control_little_iout( pdev, vout, ave_iout, limit_iout );
			chgr_control_little_iout( pdev, vout, iout, limit_iout );
		}

		// 更新输出电压、电流，过滤异常电压，电流
		if( ( vout <= pdev->chgr.data_max.vbst ) && ( vout >= 0 ) )
			pdev->chgr.vout = vout;
		
		if( ( iout <= pdev->chgr.data_max.ichg ) && ( iout >= 0 ) )
		{
			pdev->chgr.iout_ori = iout;
			
			// 界面显示的电流
			// iout在limit_iout的[-0.5, +0.5]，范围内，均显示limit_iout
			/*
			 *
			 * ------------------      iout_disp = iout
			 *
			 * ------------------ iout            |
			 *        |-> +0.5                    |
			 * ------------------ iout_limit      |  -> iout_disp = iout_limit
			 *        |-> -0.5                    |
			 * ------------------ iout            |
			 *
			 * ------------------      iout_disp = iout
			 *
			 */
			if( ( ( iout <= limit_iout ) && ( ( iout + 0.5 ) >= limit_iout ) ) || \
				( ( iout > limit_iout ) && ( ( limit_iout + 0.5 ) >= iout ) )
			  )
			{
				pdev->chgr.iout_disp = limit_iout;
			}
			else
			{
//				pdev->chgr.iout_disp = ave_iout;
				pdev->chgr.iout_disp = iout;
			}
		}
		LOG_DEBUG_APP_1("\r\niout_disp:%.1f", pdev->chgr.iout_disp);
    }
	else if( pdev->basic.chgr == BASIC_CHGR_FINISH ) // 充电完成
	{
	    ;
	}

#if 0
	vout_add = 0.8 * ( pdev->chgr.auto_limit_iout - pdev->chgr.auto_bat_iout ) * 0.2;
	set_vout = pdev->chgr.auto_bat_volt + vout_add;
    SetVout( pdev->basic.addr, set_vout );
	vTaskDelay( pdMS_TO_TICKS(10) );
	LOG_DEBUG_APP_1("\r\nset vout:%f", set_vout);
#endif


// 更新相关信息
	// 读模块温度	
	temperature = ReadTemperature( pdev->basic.addr );
	if( ( temperature - pdev->basic.temp_before >= 10 ) || ( temperature - pdev->basic.temp_before <= -10 ) )
	{
	    pdev->basic.temp = pdev->basic.temp_before;
	}
	else
	{
	    pdev->basic.temp = temperature;
        pdev->basic.temp_before = temperature;
	    LOG_DEBUG_APP_1("\r\ntemperature:%d", pdev->basic.temp);
	}

	// 读风扇转速
	pdev->basic.fan_speed = ReadFanSpeed_1( pdev->basic.addr );
	LOG_DEBUG_APP_1("\r\nfan_speed:%d\r\n\r\n", pdev->basic.fan_speed);
	
    // 更新模块状态
	ReadChgStatus( pdev->basic.addr, &status );
	pdev->chgr.status = status;
}

static void chgr_manual_with_3_section( PU_InfoDef *pdev )
{
	uint16_t status = 0;
	float iout = 0;
	float vout = 0;
	float volt = 0;
	int8_t temperature;
	uint8_t *local_bat_info;
	StatusTypeDef st_pmbus;
	
	if( pdev->basic.chgr == BASIC_CHGR_STARTING )
	{
		if( !chgr_starting( pdev ) )
			return;
	}

	// 读取电池电压
	volt = pdev->basic.bat_volt;
	LOG_DEBUG_APP_1("\r\nvolt:%f\r\n", volt);

	// 处理充电过程中，已经检测到输出电流为0的情况
	if( pdev->chgr.step == CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING )
	{
		chk_bat_conn_in_charging( pdev );
#if 0
		pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] ++;
		if( !isBatConn( volt ) )
		{
            pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;
			LOG_DEBUG_APP("\r\n充电中检测电池断开阶段，电池确实断开");
			stop( pdev );
		}
		else if( pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] >= chgrMS_TO_CNT(4000) )
		{
            pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;

			if( volt >= (float)THRESHOLD_VOLT_IN_IOUT_DIS )
			{
				// 设置充电完成
				pdev->basic.chgr = BASIC_CHGR_FINISH;
				pdev->basic.bat = BASIC_BAT_FULL;

				// 更新电池缓存信息中，设置电池充满标志
				local_bat_info = get_local_bat_info();
				set_bit( local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_FULL );
				// 关闭后，设置输出电压和输出电流为0
				pdev->chgr.vout = 0;
				pdev->chgr.iout = 0;

				pdev->chgr.step = CHGR_STEP_FVM;

				LOG_DEBUG_APP("\r\n充电中检测电池断开阶段，电池没有断开，认为充满，进入浮充状态");
			}
			else
			{
				LOG_DEBUG_APP("\r\nvolt:%f < %f", volt, (float)THRESHOLD_VOLT_IN_IOUT_DIS );
				LOG_DEBUG_APP_1(" 非法停止");
				stop( pdev );
				cache_event( pdev, EVENT_ILLEGAL_STOP );
			}
		}
#endif
		return;
	}

	// 只有在充电过程中，才去读取电流和电压
	if( pdev->basic.chgr != BASIC_CHGR_FINISH )
	{
	    ReadVout( pdev->basic.addr, &vout );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif

	    ReadIout( pdev->basic.addr, &iout );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
        if( (iout - 28.5 > 0 ) || ( iout < 0 ) ) 
        { 
	        LOG_DEBUG_APP_1("\r\nerror data, use pre iout->%f", pdev->chgr.iout_pre); 
	    	iout = pdev->chgr.iout_pre; //  使用上次的电流
	    }
	    pdev->chgr.iout_pre = iout;

	    // 更新输出电压、电流，过滤异常电压，电流
	    if( ( vout <= pdev->chgr.data_max.vbst ) && ( vout >= pdev->chgr.data_min.vbst ) )
	        pdev->chgr.vout = vout;
	    if( ( iout <= pdev->chgr.data_max.ichg ) && ( iout >= 0 ) )
		{
	        pdev->chgr.iout_ori = iout;

			if( pdev->chgr.step == CHGR_STEP_CCM )
			    pdev->chgr.iout_disp = pdev->chgr.para.data.ichg;
			else
			    pdev->chgr.iout_disp = iout;
		}

		LOG_DEBUG_APP_1("addr:%02X read vout:%f read iout:%f ", pdev->basic.addr, vout, iout);
	}

	// 如果充电过程中，输出电流为0，进入检测电池是否断开的步骤
	if( is_iout_disappear_in_charge( pdev, iout ) )
	{
		LOG_DEBUG_APP_1("\r\n模块%d输出电流为0，关闭输出，进入下一步", pdev->basic.num);

        close_output( pdev->basic.num );
		pdev->chgr.step = CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING;
		return;
	}

	ReadChgStatus_St( pdev->basic.addr, &status, &st_pmbus );
	LOG_DEBUG_APP_1("status:%04X ", status);
	disp_pmbus_comm_status( st_pmbus );
	if( status > ( FULLM|CCM|CVM|FVM|EEPER|NTCER|BTNC|CCTOF|CVTOF|FVTOF) )
	{
	    LOG_DEBUG_APP_1(" status err, return");
		return;
	}
	pdev->chgr.status = status;
	

	// 检测输出异常
	if( !chk_dc_output( pdev ) )
		return;

	// 检测过温异常
	if( !chk_temperature( pdev ) )
		return;
	
	// 读模块温度	
	temperature = ReadTemperature( pdev->basic.addr );
	if( ( temperature - pdev->basic.temp_before >= 10 ) || ( temperature - pdev->basic.temp_before <= -10 ) )
	{
	    pdev->basic.temp = pdev->basic.temp_before;
	}
	else
	{
	    pdev->basic.temp = temperature;
        pdev->basic.temp_before = temperature;
	}
	LOG_DEBUG_APP_1("\r\ntemperature:%d ", temperature);

	// 读风扇转速
	pdev->basic.fan_speed = ReadFanSpeed_1( pdev->basic.addr );
	LOG_DEBUG_APP_1("fan_speed_1:%d ", pdev->basic.fan_speed);
	
    switch( pdev->chgr.step )
	{
		case CHGR_STEP_CHECK_STAGE:
			LOG_DEBUG_APP_1("... check step ...");
			if( status & CCM )
			{
				LOG_DEBUG_APP_1("\r\nenter ccm");
				pdev->chgr.step = CHGR_STEP_CCM;
			}
			else if( status & CVM )
			{
				LOG_DEBUG_APP_1("\r\nenter cvm");
				 pdev->chgr.step = CHGR_STEP_CVM;
			}
			else if( status & FVM )
			{
				LOG_DEBUG_APP_1("\r\nenter fvm");
				pdev->chgr.step = CHGR_STEP_FVM;
				
				pdev->basic.bat = BASIC_BAT_FULL;
				pdev->basic.chgr = BASIC_CHGR_FINISH;
				pdev->chgr.step = CHGR_STEP_FVM;
			}
			else
			{
				LOG_DEBUG_APP_1("\r\nstart success, but status err(not in ccm,cvm,fvm), stop");
				stop( pdev );
				return;
			}
			break;
		case CHGR_STEP_CCM:
			LOG_DEBUG_APP_1("\r\nstep:ccm");
#if 1
			if( status & CVM )
			{
			    pdev->chgr.cnt_box[IDX_CNT_CVM] ++;
				LOG_DEBUG_APP_1("\r\nin step_ccm, cnt_cvm:%d", pdev->chgr.cnt_box[IDX_CNT_CVM]);
				if( pdev->chgr.cnt_box[IDX_CNT_CVM] >= FILTER_ENTER_CVM_3_SECTION )
				{
					ReadChgStatus( pdev->basic.addr, &status );
					LOG_DEBUG_APP_1("\r\nstatus:%04X ", status);
					if( status & CVM )
					{
						LOG_DEBUG_APP_1("\r\nenter cvm");
					    pdev->chgr.step = CHGR_STEP_CVM;
					}
					pdev->chgr.cnt_box[IDX_CNT_CVM] = 0;
				}
			}
			else
				pdev->chgr.cnt_box[IDX_CNT_CVM] = 0;	

			if( status & FVM )
			{
			    pdev->chgr.cnt_box[IDX_CNT_FVM] ++;
				LOG_DEBUG_APP_1("\r\nin step_ccm, cnt_fvm:%d", pdev->chgr.cnt_box[IDX_CNT_FVM]);
				if( pdev->chgr.cnt_box[IDX_CNT_FVM] >= FILTER_ENTER_FVM_3_SECTION )
				{
					ReadChgStatus( pdev->basic.addr, &status );
					LOG_DEBUG_APP_1("\r\nstatus:%04X ", status);
					if( status & FVM )
					{
						LOG_DEBUG_APP_1("\r\nenter fvm");
						pdev->basic.bat = BASIC_BAT_FULL;
					    pdev->basic.chgr = BASIC_CHGR_FINISH;
					    pdev->chgr.step = CHGR_STEP_FVM;
					}
					pdev->chgr.cnt_box[IDX_CNT_FVM] = 0;
				}
			}
			else
				pdev->chgr.cnt_box[IDX_CNT_FVM] = 0;
			
#endif
#if 0
			if( status & CVM )
			{
			    pdev->chgr.cnt_box[IDX_CNT_CVM] ++;
				LOG_DEBUG_APP_1("\r\nin step_ccm, cnt_cvm:%d", pdev->chgr.cnt_box[IDX_CNT_CVM]);
				if( pdev->chgr.cnt_box[IDX_CNT_CVM] >= FILTER_ENTER_CCM )
				{
					ReadChgStatus( pdev->basic.addr, &status );
					LOG_DEBUG_APP_1("\r\nstatus:%04X ", status);
					if( status & CVM )
					{
						LOG_DEBUG_APP_1("\r\nenter cvm");
					    pdev->chgr.step = CHGR_STEP_CVM;
					}
					pdev->chgr.cnt_box[IDX_CNT_CVM] = 0;
				}
			}
			else if( status & FVM )
			{
			    pdev->chgr.cnt_box[IDX_CNT_FVM] ++;
				LOG_DEBUG_APP_1("\r\nin step_ccm, cnt_fvm:%d", pdev->chgr.cnt_box[IDX_CNT_FVM]);
				if( pdev->chgr.cnt_box[IDX_CNT_FVM] >= FILTER_ENTER_CVM )
				{
					ReadChgStatus( pdev->basic.addr, &status );
					LOG_DEBUG_APP_1("\r\nstatus:%04X ", status);
					if( status & FVM )
					{
						LOG_DEBUG_APP_1("\r\nenter fvm");
						pdev->basic.bat = BASIC_BAT_FULL;
					    pdev->basic.chgr = BASIC_CHGR_FINISH;
					    pdev->chgr.step = CHGR_STEP_FVM;
					}
					pdev->chgr.cnt_box[IDX_CNT_FVM] = 0;
				}
			}
#endif
			break;
		case CHGR_STEP_CVM:
			LOG_DEBUG_APP_1("\r\nstep:cvm");
#if 1
			if( status & FVM )
			{
			    pdev->chgr.cnt_box[IDX_CNT_FVM] ++;
				LOG_DEBUG_APP_1("\r\nin step_cvm, cnt_fvm:%d", pdev->chgr.cnt_box[IDX_CNT_FVM]);
				if( pdev->chgr.cnt_box[IDX_CNT_FVM] >= FILTER_ENTER_FVM_3_SECTION )
				{
					ReadChgStatus( pdev->basic.addr, &status );
					LOG_DEBUG_APP_1("\r\nstatus:%04X ", status);
					if( status & FVM )
					{
						LOG_DEBUG_APP_1("\r\nenter fvm");
					    pdev->basic.bat = BASIC_BAT_FULL;
					    pdev->basic.chgr = BASIC_CHGR_FINISH;
					    pdev->chgr.step = CHGR_STEP_FVM;
					}
					pdev->chgr.cnt_box[IDX_CNT_FVM] = 0;
				}
			}
			else
				pdev->chgr.cnt_box[IDX_CNT_FVM] = 0;
#endif
#if 0
			if( status & FVM )
			{
			    pdev->chgr.cnt_box[IDX_CNT_FVM] ++;
				LOG_DEBUG_APP_1("\r\nin step_cvm, cnt_fvm:%d", pdev->chgr.cnt_box[IDX_CNT_FVM]);
				if( pdev->chgr.cnt_box[IDX_CNT_FVM] >= FILTER_ENTER_FVM )
				{
					ReadChgStatus( pdev->basic.addr, &status );
					LOG_DEBUG_APP_1("\r\nstatus:%04X ", status);
					if( status & FVM )
					{
						LOG_DEBUG_APP_1("\r\nenter fvm");
					    pdev->basic.bat = BASIC_BAT_FULL;
					    pdev->basic.chgr = BASIC_CHGR_FINISH;
					    pdev->chgr.step = CHGR_STEP_FVM;
					}
					pdev->chgr.cnt_box[IDX_CNT_FVM] = 0;
				}
			}
#endif
			break;
		case CHGR_STEP_FVM:
			LOG_DEBUG_APP_1("\r\nstep:fvm");
			pdev->chgr.flag_full = TRUE;

			LOG_DEBUG_APP("num:%d 已经充满，关闭输出", pdev->basic.num);

			// 更新电池缓存信息中，设置电池充满标志
			local_bat_info = get_local_bat_info();
			set_bit( local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_FULL );

			// 进入浮充后，关闭模块输出
            close_output( pdev->basic.num );

			// 一直检测电池有没有断开
			pdev->chgr.step = CHGR_STEP_CHK_BAT_DISCONN_IN_FLOATING;
			break;
		case CHGR_STEP_IDLE:
			break;
		case CHGR_STEP_CHK_BAT_DISCONN_IN_FLOATING:
			LOG_DEBUG_APP_1("\r\nstep:chk_bat_dis_in_floating");
			// 关闭输出后，检测电池是否连接
			// 如果电池没有连接，停止充电
			if( !isBatConn( volt ) )
			{
			    LOG_DEBUG_APP("\r\n浮充中检测电池连接状态，电池确实断开");
				stop( pdev );
			}
			ReadVout( pdev->basic.addr, &vout );

			ReadIout( pdev->basic.addr, &iout );
			LOG_DEBUG_APP_1("addr:%02X read vout:%f read iout:%f ", pdev->basic.addr, vout, iout);
			ReadChgStatus( pdev->basic.addr, &status );
			pdev->chgr.status = status;

			LOG_DEBUG_APP_1("status:%04X ", status);
			break;
		default:
			break;
	}
	
//	vTaskDelay( pdMS_TO_TICKS( 10 ) );
}

static void chgr_manual_with_pre( PU_InfoDef *pdev )
{
	float iout = 0;
	float vout = 0;
	float set_vout = 0;
	float limit_iout = 0;
	int8_t temperature = 0;
	float limit_read = 0;
	float volt = 0;
	__IO float ave_iout;
	uint16_t status;
	uint8_t *local_bat_info;
    StatusTypeDef st_pmbus;

	if( pdev->basic.chgr == BASIC_CHGR_STARTING )
	{
		if( !chgr_starting( pdev ) )
			return;
	}

	// 读取电池电压
	volt = pdev->basic.bat_volt;
	LOG_DEBUG_APP_1("\r\nvolt:%f", volt);

	// 处理充电过程中，已经检测到输出电流为0的情况
	if( pdev->chgr.step == CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING )
	{
		chk_bat_conn_in_charging( pdev );
#if 0
		pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] ++;
		if( !isBatConn( volt ) )
		{
            pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;
			LOG_DEBUG_APP("\r\n充电中检测电池断开阶段，电池确实断开");
			stop( pdev );
		}
		else if( pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] >= chgrMS_TO_CNT(4000) )
		{
            pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;

			if( volt >= (float)THRESHOLD_VOLT_IN_IOUT_DIS )
			{
				// 设置充电完成
				pdev->basic.chgr = BASIC_CHGR_FINISH;
				pdev->basic.bat = BASIC_BAT_FULL;

				// 更新电池缓存信息中，设置电池充满标志
				local_bat_info = get_local_bat_info();
				set_bit( local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_FULL );
				// 关闭后，设置输出电压和输出电流为0
				pdev->chgr.vout = 0;
				pdev->chgr.iout = 0;

				pdev->chgr.step = CHGR_STEP_FVM;

				LOG_DEBUG_APP("\r\n充电中检测电池断开阶段，电池没有断开，认为充满，进入浮充状态");
			}
			else
			{
				LOG_DEBUG_APP("\r\nvolt:%f < %f", volt, (float)THRESHOLD_VOLT_IN_IOUT_DIS );
				LOG_DEBUG_APP_1(" 非法停止");
				stop( pdev );
				cache_event( pdev, EVENT_ILLEGAL_STOP );
			}
		}
#endif

		return;
	}

	// 读模块温度	
	temperature = ReadTemperature( pdev->basic.addr );
	if( ( temperature - pdev->basic.temp_before >= 10 ) || ( temperature - pdev->basic.temp_before <= -10 ) )
	{
	    pdev->basic.temp = pdev->basic.temp_before;
	}
	else
	{
	    pdev->basic.temp = temperature;
        pdev->basic.temp_before = temperature;
	}

	// 读风扇转速
	pdev->basic.fan_speed = ReadFanSpeed_1( pdev->basic.addr );

    // 更新模块状态
	ReadChgStatus( pdev->basic.addr, &status );
	pdev->chgr.status = status;

	// 检测过温异常
	if( !chk_temperature( pdev ) )
		return;

/**********************************************************/
	// 进入浮充后，停止读取
	if( pdev->chgr.step == CHGR_STEP_FVM )
	{
		// 如果电池没有连接，停止充电
		if( !isBatConn( volt ) )
		{
		    LOG_DEBUG_APP_1("\r\n浮充中检测电池连接状态，电池断开");
			stop( pdev );
			return;
		}

		return; // ！！！！！！
	}

	// 检测输出异常
	if( !chk_dc_output( pdev ) )
		return;

	// 读取充电电压
	ReadVout_St( pdev->basic.addr, &vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
	LOG_DEBUG_APP_1("\r\nread_vout:%f ", vout );
    disp_pmbus_comm_status( st_pmbus );
	if( ( vout < 0 ) || ( vout - pdev->chgr.data_max.vbst > 1 ) )
	{
		LOG_DEBUG_APP_1("\r\nvour err, return" );
		return;
	}

	// 读取充电电流
    ReadIout_St( pdev->basic.addr, &iout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
	LOG_DEBUG_APP_1("\r\nread_iout:%f ", iout );
    disp_pmbus_comm_status( st_pmbus );

	if( st_pmbus != SMBUS_OK )
	{
		LOG_DEBUG_APP_1("\r\nsmbus err, use pre iout->%f", pdev->chgr.iout_pre);
		iout = pdev->chgr.iout_pre; //  使用上次的电流
	}
	
    if( (iout - 28.5 > 0 ) || ( iout < 0 ) ) 
    { 
	    LOG_DEBUG_APP_1("\r\nerror data, use pre iout->%f", pdev->chgr.iout_pre); 
		iout = pdev->chgr.iout_pre; //  使用上次的电流
	}
	pdev->chgr.iout_pre = iout;

	// 写入iout queue，并计算平均值
    write_iout_to_queue( pdev, iout );
    calc_iout_ave( pdev );
    ave_iout = pdev->chgr.iout;
	LOG_DEBUG_APP_1("\r\nave_iout:%f", ave_iout);

	// 如果充电过程中，输出电流为0，进入检测电池是否断开的步骤
    if( is_iout_disappear_in_charge( pdev, iout ) )
	{
		LOG_DEBUG_APP_1("\r\n模块%d输出电流为0，关闭输出，进入下一步", pdev->basic.num);

        close_output( pdev->basic.num );
		pdev->chgr.step = CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING;
		return;
	}

	switch( pdev->chgr.step )
	{
		case CHGR_STEP_CHECK_STAGE:
			LOG_DEBUG_APP_1("... check step ...");
			// 小于预充电压，需要开启预充
			if( volt < pdev->chgr.para.data_pre.pre_vout )
			{
				pdev->chgr.cnt_box[ IDX_CNT_IS_PRE_CHGR ] ++;
			}
			else
				pdev->chgr.cnt_box[ IDX_CNT_IS_PRE_CHGR ] --;

			// 连续3次电压小于预充电压，进入预充步骤
			if( pdev->chgr.cnt_box[ IDX_CNT_IS_PRE_CHGR ] >= 3 )
			{
				pdev->chgr.step = CHGR_STEP_PRE;
				pdev->basic.chgr = BASIC_CHGR_ING;

				// 清零电池断开检测，防止误判
				pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;

				LOG_DEBUG_APP_1("\r\nnum:%d 电池电压:%f，小于预充电压:%f，进入预充阶段", pdev->basic.num, volt, pdev->chgr.para.data_pre.pre_vout);

			}
			else if ( pdev->chgr.cnt_box[ IDX_CNT_IS_PRE_CHGR ] <= ( 3 * (-1) ) )
			{
				pdev->chgr.step = CHGR_STEP_CCM;
				pdev->basic.chgr = BASIC_CHGR_ING;

				// 清零电池断开检测，防止误判
				pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;

				LOG_DEBUG_APP_1("\r\nnum:%d 电池电压:%f，大于预充电压:%f，进入恒流阶段", pdev->basic.num, volt, pdev->chgr.para.data_pre.pre_vout);

			}
			break;
        case CHGR_STEP_PRE:
			if( volt > pdev->chgr.para.data_pre.pre_vout )
			{
				pdev->chgr.cnt_box[ IDX_CNT_CCM ] ++;
			}
			else
			{
				pdev->chgr.cnt_box[ IDX_CNT_CCM ] = 0;
			}

			if( pdev->chgr.cnt_box[ IDX_CNT_CCM ] >= FILTER_ENTER_CCM )
			{
				pdev->chgr.cnt_box[ IDX_CNT_CCM ] = 0;
			    pdev->chgr.step = CHGR_STEP_CCM;
		        LOG_DEBUG_APP_1("num:%d 电池电压:%f，大于恒充电压:%f，进入恒流阶段", pdev->basic.num, volt, pdev->chgr.para.data_pre.pre_vout);

				// 清零电池断开检测，防止误判
				pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;

			    // 更新限制电流
                pdev->chgr.limit_iout = pdev->chgr.para.data_pre.pre_iout;

				return;
			}

			// 更新限制电流
            pdev->chgr.limit_iout = pdev->chgr.para.data_pre.pre_iout;
		    break;
        case CHGR_STEP_CCM:
			if( volt >= ( pdev->chgr.para.data_pre.vbst - 0.5 ) )
			{
				pdev->chgr.cnt_box[ IDX_CNT_CVM ] ++;
			}
			else
			{
				pdev->chgr.cnt_box[ IDX_CNT_CVM ] = 0;
			}

			if( pdev->chgr.cnt_box[ IDX_CNT_CVM ] >= FILTER_ENTER_CVM )
			{
                pdev->chgr.cnt_box[ IDX_CNT_CVM ] = 0;
			    pdev->chgr.step = CHGR_STEP_CVM;
		        LOG_DEBUG_APP_1("num:%d 电池电压:%f，大于等于恒充电压:%f - 0.5，进入恒压阶段", pdev->basic.num, volt, pdev->chgr.para.data_pre.vbst);
				
				// 清零电池断开检测，防止误判
				pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;

			    // 更新限制电流
                pdev->chgr.limit_iout = pdev->chgr.para.data_pre.ichg;

				return;
			}

			// 更新限制电流
            pdev->chgr.limit_iout = pdev->chgr.para.data_pre.ichg;
		    break;
        case CHGR_STEP_CVM:
			if( ( iout < pdev->chgr.para.data_pre.itaper ) && ( pdev->chgr.para.data_pre.vbst - volt ) <= 0.2 )
			{
                pdev->chgr.cnt_box[ IDX_CNT_FVM ] ++;
			}
			else
			{
                pdev->chgr.cnt_box[ IDX_CNT_FVM ] = 0;
			}

			if( pdev->chgr.cnt_box[ IDX_CNT_FVM ] >= FILTER_ENTER_FVM )
			{
                pdev->chgr.cnt_box[ IDX_CNT_FVM ] = 0;
			    pdev->chgr.step = CHGR_STEP_FVM;

		        LOG_DEBUG_APP_1("num:%d 充电电流:%f，小于终止电流:%f，进入浮充阶段", pdev->basic.num, iout, pdev->chgr.para.data_pre.itaper);
				// 表示充满
		        LOG_DEBUG_APP_1("\r\nnum:%d 已经充满，关闭输出", pdev->basic.num);

			    // 更新电池缓存信息中，设置电池充满标志
			    local_bat_info = get_local_bat_info();
			    set_bit( local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_FULL );

		        // 充满后，关闭输出
		        close_output( pdev->basic.num );
			    
			    // 设置充电完成
			    pdev->basic.chgr = BASIC_CHGR_FINISH;
				pdev->basic.bat = BASIC_BAT_FULL;
			    
			    // 关闭后，设置输出电压和输出电流为0
			    pdev->chgr.vout = 0;
			    pdev->chgr.iout = 0;

				// 清零电池断开检测，防止误判
				pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;

			    // 更新限制电流
                pdev->chgr.limit_iout = pdev->chgr.para.data_pre.ichg;
				
				return;
			}

			// 更新限制电流
            pdev->chgr.limit_iout = pdev->chgr.para.data_pre.ichg;
		    break;
		default:
		break;
	}

/************************* 控制部分 ****************************/
	LOG_DEBUG_APP_1("\r\n<-----iout control----->");
	limit_iout = pdev->chgr.limit_iout;

	if( limit_iout - 5.5 >= 0 )  // ioc >= 5.5，使用B46指令设置限制电流即可
	{
		LOG_DEBUG_APP_1("\r\nlimit >= 5.5, use B46");
	    // 写入限压值
        set_vout = pdev->chgr.para.data_pre.vbst;
        SetVout_St( pdev->basic.addr, set_vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
	    LOG_DEBUG_APP_1("\r\nset_vout:%f ", set_vout );
        disp_pmbus_comm_status( st_pmbus );

	    // 写入限流值
	    LOG_DEBUG_APP_1("\r\nset limit_iout:%f ", limit_iout);
        SetIoutOC_FaultLimit_St( pdev->basic.addr, limit_iout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
        disp_pmbus_comm_status( st_pmbus );
        ReadIoutOC_FaultLimit_St( pdev->basic.addr, &limit_read, &st_pmbus );
	    LOG_DEBUG_APP_1("  read limit_iout:%f ", limit_read);

		// 更新电压基准值，处理当电流从>5.5慢慢降到<5.5时，由于基准值过低导致输出电流为0的问题
   	    pdev->chgr.volt_base = volt;
		pdev->chgr.auto_vout_step = 0;
	}
	else
	{
		LOG_DEBUG_APP_1("\r\nlimit < 5.5, use mcu control");
//        LOG_DEBUG_APP_1("\r\niout: 1:%f, 2:%f, 3:%f, 4:%f", pdev->chgr.iout_queue.buff[0], pdev->chgr.iout_queue.buff[1], pdev->chgr.iout_queue.buff[2], pdev->chgr.iout_queue.buff[3]);
		chgr_control_little_iout( pdev, vout, iout, limit_iout );
	}

	// 更新输出电压、电流，过滤异常电压，电流
	if( ( vout <= pdev->chgr.data_max.vbst + 1 ) && ( vout >= 0 ) )
		pdev->chgr.vout = vout;
	if( ( pdev->chgr.step == CHGR_STEP_PRE ) || ( ( pdev->chgr.step == CHGR_STEP_CCM ) ) )
	{
	    pdev->chgr.iout_ori = iout;
		pdev->chgr.iout_disp = limit_iout;
	}
	else 
	{
		if( ( iout <= pdev->chgr.data_max.ichg ) && ( iout >= 0 ) )
		{
			pdev->chgr.iout_ori = iout;
			// 界面显示的电流
			// iout在limit_iout的[-0.5, +0.5]，范围内，均显示limit_iout
			if( ( ( iout <= limit_iout ) && ( ( iout + 0.5 ) >= limit_iout ) ) || \
				( ( iout >= limit_iout ) && ( ( limit_iout + 0.5 ) <= iout ) )
			  )
			{
				pdev->chgr.iout_disp = limit_iout;
			}
			else
			{
				pdev->chgr.iout_disp = iout;
			}
		}
	}

	if( pdev->chgr.iout_disp == 0 )
		LOG_DEBUG_APP_1("\r\n** for test ****************************************************itou_disp == 0*******************");

	LOG_DEBUG_APP_1("\r\niout_disp:%f, chgr_time_cnt:%d, chgr_time:%d\r\n\r\n\r\n", pdev->chgr.iout_disp, pdev->chgr.cnt_box[ IDX_CNT_CHGR_TIME ], pdev->chgr.time );

}

static void chgr_control_little_iout( PU_InfoDef *pdev, float vout, float iout, float limit_iout )
{
	float set_vout = 0;
	float max_set_vout = 0;
	StatusTypeDef st_pmbus;

	// 防止set_vout为0
	set_vout = pdev->basic.bat_volt;

	/****************************   控制过程    *******************************/
	LOG_DEBUG_APP_1("\r\nnum:%d iout limit:%f", pdev->basic.num, limit_iout);

	/* 20191015，针对自动充电模式，充满误判的问题，采用如下方案:
	 * 如果需求电流小于2A，按照2A处理
	 *
	 * 实际上，通过调电压控制输出电流的方式，最小输出电流只能是2A，所以这里不管是手动模式还是自动模式，均按照最小2A处理
	 */
	if( limit_iout < (float)2 )
	{
		LOG_DEBUG_APP_1("\r\nlimit_iout < 2A, set limit_iout = 2A");
		limit_iout = (float)2;
	}

	// 恒流
	// 计算电流增量，误差为0.5
	if( ( limit_iout - iout ) > 0 )
	{
			
		if( ( limit_iout - iout ) >= 1.5 )
		{
			pdev->chgr.auto_vout_step += 0.02;
		}
		else
		{
			pdev->chgr.auto_vout_step += 0.008;
		}
	}
	else if( ( limit_iout - iout ) < -0.5 )
	{
			
		if( (limit_iout - iout) <= (-1.5) )
		{
			pdev->chgr.auto_vout_step -= 0.02;
		}
		else 
		{
			pdev->chgr.auto_vout_step -= 0.008;
		}
	}

	set_vout = pdev->chgr.volt_base + pdev->chgr.auto_vout_step;
	// 判断限制压值的范围
    if( pdev->chgr.mode == MODE_AUTO  ) {
		max_set_vout = (float)pdev->bat_info_bms.full_volt/10;
	}else if( pdev->chgr.type == CHGR_TYPE_PRE ) {
		max_set_vout = pdev->chgr.para.data_pre.vbst;
	}

	if( set_vout >= max_set_vout ) {
		LOG_DEBUG_APP_1("\r\nmax_set_vout = %.1f", max_set_vout);
		set_vout = max_set_vout;
	    pdev->chgr.auto_vout_step = set_vout - pdev->chgr.volt_base;
	}

	// 写入设置的电压值
	LOG_DEBUG_APP_1("\r\nset vout:%f, vbst:%f, volt_base:%f, vout_add_step:%f ", set_vout, pdev->chgr.para.data_pre.vbst, pdev->chgr.volt_base, pdev->chgr.auto_vout_step);
	SetVout_St( pdev->basic.addr, set_vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
	disp_pmbus_comm_status( st_pmbus );
}

static void chgr_control_little_iout_in_starting( PU_InfoDef *pdev, float vout, float iout, float limit_iout )
{
	float set_vout = 0;
	StatusTypeDef st_pmbus;
	float max_set_vout = 0;

	// 防止set_vout为0
	set_vout = pdev->basic.bat_volt;

	/****************************   控制过程    *******************************/
	LOG_DEBUG_APP_1("\r\nnum:%d iout limit:%f", pdev->basic.num, limit_iout);

	// 恒流
	// 计算电流增量，误差为0.5
	if( ( limit_iout - iout ) > 0 )
	{
			
		if( ( limit_iout - iout ) >= 1.5 )
		{
			pdev->chgr.auto_vout_step += 0.1;
		}
		else
		{
			pdev->chgr.auto_vout_step += 0.05;
		}
	}
	else if( ( limit_iout - iout ) < -0.5 )
	{
			
		if( (limit_iout - iout) <= (-1.5) )
		{
			pdev->chgr.auto_vout_step -= 0.1;
		}
		else 
		{
			pdev->chgr.auto_vout_step -= 0.05;
		}
	}

	set_vout = pdev->chgr.volt_base + pdev->chgr.auto_vout_step;

	// 判断限制压值的范围
    if( pdev->chgr.mode == MODE_AUTO  ) {
		max_set_vout = (float)pdev->bat_info_bms.full_volt/10;
	}else if( pdev->chgr.type == CHGR_TYPE_PRE ) {
		max_set_vout = pdev->chgr.para.data_pre.vbst;
	}

	if( set_vout >= max_set_vout ) {
		LOG_DEBUG_APP_1("\r\nmax_set_vout = %.1f", max_set_vout);
		set_vout = max_set_vout;
	    pdev->chgr.auto_vout_step = set_vout - pdev->chgr.volt_base;
	}

	// 写入设置的电压值
	LOG_DEBUG_APP_1("\r\nset vout:%f, vbst:%f, volt_base:%f, vout_add_step:%f ", set_vout, pdev->chgr.para.data_pre.vbst, pdev->chgr.volt_base, pdev->chgr.auto_vout_step);
	SetVout_St( pdev->basic.addr, set_vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // 发布release版本，屏蔽log信息后，pmbus通信需要添加延时
#endif
	disp_pmbus_comm_status( st_pmbus );
}

static void chk_bat_conn_in_charging( PU_InfoDef *pdev )
{
	float volt = pdev->basic.bat_volt;
	uint8_t *local_bat_info;
	
	LOG_DEBUG_APP_1("\r\nvolt:%f, chk bat conn in charging...", volt);
	pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] ++;
	if( !isBatConn( volt ) )
	{
		pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;
		LOG_DEBUG_APP_1("\r\n检测电池断开阶段，电池确实断开");
		stop( pdev );
	}
	else if( pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] >= chgrMS_TO_CNT(4000) )
	{
		pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;

		// 如果电池没有断开，判断电池电压，如果电池电压>=指定电压值（THRESHOLD_VOLT_IN_IOUT_DIS），则认为充满，否则停止，报非法停止事件 
		if( volt >= (float)THRESHOLD_VOLT_IN_IOUT_DIS )
		{
			LOG_DEBUG_APP("\r\nvolt:%f", volt);
			
			// 设置充电完成
			pdev->basic.chgr = BASIC_CHGR_FINISH;
			pdev->basic.bat = BASIC_BAT_FULL;

			// 更新电池缓存信息中，设置电池充满标志
//			local_bat_info = get_local_bat_info();
//			set_bit( local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_FULL );
			// 关闭后，设置输出电压和输出电流为0
			pdev->chgr.vout = 0;
			pdev->chgr.iout = 0;

			pdev->chgr.step = CHGR_STEP_FVM;
			
			if( pdev->chgr.mode == MODE_AUTO )
			{
				// mcu检测电池是否充满
				pdev->chgr.auto_flag_full_mcu = TRUE;
				// 清零电池断开检测，防止误判
				pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;
			}

			LOG_DEBUG_APP_1("\r\n检测电池断开阶段，电池没有断开，认为充满，进入浮充状态");
		}
		else
		{
			LOG_DEBUG_APP_1("\r\nvolt:%f < %f，非法停止", volt, (float)THRESHOLD_VOLT_IN_IOUT_DIS );
			stop( pdev );
			cache_event( pdev, EVENT_ILLEGAL_STOP );
			
			// 设置警告标志
			set_bit( pdev->basic.warn, WARN_ILLEGAL_STOP );
		}
	}
}

// pi control
#if 0
/*
 * wRTrackVolt - limit_iout
 * wRKVoltrack1, wRKVoltrack2, - pi参数
 * wRInverterVoltNew , 读取的反馈电流值
 * wRNewSinAmp，电压调节基准值
 * wRVoltError0， 上一次的偏差
 * 0.2, 0.1
 */

void sRVoltRegu(float iout_limit, float wRKVoltrack1, float wRKVoltrack2, float *err_before, float input, float *output, float volt ) 
{ 
   float        wRVoltError; 
   float        temp; 

   float wRTrackVolt = iout_limit;
   float wRVoltError0 = *err_before;
   float wRInverterVoltNew = input;
   float wRNewSinAmp = *output;
   float wRNewSinAmpTemp;
    
   wRNewSinAmpTemp=wRNewSinAmp; 
    
   wRVoltError=wRTrackVolt-wRInverterVoltNew; 
	
   if( ( ( wRVoltError > 0 ) && ( wRVoltError < 0.3 ) ) || ( ( wRVoltError < 0 ) && ( wRVoltError > -0.3 ) ) )   
	   wRVoltError = 0;
//	if( ((wRVoltError - wRVoltError0)> 0.1) || ((wRVoltError - wRVoltError0)< -0.1))
//	{
//		//wRVoltError = wRVoltError0;
//		return;
//	}
   if(wRVoltError>=0.2)          
   { 
      wRVoltError = 0.2;       
   } 
   else if(wRVoltError<=-0.2)    
   { 
      wRVoltError = -0.2;    
   }
   

   temp=wRKVoltrack1*wRVoltError-wRKVoltrack2*wRVoltError0; 
    
   wRNewSinAmpTemp=wRNewSinAmpTemp+temp; 
    
//   wRVoltError0=wRVoltError; 
    
   wRNewSinAmp=wRNewSinAmpTemp; 

//   if(wRNewSinAmp > cNewSinAmpMax)   
   if(wRNewSinAmp > 55)   
   { 
      wRNewSinAmp = 55; 
   } 
   
//   if(wRNewSinAmp >(  volt + 0.1 ) )   
//   { 
//      wRNewSinAmp = volt + 0.1; 
//   }
//   else if( wRNewSinAmp <= 36  )
//   {
//       wRNewSinAmp = 36; 
//   }

//   if( wRNewSinAmp > 57 )
//   {
//       wRNewSinAmp = 57;
//   }	   

   *output = wRNewSinAmp;
   *err_before = wRVoltError;
}

static void chgr_control_little_iout( PU_InfoDef *pdev, float vout, float iout, float limit_iout )
{

    uint8_t retry_time= 0;
	uint16_t status = 0;
	float set_vout = 0;
	float temp = 0;

	float smp = 0;
	float volt = 0;

	float iout_deviation_before = 0;

    uint8_t *local_bat_info = NULL;

	StatusTypeDef st_pmbus;

	/****************************   控制过程    *******************************/
	LOG_DEBUG_APP_1("\r\nnum:%d iout limit:%f", pdev->basic.num, limit_iout);
	LOG_DEBUG_APP_1("\r\nset_vout_before:%f, iout_err_before:%f", pdev->chgr.set_vout, pdev->chgr.iout_deviation_before);

    set_vout = pdev->chgr.set_vout;
    iout_deviation_before = pdev->chgr.iout_deviation_before;
	
	volt = pdev->basic.bat_volt;

    sRVoltRegu( limit_iout, 0.06, 0.05, &iout_deviation_before, iout, &set_vout, volt );

	pdev->chgr.set_vout = set_vout;
    pdev->chgr.iout_deviation_before = iout_deviation_before;

	LOG_DEBUG_APP_1("\r\nset_vout_current:%f, iout_err_current:%f", pdev->chgr.set_vout, pdev->chgr.iout_deviation_before);

	// 写入设置的电压值
	SetVout_St( pdev->basic.addr, set_vout, &st_pmbus );
//	vTaskDelay( pdMS_TO_TICKS(20) );
	disp_pmbus_comm_status( st_pmbus );
}
#endif
static BOOL is_vout_dispear_in_auto_mode( PU_InfoDef *pdev )
{
	float volt = 0;
	uint8_t *local_bat_info = NULL;
	
	local_bat_info = get_local_bat_info();
	
	volt = pdev->basic.bat_volt;
	
	if( !isBatConn( volt ) )
	{
	    pdev->chgr.cnt_box[IDX_CNT_CHK_VOUT_DISAPPEAR_IN_CHGRING] ++;
	}
	else
	{
	    pdev->chgr.cnt_box[IDX_CNT_CHK_VOUT_DISAPPEAR_IN_CHGRING] = 0;
	}
	
	if( pdev->chgr.cnt_box[IDX_CNT_CHK_VOUT_DISAPPEAR_IN_CHGRING] >= FILTER_VOUT_DISPEAR )
	{
		// 更新本地电池信息缓存中，电池未连接信息
	    reset_bit(local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_CONN);

		// 设置电池未连接
		pdev->basic.bat = BASIC_BAT_DISCONN;
		
	    pdev->chgr.cnt_box[IDX_CNT_CHK_VOUT_DISAPPEAR_IN_CHGRING] = 0;
		return TRUE;
	}
		
	return FALSE;
}

void chk_chgr_step_in_3_section( PU_InfoDef *pdev, uint16_t status)
{
	if( status & CCM )
	{
		pdev->chgr.cnt_box[IDX_CNT_CCM] ++;
		LOG_DEBUG_APP_1("\r\nin step_ccm, cnt_ccm:%d", pdev->chgr.cnt_box[IDX_CNT_CCM]);
		if( pdev->chgr.cnt_box[IDX_CNT_CCM] >= FILTER_ENTER_CCM )
		{
			LOG_DEBUG_APP_1("\r\nenter ccm");
			pdev->chgr.step = CHGR_STEP_CCM;
		}
	}
	else
		pdev->chgr.cnt_box[IDX_CNT_FVM] = 0;
}

static void write_iout_to_queue ( PU_InfoDef *pdev, float iout )
{
	pdev->chgr.iout_queue.buff[ pdev->chgr.iout_queue.head++ ] = iout; 
	if( pdev->chgr.iout_queue.head >= IOUT_AVE_QUEUE_SIZE  )
	{
		pdev->chgr.iout_queue.full_flag = TRUE;
		pdev->chgr.iout_queue.head = 0;
	}
}

static void calc_iout_ave( PU_InfoDef *pdev )
{
	uint16_t i = 0, size = 0;
	float iout_sum = 0;

    if( pdev->chgr.iout_queue.full_flag != TRUE )
	{
		size = pdev->chgr.iout_queue.head;
	}
	else
	{
		size = IOUT_AVE_QUEUE_SIZE;
	}

    for( i = 0; i < size; i ++ )
	{
	    iout_sum += pdev->chgr.iout_queue.buff[ i ];
	    if( i >= IOUT_AVE_QUEUE_SIZE )
			i = 0;
	}

	pdev->chgr.iout = iout_sum / size;
}

static BOOL chk_dc_output( PU_InfoDef *pdev )
{
	// 检测输出异常 ，打开输出阶段和检测输出阶段，不检测
	switch ( pdev->chgr.step )
	{
		case CHGR_STEP_OPEN_OUTPUT:
			return TRUE;
//			break;
		case CHGR_STEP_CHECK_OUTPUT:
			return TRUE;
//			break;
		case CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING:
			return TRUE;
//			break;
		case CHGR_STEP_FVM:
			return TRUE;
//			break;
		case CHGR_STEP_CHK_BAT_DISCONN_IN_FLOATING:
			return TRUE;
//			break;

		default:
			break;
	}

	if( isDcOutPutErr( pdev ) )
	{
		LOG_DEBUG_APP_1("\r\nnum:%d输出异常", pdev->basic.num);

		if( ( pdev->chgr.op != op_start ) && ( pdev->chgr.step == CHGR_STEP_OPEN_OUTPUT ) ) // 如果还没有启动成功
		{
			start_failed( pdev );
		}
		else
		{
			stop( pdev );
		}

		// 设置错误标志
		set_bit( pdev->basic.err, ERR_DC_OK );

		// 添加输出异常事件
		cache_event( pdev, EVENT_DC_OK_ERR );

		return FALSE;
	}
	else
	{
		// 清除错误标志
		reset_bit( pdev->basic.err, ERR_DC_OK );
	}

	return TRUE;
}

static BOOL chk_temperature( PU_InfoDef *pdev )
{
	// 检测过温, 如果过温，直接stop掉
	if( is_t_alarm_in_charge( pdev ) )
	{

	    if( ( pdev->chgr.op != op_start ) && ( pdev->chgr.step == CHGR_STEP_OPEN_OUTPUT ) ) // 如果还没有启动成功
            start_failed( pdev );
		else
		    stop( pdev );

		// 设置错误标志
		set_bit( pdev->basic.err, ERR_T_ALARM );

		cache_event( pdev, EVENT_T_ALARM );
		return FALSE;
	}
	else
	{
		// 清除错误标志
		reset_bit( pdev->basic.err, ERR_T_ALARM );
	}

	return TRUE;
}

static void open_output( uint8_t num )
{
	// 打开模块的AC输入
	power_on( num );

    // 打开dc输出继电器
    open_dc_relay( num );

	// 打开模块输出
//	on_off_control( num, ON );
}

static void close_output( uint8_t num )
{
	// 关闭on/off输出
	on_off_control( num, OFF );

	// 再关闭dc输出继电器
    close_dc_relay( num );
}

static BOOL is_iout_disappear_in_charge( PU_InfoDef *pdev, float iout )
{
	// 如果处在OpenOutPut的状态，不检测
	if( pdev->chgr.step == CHGR_STEP_OPEN_OUTPUT ) return FALSE;

	// 如果处在CHGR_STEP_OPEN_OUTPUT的状态，不检测
	if( pdev->chgr.step == CHGR_STEP_CHECK_OUTPUT ) return FALSE;
	
	// 如果处于浮充状态，不检测
	if( pdev->chgr.step == CHGR_STEP_FVM ) return FALSE;

	// 如果处于检测电池是否断开的步骤，不检测
	if( pdev->chgr.step == CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING ) return FALSE;

	// 如果已经充满了，不检测
	if( pdev->chgr.step == CHGR_STEP_CHK_BAT_DISCONN_IN_FLOATING ) return FALSE;
	
	// chk
	if ( (iout - 0) <= 0 )
	{
	    pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] ++;
	}
	else if ( iout <= (pdev->chgr.data_max.ichg+2) )
	{
	    pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;
	}
	else // 读取数据有误，说明通信有问题，按照电流为0处理
		pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] ++;
	
	// process result
	/*
	 * 连续10次检测到输出电流为0之后，采用如下操作检测电池是否真的断开
	 * 1. 断开模块输出(on/off)
	 * 2. 延时5s
	 * 3. 使用AD采样电池电压，如果采样到电池未连接，则停止充电，如果采样到连接，那么打开模块输出(on/off)
	 *
	 */
	if( pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] >= FILTER_IOUT_DIS_IN_CHARGING )
	{
		pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;
	    return TRUE;
	}
	
	return FALSE;	
}

static BOOL is_t_alarm_in_charge( PU_InfoDef *pdev )
{
	uint8_t num = 0;
	uint16_t pin_tab[] = 
	{
		T_ALARM1_GPIO_PIN,
		T_ALARM2_GPIO_PIN,
		T_ALARM3_GPIO_PIN,
		T_ALARM4_GPIO_PIN,
		T_ALARM5_GPIO_PIN,
		T_ALARM6_GPIO_PIN,
		T_ALARM7_GPIO_PIN,
		T_ALARM8_GPIO_PIN	
	};
	
	// 如果处在OpenOutPut的状态，不检测
	if( pdev->chgr.step == CHGR_STEP_OPEN_OUTPUT ) return FALSE;
	
    num = pdev->basic.num;
	// chk, high -> err
	if ( GPIO_READ( T_ALARM1_GPIO_PORT, pin_tab[ num ]) != LOW )
	{
	    pdev->chgr.cnt_box[ IDX_CNT_CHK_T_ALARM ] ++;
	}
	else if ( GPIO_READ( T_ALARM1_GPIO_PORT, pin_tab[ num ]) == LOW )
	{
	    pdev->chgr.cnt_box[ IDX_CNT_CHK_T_ALARM ] = 0;
	}
	
	// process result
	if( pdev->chgr.cnt_box[ IDX_CNT_CHK_T_ALARM ] > FILTER_T_ALARM )
	{
		pdev->chgr.cnt_box[ IDX_CNT_CHK_T_ALARM ] = 0;
	    return TRUE;
	}
	
	return FALSE;
}

static BOOL chk_pmbus_comm( uint32_t *pcnt, uint8_t addr )
{
    if( ChkAddrDev( addr ) )
	{
		*pcnt = 0;
		return TRUE;
	}
	else
	{
	    *pcnt += 1;
	    return FALSE;
	}
}

static void chgr_emergency_stop(void)
{
    ChgrListDef *pnode = NULL;

	// 先显示急停画面
    xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_EMERGENCY_STOP, eSetBits );

	// 挂起充电任务
	suspend_charge_damon();

	// 关闭所有正在充电的模块的输出
	pnode = pchgr_list->next;
	while( pnode != NULL )
	{
        close_output( pnode->pdev->basic.num );	
	    pnode = pnode->next;
	}
}

static void chgr_start_from_emergency_stop(void)
{
    ChgrListDef *pnode = NULL;

	// 切换到之前的界面
    xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_START_FROM_EMERGENCY_STOP, eSetBits );
	
	// 打开所有之前正在充电的模块的输出
	pnode = pchgr_list->next;
	while( pnode != NULL )
	{
        open_output( pnode->pdev->basic.num );	
	    pnode = pnode->next;
	}

	// 恢复充电任务
	resume_charge_damon();
}

static BOOL isBatReverse( float volt )
{

	// 判断是否反接
    if( volt - (float)BAT_REVERSE_VOL < -15 )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

BOOL isBatConn( float volt )
{
	// 判断电池是否连接, 20v - 60v
	if( ( volt - BAT_CONN_VOL_MIN >= 2 ) && ( volt - BAT_CONN_VOL_MAX <= 2 ) )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}

}

static uint8_t read_dc_ok( uint8_t num )
{
    uint16_t pin_tab[] = 
	{
	    DC_OK1_GPIO_PIN,
        DC_OK2_GPIO_PIN,
        DC_OK3_GPIO_PIN,
        DC_OK4_GPIO_PIN,
        DC_OK5_GPIO_PIN,
        DC_OK6_GPIO_PIN,
        DC_OK7_GPIO_PIN,
        DC_OK8_GPIO_PIN
	};

	if( GPIO_READ( DC_OK1_GPIO_PORT, pin_tab[ num ] ) != LOW )
	{
	    LOG_DEBUG_APP_1("\r\nnum:%d, dc_ok_pin - high", num);
	}
	else
	    LOG_DEBUG_APP_1("\r\nnum:%d, dc_ok_pin - low", num);

	return ( GPIO_READ( DC_OK1_GPIO_PORT, pin_tab[ num ] ) );
}

static BOOL isDcOutPutErr( PU_InfoDef *pdev )
{
	// chk ,low - err, high - normal
	if ( read_dc_ok( pdev->basic.num ) != LOW )
	{
	    pdev->chgr.cnt_box[ IDX_CNT_CHK_DC_OK ] ++;
	}
	else
	{
	    pdev->chgr.cnt_box[ IDX_CNT_CHK_DC_OK ] = 0;
	}
	
	// process result
	/*
	 * 连续 FILTER_DC_OK 次数为低电平，说明输出异常 
	 */
	if( pdev->chgr.cnt_box[ IDX_CNT_CHK_DC_OK ] > FILTER_DC_OK )
	{
		pdev->chgr.cnt_box[ IDX_CNT_CHK_DC_OK ] = 0;
	    return TRUE;
	}
	
	return FALSE;	
}

static ChgrListDef *init_chgr_list(void)
{
    ChgrListDef *phead = NULL;
	uint32_t mem_size;	

	if( __LOG_DEBUG_APP )
	{
	    mem_size = xPortGetFreeHeapSize();
	    LOG_DEBUG_FILE_APP("\r\n系统当前内存大小为：%ld 字节", mem_size);
	}

	if( NULL == (phead = (ChgrListDef *)pvPortMalloc( sizeof(DevListDef)) ) ) { LOG_DEBUG_FILE_APP("\r\nmalloc failed"); return NULL; }

	if( __LOG_DEBUG_APP )
	{
	    mem_size = xPortGetFreeHeapSize();
	    LOG_DEBUG_FILE_APP("\r\n系统剩余内存大小为：%ld 字节", mem_size );
	}

	phead->next = NULL;
	
	return phead;
}

static void add_to_chgr_list( PU_InfoDef *pdev )
{
    ChgrListDef *pnode = pchgr_list;
	ChgrListDef *pnew = NULL;	
	uint32_t mem_size;	

	if( __LOG_DEBUG_APP )
	{
	    mem_size = xPortGetFreeHeapSize();
	    LOG_TRACE("\r\n系统当前内存大小为：%ld 字节", mem_size);
	}

	// 遍历到最后一个节点
	while( pnode->next != NULL ) 
		pnode = pnode->next;
		
	// 创建新节点
	if( NULL == ( pnew = (ChgrListDef *)pvPortMalloc( sizeof( ChgrListDef ) ) ) )
	{
		LOG_TRACE_1("\r\nadd to chgr list failed, num:%d", pdev->basic.num);
		return;
	}
	
	// 把待插入的设备地址，放到新节点里面
	pnew->pdev = pdev;
	
	// 把新节点插入到链表中
	pnew->next = pnode->next;
	pnode->next = pnew;
	
	LOG_DEBUG_APP_1("\r\nadd_to_chgr_lis success, num:%d", pnew->pdev->basic.num );

	if( __LOG_DEBUG_APP )
	{
	    mem_size = xPortGetFreeHeapSize();
	    LOG_TRACE_1("\r\n系统剩余内存大小为：%ld 字节", mem_size );
	}
}

static void remove_from_chgr_list( const uint8_t num )
{
    ChgrListDef *p_pre = NULL;
	ChgrListDef *p_remove = NULL;
	uint32_t mem_size;	

	// 查找序号等于num的节点，即待remove的节点
	p_pre = pchgr_list;
	while( p_pre->next != NULL )
	{
		// 找到之后，把待remove的节点保存一下，然后把待remove的节点从链表中移除
		if( p_pre->next->pdev->basic.num == num )
		{
		    p_remove = p_pre->next;
			p_pre->next = p_remove->next;
			
			// 释放待remove节点的空间
            vPortFree( p_remove );
			
	        if( __LOG_DEBUG_APP )
	        {
	            mem_size = xPortGetFreeHeapSize();
	            LOG_TRACE("\r\nremove后，系统剩余内存大小为：%u 字节", mem_size );
	        }

			return;
		}
		
	    p_pre = p_pre->next;
	}
}

static BOOL chk_before_start_in_manual_mode( PU_InfoDef *pdev )
{
	if( pdev->chgr.mode == MODE_AUTO ) { LOG_DEBUG_APP("\r\nnum:%d, when start, mode = auto", pdev->basic.num ); return FALSE; } // 如果当前是自动模式，直接屏蔽
	if( pdev->basic.flag_bms_conn == ON ) { LOG_DEBUG_APP("\r\nnum:%d, when start, bms conn", pdev->basic.num); return FALSE; } // 如果bms连接，直接屏蔽
	if( ( pdev->basic.chgr == BASIC_CHGR_OFF ) &&( pdev->chgr.flag_full == TRUE ) ) { LOG_DEBUG_APP("\r\nnum:%d, when start, flag_full = true", pdev->basic.num); return FALSE; } // 如果在延时关闭，屏蔽
    if( pdev->basic.flag_mode_switch != BASIC_MODE_NONE_SWITCH ) { LOG_DEBUG_APP("\r\nnum:%d, when start, in mode switch, return", pdev->basic.num);  return FALSE; }

	// 判断电池连接状态
	LOG_TRACE_1("\r\nstart num:%d, bat state: ", pdev->basic.num);
	switch( pdev->basic.bat )
	{
		case BASIC_BAT_CONN:
	        LOG_TRACE_1("bat conn");
			break;
		case BASIC_BAT_DISCONN:
	        LOG_TRACE_1("bat disconn");
		    cache_event( pdev, EVENT_BAT_DISCONN );
			return FALSE;
		case BASIC_BAT_REVERSE:
	        LOG_TRACE_1("bat reverse");
		    cache_event( pdev, EVENT_BAT_REVERSE );
			return FALSE;
		case BASIC_BAT_CHGR_ING:
	        LOG_TRACE_1("bat charging");
			return FALSE;
		case BASIC_BAT_FULL:
	        LOG_TRACE_1("bat full");
			break;
		default:
			break;
	}

	// 判断bms是否连接
	if( pdev->basic.flag_bms_conn == ON )
	{
		start_failed( pdev );
	    return FALSE;
	}

    // 如果在执行其他操作，屏蔽
	if( pdev->basic.chgr != BASIC_CHGR_OFF )
	{
		LOG_DEBUG_APP("\r\nnum:%d, when start, basic.chgr != chgr_off, basi_chgr:%x", pdev->basic.num, pdev->basic.chgr);
		return FALSE;
	}



	return TRUE;
}

static void start_by_hmi( PU_InfoDef *pdev )
{
    if( !chk_before_start_in_manual_mode( pdev ) )
	{
		// 显示禁止启动
        xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_NO_START, eSetBits );
		LOG_DEBUG_APP_1("\r\nnum:%d not start", pdev->basic.num);
		return;
	}

	// 执行start操作
	start( pdev );
	
	// 向屏幕发送可以启动的信号，显示启动中
    xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_TO_START, eSetBits );

    // 直接显示启动成功
//    start_success( pdev );
}

static void start_by_booking( PU_InfoDef *pdev )
{
    if( !chk_before_start_in_manual_mode( pdev ) )
		return;

	// 执行start操作
	start( pdev );

    // 直接显示启动成功
//    start_success( pdev );
}

static void start_by_bms( PU_InfoDef *pdev )
{
	assert_app( (pdev->basic.num >= POWER_UNIT_NUM) );

	// 判断该通道是否处于被动关闭状态，如果是，不启动
	if( pdev->basic.chgr == BASIC_CHGR_STOP_BY_HOST )
	{
	    LOG_DEBUG_APP_1("\r\nstart num:%d, in stoy by host status, return", pdev->basic.num);
	}

	// 判断电池连接状态
	LOG_DEBUG_APP_1("\r\nstart num:%d, bat state: ", pdev->basic.num);
	switch( pdev->basic.bat )
	{
		case BASIC_BAT_CONN:
	        LOG_DEBUG_APP_1("bat conn");
			break;
		case BASIC_BAT_DISCONN:
	        LOG_DEBUG_APP_1("bat disconn, return");
			if( pdev->flag_bat_disconn_event_cached != TRUE )
			{
	            LOG_DEBUG_APP_1("\r\ncache bat_disconn event");
				pdev->flag_bat_disconn_event_cached = TRUE;
		        cache_event( pdev, EVENT_BAT_DISCONN );
			}
			else
			{
	            LOG_DEBUG_APP_1("\r\nbat_disconn event has been cached");
			}
			return;
		case BASIC_BAT_REVERSE:
	        LOG_DEBUG_APP_1("bat reverse, return");
		    cache_event( pdev, EVENT_BAT_REVERSE );
			return;
		case BASIC_BAT_CHGR_ING:
	        LOG_DEBUG_APP_1("bat charging");
			return;
		case BASIC_BAT_FULL:
	        LOG_DEBUG_APP_1("bat full");
			break;
		default:
			break;
	}

	start( pdev );
}

void chgr_init(void)
{
BaseType_t xReturn = pdPASS;

		// 初始化充电列表
		if( NULL == (pchgr_list = init_chgr_list())) { printf("\r\ninit chgr list failed"); return; }

		// 获取设备列表
		if( NULL == (pdev_list = get_device_list())) { printf("\r\nget dev list failed"); return; }

		// create damon task
		xReturn = xTaskCreate( (TaskFunction_t) prvChargeDamonTask,
						(const char *) "charge_damon",
						(unsigned short) CHGR_DAMON_STACK,
						(void *) NULL,
						(UBaseType_t) CHGR_DAMON_PRIO,
						(TaskHandle_t *) &xChargeDamonHandle );
		if( xReturn != pdPASS ) { printf("\r\ncreate charge damon failed"); return; }

#if 0
		// create bat chk damon task
		xReturn = xTaskCreate( (TaskFunction_t) prvChkBatDamonTask,
						(const char *) "chk_bat_damon",
						(unsigned short) CHK_BAT_DAMON_STACK,
						(void *) NULL,
						(UBaseType_t) CHK_BAT_DAMON_PRIO,
						(TaskHandle_t *) &xChkBatDamonHandle );
		if( xReturn != pdPASS ) { printf("\r\ncreate chk bat damon failed"); return; }
#endif

#if 0
		// create key damon task
		xReturn = xTaskCreate( (TaskFunction_t) prvKeyDamonTask,
						(const char *) "key_damon",
						(unsigned short) KEY_DAMON_STACK,
						(void *) NULL,
						(UBaseType_t) KEY_DAMON_PRIO,
						(TaskHandle_t *) &xKeyDamonHandle );
		if( xReturn != pdPASS ) { printf("\r\ncreate key damon failed"); return; }
#endif

		// create event damon task
		xReturn = xTaskCreate( (TaskFunction_t) prvEventDaemonWriteTask,
						(const char *) "event_write_syn_damon",
						(unsigned short) EVENT_DAEMON_WRITE_STACK,
						(void *) NULL,
						(UBaseType_t) EVENT_DAEMON_WRITE_PRIO,
						(TaskHandle_t *) &xEventDaemonWriteHandle );
		if( xReturn != pdPASS ) { printf("\r\ncreate event syn damon failed"); return; }

		// create event operation mutex
		xSemEventRW = xSemaphoreCreateMutex();
		if( xSemEventRW == NULL ) { LOG_TRACE("\r\ncreate event operation mutex failed"); return; }
		// 给出互斥量
		xSemaphoreGive( xSemEventRW );

		// create chgr time control task
		xReturn = xTaskCreate( (TaskFunction_t) prvChgrTimeCtrlDaemonTask,
						(const char *) "chgr_time_ctl_damon",
						(unsigned short) CHGR_TIME_CTRL_DAEMON_STACK,
						(void *) NULL,
						(UBaseType_t) CHGR_TIME_CTRL_DAEMON_PRIO,
						(TaskHandle_t *) &xChgrtimeCtrlHandle );
		if( xReturn != pdPASS ) { printf("\r\ncreate chgr time control damon failed"); return; }

		// create chgr smp volt task
		xReturn = xTaskCreate( (TaskFunction_t) prvChgrSmpVoltDaemonTask,
						(const char *) "chgr_smp_volt_damon",
						(unsigned short) CHGR_SMP_VOLT_DAEMON_STACK,
						(void *) NULL,
						(UBaseType_t) CHGR_SMP_VOLT_DAEMON_PRIO,
						(TaskHandle_t *) &xChgrSmpVoltHandle );
		if( xReturn != pdPASS ) { printf("\r\ncreate chgr smp volt damon failed"); return; }
}

void close_ac_input( uint8_t num )
{
	assert_app( ( num >= POWER_UNIT_NUM ) );

	LOG_TRACE_1("\r\nclose uint:%d relay ac input, set low", num );

    gpio_close_ac_input( num );
}

void open_ac_input( uint8_t num )
{
	assert_app( ( num >= POWER_UNIT_NUM ) );

    LOG_TRACE_1("\r\nopen uint:%d relay ac input, set high", num );

    gpio_open_ac_input( num );
}

void close_ac_input_all( void )
{
    uint8_t idx = 0;

	for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
	    close_ac_input( idx );
}

void open_ac_input_all( void )
{
    uint8_t idx = 0;

	for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
	    open_ac_input( idx );
}

void open_dc_relay( uint8_t num )
{
	assert_app( ( num >= POWER_UNIT_NUM ) );

    gpio_open_dc_relay( num );

	LOG_TRACE_1("\r\nopen uint:%d dc relay, set high", num );
}

void close_dc_relay( uint8_t num )
{
	assert_app( ( num >= POWER_UNIT_NUM ) );

    gpio_close_dc_relay( num );

	LOG_TRACE_1("\r\nclose uint:%d dc relay, set low", num );
}

void on_off_control( uint8_t num, uint8_t op )
{
	uint16_t gpio_pin_tab[] = 
	{
		ON_OFF_1_GPIO_PIN,
		ON_OFF_2_GPIO_PIN,
		ON_OFF_3_GPIO_PIN,
		ON_OFF_4_GPIO_PIN,
		ON_OFF_5_GPIO_PIN,
		ON_OFF_6_GPIO_PIN,
		ON_OFF_7_GPIO_PIN,
		ON_OFF_8_GPIO_PIN
	};

	if( num > 8 )
	{
		LOG_TRACE("\r\non_off_control, unit num out of range");
		return;
	}

	if( op == ON )
	{
		GPIO_SET( ON_OFF_1_GPIO_PORT, gpio_pin_tab[ num ] );
		LOG_TRACE_1("\r\nuint:%d on off control open, set high", num );
	}
	else
	{
		GPIO_RESET( ON_OFF_1_GPIO_PORT, gpio_pin_tab[ num ] );
		LOG_TRACE_1("\r\nuint:%d on off control close, set low", num );
	}
}

void chmod_select( uint8_t num, uint8_t mode )
{
	uint16_t gpio_pin_tab[] = 
	{
		RELAY_CHMOD_D1_GPIO_PIN,
		RELAY_CHMOD_D2_GPIO_PIN,
		RELAY_CHMOD_D3_GPIO_PIN,
		RELAY_CHMOD_D4_GPIO_PIN,
		RELAY_CHMOD_D5_GPIO_PIN,
		RELAY_CHMOD_D6_GPIO_PIN,
		RELAY_CHMOD_D7_GPIO_PIN,
		RELAY_CHMOD_D8_GPIO_PIN
	};

	if( num > 8 )
	{
		LOG_DEBUG_FILE_APP("\r\nchmod select, unit num out of range");
		return;
	}

	if( mode == MODE_CURVE )
	{
		GPIO_SET( RELAY_CHMOD_D1_GPIO_PORT, gpio_pin_tab[ num ] );
		LOG_DEBUG_APP("uint:%d select charging curve, set high", num );
	}
	else if( mode == MODE_PMBUS )
	{
		GPIO_RESET( RELAY_CHMOD_D1_GPIO_PORT, gpio_pin_tab[ num ] );
		LOG_DEBUG_APP("uint:%d select pmbus control, set low", num );
	}
}

void power_on( uint8_t num )
{
    open_ac_input( num );

	// 设置开机标志位
	get_device( num )->basic.flag_ac_power = BASIC_POWER_ON;
}

void power_on_all( void )
{
	uint8_t idx = 0;

	open_ac_input_all();

	for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
	{
	    get_device( idx )->basic.flag_ac_power = BASIC_POWER_ON;
	}
}

void power_off( uint8_t num )
{
    close_ac_input( num );

//	vTaskDelay( pdMS_TO_TICKS( POWER_OFF_MS_DELAY ) );

	// 设置关机标志位
	get_device( num )->basic.flag_ac_power = BASIC_POWER_OFF;
}

void power_off_all( void )
{
	uint8_t idx = 0;

    close_ac_input_all();

//	vTaskDelay( pdMS_TO_TICKS( POWER_OFF_MS_DELAY ) );

	for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
	{
	    get_device( idx )->basic.flag_ac_power = BASIC_POWER_OFF;
	}
}

void pub_stop_all( void )
{
ChgrListDef *pnode = NULL;
	
	if( pchgr_list == NULL ) { return; };
	
	pnode = pchgr_list->next;
	while( pnode != NULL )
	{
		stop( pnode->pdev );
		
		pnode = pnode->next;
	}
}

void pub_open_output( uint8_t num )
{
    open_output( num );
}

void pub_start( uint8_t start_type, uint8_t num )
{
	PU_InfoDef *pdev = get_device( num );

	pdev->chgr.start_type = start_type;

	switch ( start_type )
	{
		case START_TYPE_HMI:
            start_by_hmi( pdev );
			break;
		case START_TYPE_BOOKING:
            start_by_booking( pdev );
			break;
		case START_TYPE_BMS:
            start_by_bms( pdev );
			break;
		default:
			break;
	}
}

void pub_stop( uint8_t stop_type, uint8_t num )
{
	PU_InfoDef  *pdev = get_device( num );

	switch ( stop_type )
	{
		case STOP_TYPE_HMI:
            LOG_DEBUG_APP_1(".....stop type->hmi, ");
			if( (pdev->chgr.mode == MODE_AUTO) || (pdev->basic.flag_bms_conn == ON) )
			{
				LOG_DEBUG_APP_1("....in auto mode, not stop");
				return;
			}

	        // 显示停止中延时
	        pdev->chgr.flag_stop = TRUE;

			// 向屏幕发送停止信号
		    xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_TO_STOP, eSetBits);

			// 执行停止操作
			LOG_DEBUG_APP_1("...exec stop operation");
            stop( pdev );
			break;
		case STOP_TYPE_BOOKING:
            LOG_DEBUG_APP_1(".....stop type->booking,");
			if( (pdev->chgr.mode == MODE_AUTO) || (pdev->basic.flag_bms_conn == ON) )
			{
				LOG_DEBUG_APP_1(" ......in auto mode, not stop");
				return;
			}

			// 执行停止操作
			LOG_DEBUG_APP_1("...exec stop operation");
            stop( pdev );
			break;
		case STOP_TYPE_BMS:
            LOG_DEBUG_APP_1(".....stop type->bms,");

			// 直接停止
			LOG_DEBUG_APP_1("...exec stop operation");
            stop( pdev );
			break;
		case STOP_TYPE_HOST:
            LOG_DEBUG_APP_1(".....stop type->by host,");
			if( pdev->basic.chgr == BASIC_CHGR_OFF )
			{
			    LOG_DEBUG_APP_1("\r\nunit %d already in stop status, return\r\n", pdev->basic.num );
				return;
			}

			// 直接停止
			LOG_DEBUG_APP_1("...exec stop operation");
            stop( pdev );

			// 设置模块的状态为被动停止状态
			    // 仅针对自动充电，如果是手动充电模式，不标识该标志位
			if( pdev->chgr.mode == MODE_AUTO )
			    pdev->basic.chgr = BASIC_CHGR_STOP_BY_HOST;
			break;
		default:
			break;
	}
}

void pub_bat_type_switch( uint8_t num, uint8_t to )
{
	PU_InfoDef  *pdev = get_device( num );

	LOG_DEBUG_APP("\r\nnum:%d, exec bat type switch...", num);
    if( (pdev->chgr.mode == MODE_AUTO) || (pdev->basic.flag_bms_conn == ON) )
    {
		LOG_DEBUG_APP_1("....in auto mode, not switch");
		return;
	}
	if( pdev->basic.flag_mode_switch != BASIC_MODE_NONE_SWITCH )
	{
		LOG_DEBUG_APP_1("....mode switch->ing, not switch");
		return;
	}
	if( pdev->basic.chgr != BASIC_CHGR_OFF )
	{
		LOG_DEBUG_APP_1("....not in chgr off, not switch");
		return;
	}

	switch( to )
	{
		case CHGR_BAT_TYPE_LI:
		    LOG_DEBUG_APP_1("\r\nswitch to bat li");
			pdev->basic.flag_mode_switch = BASIC_MODE_THREE_SECTION_TO_PRE;
			break;
		case CHGR_BAT_TYPE_LEAD_ACID:
		    LOG_DEBUG_APP_1("\r\nswitch to bat lead_acid");
			pdev->basic.flag_mode_switch = BASIC_MODE_PRE_TO_THREE_SECTION;
			break;
		default:
			break;
	}

    // 发送给touch_screen_entry通知，显示切换中
    xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_TYPE_SWITCH, eSetBits );
}

ChgrListDef *get_chgr_list(void)
{
	return pchgr_list;
}

void disp_pmbus_comm_status( StatusTypeDef st_pmbus )
{

    LOG_DEBUG_APP_1("st_pmbus: ");
    switch( st_pmbus )
    {
      	case SMBUS_OK:
       		LOG_DEBUG_APP_1("SMBUS_OK");
       		break;
       	case SMBUS_ERROR:
      		LOG_DEBUG_APP_1("SMBUS_ERROR");
       		break;
       	case SMBUS_BUSY:
     		LOG_DEBUG_APP_1("SMBUS_BUSY");
       		break;
       	case SMBUS_TIMEOUT:
       		LOG_DEBUG_APP_1("SMBUS_TIMEOUT");
       		break;
        default:
       		break;
    }
}

void suspen_chgr( void )
{
	// 挂起充电相关
#if 0
	if( eSuspended != eTaskGetState( h_charge_entry ) )
	    vTaskSuspend( h_charge_entry );
#endif
	
	if( eSuspended != eTaskGetState( xChargeDamonHandle ) )
	    vTaskSuspend( xChargeDamonHandle );
	
	if( eSuspended != eTaskGetState( xEventDaemonWriteHandle ) )
	    vTaskSuspend( xEventDaemonWriteHandle );
	
	if( eSuspended != eTaskGetState( xChgrtimeCtrlHandle ) )
	    vTaskSuspend( xChgrtimeCtrlHandle );
}

void resume_chgr( void )
{
    // 恢复充电相关
#if 0
	if( eSuspended == eTaskGetState( h_charge_entry ) )
		vTaskResume( h_charge_entry );
#endif
	
	if( eSuspended == eTaskGetState( xChargeDamonHandle ) )
		vTaskResume( xChargeDamonHandle );
	
	if( eSuspended == eTaskGetState( xEventDaemonWriteHandle ) )
		vTaskResume( xEventDaemonWriteHandle );
	
	if( eSuspended == eTaskGetState( xChgrtimeCtrlHandle ) )
		vTaskResume( xChgrtimeCtrlHandle );
}

