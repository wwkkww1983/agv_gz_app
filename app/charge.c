/************************************************************************
note.
20181226����ɻ����ĳ��������ֹͣ���ܿ������ԣ�
        , �޸�stopʱ������hard fault�����⣻
20181227����Ӳ�����ʾ�Ĺ��ˣ�����ȡ��iout��vout�Ȳ����������ڶ�Ӧ�ͺŵĲ�����Χ�ڣ��Ż�ȥ���¸ò�������ʾ�����򲻸���
        ���޸���⵽��ضϿ���stopʱ��������Ȼ����ʾvout�����⣬��Ϊstop��û�м�return
20190109, ��ɼ�ͣ����ʹ�ð���ɨ��ķ�ʽ��⼱ͣ��Ȼ��ͨ������֪ͨ���м�ͣ�ͻָ���Ϣ�Ĵ��ݣ�����˳�����£�
          prvKeyDamonTask -> charge_entry -> touch_screen_entry
20190114, ͬ�����õ���ͨ��
          SET_NOTIFY_TO_RESTART_ALL, ->touch_screen_entry->charge_entry
          SET_NOTIFY_RESTART_FINISHED, restart_dev_all -> touch_screen_entry
20190124, �����ʷ�¼���ѯ����
          - �¼��Ļ���
		  - �¼��ı��棬��ͬ����eeprom��
		  - �¼��Ķ�ȡ 
20190213, ��ӵ�ؼ�⹦�ܣ��������ӣ�������ͣ��Ͷ�Ӧ���¼���¼
20190216, ��ӵ���Ƿ����ӹ���, �Ͷ�Ӧ���¼���¼
20190225, ��ӿ���״̬�����������״̬����
20190228, �޸�dc_relay��ƽ���ƣ��͵�ƽ�أ��ߵ�ƽ��


������
1. �ǵ���Ӳ������õ���������!
2. ģʽ�л��Ѿ����Σ�������Ҫ�ǵô�
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
#define    FILTER_OVER_OUTPUT     3   // ��ѹ����������Ϊ3��
#define    FILTER_DC_OK           10   // ����쳣�����Ϊ10��
#define    FILTER_VOUT_DISPEAR    3   // �Զ�ģʽ����У������ͻȻ�ر����

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


#define    SMP_BAT_VOLT_FILTER_CNT    100 // ����100�Σ�ȡƽ��ֵ
#define    AVE_IOUT_FILTER_CNT        4 // ����4�Σ�ȡƽ��ֵ

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
#define    KEY_DAMON_STACK    220 // ��Ե���һ�����������ܵ���200�������stack error
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
TickType_t portDelayTime = portMAX_DELAY;  // ��������
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
					   // ִ����������
					   restart_dev( &pnode->dev );
				   }

				   pnode = pnode->next;
				}
			}
			else if( ulNotifiedValue & SET_NOTIFY_TO_RESTART_ALL )
			{
				LOG_TRACE("\r\ncharge_entry rec restart all... ...");
				// ��������ģ��
//			    restart_dev_all();
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_EMERGENCY_STOP )
			{
				    // ����ͣ 
				    LOG_DEBUG_APP("\r\nchgr, stop");
//			        chgr_emergency_stop();
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_START_FROM_EMERGENCY_STOP )
			{
				    // ����Ӽ�ͣ״̬�ָ�
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
		    // ÿ��1s����wdg_daemon�һ�����
	        xTaskNotify( h_wdg_daemon, WDG_BIT_CHGR_TIME_CTRL, eSetBits );
			
			if( pnode->dev.chgr.flag_booking == TRUE )
			{
			    pnode->dev.chgr.flag_booking = FALSE;
				LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d booking finish, start chgr", pnode->dev.chgr.cnt_box[ IDX_CNT_BOOKING_TIME ], pnode->dev.basic.num );
				// ִ����������
				pub_start( START_TYPE_BOOKING, pnode->dev.basic.num );
			}
			if( pnode->dev.chgr.flag_timing == TRUE )
			{
			    pnode->dev.chgr.flag_timing = FALSE;
				LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d  timing finish��stop chgr", pnode->dev.chgr.cnt_box[ IDX_CNT_TIMING_TIME ], pnode->dev.basic.num );
				// ִ��ֹͣ����
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
			    // ����BMSû�����ӵ���Ϣ
			    local_bat_info = get_local_bat_info();
		        reset_bit(local_bat_info[ pnode->dev.basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BMS_CONN);
				// ��ն�Ӧ�ĵ����Ϣ
				memset( &local_bat_info[ pnode->dev.basic.num * SIZE_SINGLE_BAT_INFO + 1 ], 0, ( SIZE_SINGLE_BAT_INFO - 1 ) );
				pnode->dev.bat_info_bms.volt_single_14 = 57.6; // Ĭ��57.6
				pnode->dev.bat_info_bms.volt_single_16 = 57.6; // Ĭ��57.6

				// ��մ�bms��ȡ�ĵ����Ϣ
		        memset( &pnode->dev.bat_info_bms, 0, sizeof( BatInfoDefFromBMS ) );
		        pnode->dev.bat_info_bms.flag_full_volt = FALSE;
		        pnode->dev.bat_info_bms.full_volt = 576; // Ĭ��ֵ�趨Ϊ57.6V

				// ���bms�Ͽ�����ô����Ƿ�ֹͣ״̬
                pnode->dev.basic.warn = BASIC_WARN_NO;

				// ���bms�Ͽ������õ����ѹ����Ϣ����Ϊ0����ն�Ӧ�Ļ���
				pnode->dev.bat_info_bms.single_bat_volt_len = 0;
				if( NULL != ( p_single_bat_volt_info_fifo = getSingleBatVoltInfoFifo() ) )
				    memset( (p_single_bat_volt_info_fifo + pnode->dev.basic.num * SIZE_SINGLE_BAT_VOLT_INFO), 0, SIZE_SINGLE_BAT_VOLT_INFO );

				// ���bms�Ͽ������bms�汾��Ϣ
				memset(pnode->dev.bat_info_bms.bms_ver, 0, SIZE_BMS_VER);
				if( NULL != ( p_bms_info_fifo = getBatBmsVerFifo() ) )
				    memset( (p_bms_info_fifo+pnode->dev.basic.num*SIZE_BMS_VER), 0, SIZE_BMS_VER );

				// ���bms�Ͽ������ID������Ϣ
				memset(pnode->dev.bat_info_bms.id_num, 0, SIZE_BAT_ID_NUM);
				if( NULL != ( p_bat_id_num_fifo = getBatIdNumFifo() ) )
				    memset( (p_bat_id_num_fifo+pnode->dev.basic.num*SIZE_BAT_ID_NUM), 0, SIZE_BAT_ID_NUM );

				// ���bms�Ͽ��������ѯ��־λ
				pnode->dev.chgr.auto_flag_bms_start_query = FALSE;
				pnode->dev.chgr.cnt_box[ IDX_CNT_DELAY_BMS_START_QUERY ] = 0;

				// bms�Ͽ�ʱ��������MOS�رյĸ澯��Ϣ��Ȼ���ڣ�����澯��Ϣ��20191030���
	            if( !( pnode->dev.bat_info_bms.warn_4 & bit( 6 ) ) )
					reset_bit( pnode->dev.basic.err, ERR_BAT_CHGR_MOS_CLOSE );

				// ����Ƿ���Ҫcache��ضϿ��¼��ı�־λ
				pnode->dev.flag_bat_disconn_event_cached = FALSE;

			    LOG_DEBUG_APP_1("switch to manual mode");
			    pnode->dev.basic.flag_mode_switch = BASIC_MODE_AUTO_TO_MANUAL; 
			}

			/*
			 * ������ʱ�ر�AC����
			 */
            deal_with_delay_close_ac( &pnode->dev );
			
			// ����ģʽ�л�
			if( pnode->dev.basic.flag_mode_switch != BASIC_MODE_NONE_SWITCH )
			    chgr_mode_switch( &pnode->dev );

			// ���³��վ״̬��Ϣ���ϴ�����λ��
           update_local_chgr_station_info( &pnode->dev );

		   // ���뱻��ֹͣ״̬�󣬼�����Ƿ�Ͽ�
//		   deal_with_bat_disconn_in_stop_by_host( &pnode->dev );
			
	       pnode = pnode->next;
	    }
	
		vTaskDelay( pdMS_TO_TICKS(1000) );
	}
}

static void clear_chgr_info( PU_InfoDef *pdev )
{
	// ��������ˢ��
    if( ( pdev->basic.chgr == BASIC_CHGR_OFF ) || ( pdev->basic.chgr == BASIC_CHGR_STOP_BY_HOST ) )
	{
		pdev->basic.temp = 25;
		
        pdev->chgr.status = (uint16_t)0;

		// ��ն�Ӧģ�������
		pdev->chgr.vout = 0;
		pdev->chgr.iout = 0;
		pdev->chgr.time = 0;
		
	    // �Զ���粿��
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
 * 100us��һ�Σ�������100�Σ�ȡ��ƽ��ֵ����Ϊʵ�ʵĵ�ص�ѹֵ
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

	// ����У׼ֵ
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
		// 1. �ȹر�ģ��	
		// ���ģ���ǿ�����״̬����ô�͹ص���Ȼ����ʱ8s���л�ģʽ
		if( pdev->basic.flag_ac_power == BASIC_POWER_ON )
		{
			LOG_DEBUG_APP("\r\nģʽ�л�ʱ��ģ���ǿ���״̬��ֱ�ӹص�, num(0-base):%d", pdev->basic.num);
			power_off( pdev->basic.num );
			pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] = 0;
		}
		// 2. ��ʱ8s���ȴ�ģ��ر����
		pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] ++;

		// 3. ��ʱʱ�䵽���л�ģʽ
		if( pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] > 8 )
		{
			if( pdev->basic.flag_mode_switch == BASIC_MODE_AUTO_TO_MANUAL )
			{
			    pdev->basic.flag_mode_switch = BASIC_MODE_NONE_SWITCH;

			    LOG_DEBUG_APP("\r\nģʽ�л���ʱʱ�䵽���л����ֶ�ģʽ, num(0-base):%d", pdev->basic.num);
		        pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] = 0;
			    
				if( pdev->chgr.type == CHGR_TYPE_PRE )
				{
					LOG_DEBUG_APP_1("\r\n��緽ʽΪԤ��緽ʽ");
			        chmod_select( pdev->basic.num, MODE_PMBUS );
				}
				else if ( pdev->chgr.type == CHGR_TYPE_THREE_SECTION )
				{
					LOG_DEBUG_APP_1("\r\n��緽ʽΪ����ʽ������߷�ʽ");
			        chmod_select( pdev->basic.num, MODE_CURVE );
				}
						
	            pdev->chgr.mode = MODE_MANUAL;  
			}
			else if( pdev->basic.flag_mode_switch == BASIC_MODE_MANUAL_TO_AUTO )
			{
			    pdev->basic.flag_mode_switch = BASIC_MODE_NONE_SWITCH;

			    LOG_DEBUG_APP("\r\nģʽ�л���ʱʱ�䵽���л����Զ�ģʽ, num(0-base):%d", pdev->basic.num);
		        pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] = 0;
			    
			    chmod_select( pdev->basic.num, MODE_PMBUS );
	            pdev->chgr.mode = MODE_AUTO;  
			}
			else if( pdev->basic.flag_mode_switch == BASIC_MODE_PRE_TO_THREE_SECTION )
			{
			    pdev->basic.flag_mode_switch = BASIC_MODE_NONE_SWITCH;

			    LOG_DEBUG_APP("\r\n��緽ʽ�л���ʱʱ�䵽���л�������ʽ���߷�ʽ, num(0-base):%d", pdev->basic.num);
		        pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] = 0;
			    
			    chmod_select( pdev->basic.num, MODE_CURVE );

				LOG_DEBUG_APP_1("\r\n���õ������ΪǦ����");
				pdev->chgr.bat_type = CHGR_BAT_TYPE_LEAD_ACID;
				pdev->chgr.type = CHGR_TYPE_THREE_SECTION;

			    // ���͸�touch_screen_entry֪ͨ����ʾ�л����
	            xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_TYPE_SWITCH_FINISH, eSetBits );
			}
			else if( pdev->basic.flag_mode_switch == BASIC_MODE_THREE_SECTION_TO_PRE )
			{
			    pdev->basic.flag_mode_switch = BASIC_MODE_NONE_SWITCH;

			    LOG_DEBUG_APP("\r\n��緽ʽ�л���ʱʱ�䵽���л���Ԥ��緽ʽ, num(0-base):%d", pdev->basic.num);
		        pdev->chgr.cnt_box[ IDX_CNT_DELAY_MODE_SWITCH ] = 0;
			    
			    chmod_select( pdev->basic.num, MODE_PMBUS );

				LOG_DEBUG_APP_1("\r\n���õ������Ϊ﮵��");
				pdev->chgr.bat_type = CHGR_BAT_TYPE_LI;
				pdev->chgr.type = CHGR_TYPE_PRE;

			    // ���͸�touch_screen_entry֪ͨ����ʾ�л����
	            xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_TYPE_SWITCH_FINISH, eSetBits );
			}

			// �����Ҫ��ʱ�رգ���ô�����´�
			if( pdev->basic.flag_delay_close_ac != OFF )
			{
			    LOG_DEBUG_APP("\r\nģʽ�л���ɣ�������Ҫ��ʱ�رգ����´�ģ��, num(0-base):%d", pdev->basic.num); 
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

		// ÿ��100ms����wdg_daemon�һ�����
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
		    // ��ⰴ���Ƿ���
    		if( key_detect( &key_emergency_stop ) == KEY_ON )
    		{
				// ֪ͨ�����ں�����ִ���˽���ͣ��
                xTaskNotify( h_charge_entry, CHGR_NOTIFY_EMERGENCY_STOP, eSetBits );
    		    _step = _step_chk_release;
    		}
    	}
    	else if( _step == _step_chk_release )
    	{
			// ��ⰴ���Ƿ��ͷ�
    		if( key_detect( &key_emergency_stop ) == KEY_OFF )
    		{
                xTaskNotify( h_charge_entry, CHGR_NOTIFY_START_FROM_EMERGENCY_STOP, eSetBits );
    		    _step = _step_chk_press;
    		}
    	}

		// ÿ10ms���һ��
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
		// ��������ģ��
		pnode = pdev_list->next;

		while( pnode != NULL)
		{
		    // ÿ��500ms����wdg_daemon�һ�����
	        xTaskNotify( h_wdg_daemon, WDG_BIT_EVENT_WRITE, eSetBits );

			// ��ѯ��ģ����¼�����
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

					// �����Ӧ���¼�
				    save_event( &pnode->dev, idx );

					// ��ն�Ӧ���¼�
					reset_bit( pnode->dev.event_cache.code, idx );

	                // ����������
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
	uint8_t num = 0;          // ģ�����
	uint16_t latest_idx = 0;  // ���´洢�¼������
	uint32_t ee_wr_addr = 0;  // дeeprom�ĵ�ַ
    EE_SysEventDef event;     // ���洢���¼�
	uint16_t flag_full = 0;   // ������¼�¼��Ƿ��Ѿ�����

    LOG_DEBUG_APP_1("\r\nsave_event, num:%d, event_code:%d,time:20%02x-%02x-%02x %02x:%02x:%02x", pdev->basic.num, event_code, \
					  pdev->event_cache.time[ event_code ].year, pdev->event_cache.time[ event_code ].month, pdev->event_cache.time[ event_code ].day, \
					  pdev->event_cache.time[ event_code ].hour, pdev->event_cache.time[ event_code ].min, pdev->event_cache.time[ event_code ].sec );

	// ��ȡģ�����
	num = pdev->basic.num;

	// ��ȡ���µ����latest_idx, Ϊ����д��ĵ�ַ
    ee_ReadBytes( (uint8_t *)&latest_idx, EE_EVENT_LATEST_IDX_BASE_ADDR + ( num * EE_EVENT_LATEST_IDX_LEN ), EE_EVENT_LATEST_IDX_LEN );
	vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	LOG_DEBUG_APP_1("\r\nsave event, before save, latest_idx:%d", latest_idx );

	// д���¼�
	/* ����eeprom��д���ַ:
	 * 1. �Ȼ�ȡ��ģ�� �洢�¼� �Ļ���ַ��
	 * 2. ��ȡд��ĵ�ַ����: ����ַ + latest_idx * sizeof(EE_SysEventDef)
	 * ���ôӴ�Сд�������ڵĶ���ʱ���С�����������֧��eeprom��page_read��
	 */
	ee_wr_addr = EE_EVENT_STORE_BASE_ADDR + num * EVENT_MAX_STORE_NUM * sizeof( EE_SysEventDef ) + \
				 (latest_idx) * sizeof( EE_SysEventDef );

	// �����¼��洢����
	event.sys_time = pdev->event_cache.time[ event_code ]; // ��ȡ��Ӧ�¼�������ʱ��
	event.code = (uint16_t)( ( event_code & 0xFF ) << 8); // �߰�λ�洢�¼�����
	// ����������¼�����Ҫ��¼ģʽ�Ͳ���
	if( event_code == EVENT_START )
	{
		event.code |= ( ( ( ( (uint8_t)event.code ) & 0x0F ) >> 4 ) | ( pdev->chgr.mode & 0x0F) ) << 4;    // �Ͱ�λ�ĸ���λ��ģʽ
        event.code |= ( ( ( (uint8_t)event.code ) & 0xF0 ) | ( pdev->chgr.para.curve & 0x0F ) );    // �Ͱ�λ�ĵ���λ�ǲ���
	}

	ee_WriteBytes( (uint8_t *)&event, ee_wr_addr, sizeof( EE_SysEventDef ) );
	vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	// ���������¼����
	if( latest_idx != 0 )
	{
	    latest_idx --;
	}
	else if( latest_idx == 0 )  // ������µ�д������Ѿ���0�ˣ���ôѭ���������������¸���д
	{	
	    latest_idx = EVENT_MAX_STORE_NUM - 1;	

		// ��ʱ�����֮ǰ�¼�û��������ô�����¼��Ѿ�����
		if( !is_ee_event_full( num ) )
		{
	    	// ��¼�Ѿ��¼��Ѿ�����
            ee_ReadBytes((uint8_t *)&flag_full, EE_EVENT_IS_FULL_ADDR, EE_EVENT_IS_FULL_LEN );
	        vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	    	set_bit( flag_full, num );
	    	
            ee_WriteBytes((uint8_t *)&flag_full, EE_EVENT_IS_FULL_ADDR, EE_EVENT_IS_FULL_LEN );
	        vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
		}
	}
	// �������¼���ű��浽eeprom
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
	uint32_t ee_base_addr = 0;    //  �洢��ǰģ�飬��ʷ�¼��Ļ���ַ
	uint32_t ee_read_addr = 0;    //  ����ʷ�¼��ĵ�ַ

	// ģ�����
	num = pdev->basic.num;

	LOG_DEBUG_APP("\r\n-------------start read_event, num:%d ....", num);

	// ��ȡ������
	if( pdTRUE != xSemaphoreTake( xSemEventRW, pdMS_TO_TICKS(500) ) )
	{
	    LOG_DEBUG_APP_1("\r\nread event, get mutex failed");
	    LOG_DEBUG_APP_1("\r\n... read event finish--------------------end");
		return FALSE;
	}
	else
	    LOG_DEBUG_APP_1("\r\nread event, get mutex success");

	// ִ�ж��Ĳ���	
	    // ��ȡ���µ�д�����
    ee_ReadBytes( (uint8_t *)&latest_idx, EE_EVENT_LATEST_IDX_BASE_ADDR + ( num * EE_EVENT_LATEST_IDX_LEN ), EE_EVENT_LATEST_IDX_LEN );
	vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
	LOG_DEBUG_APP_1("\r\nread_event, latest_idx:%d", latest_idx);

    ee_base_addr = EE_EVENT_STORE_BASE_ADDR + num * EVENT_MAX_STORE_NUM * sizeof( EE_SysEventDef ); 
	
	// ��ȷ�����µĶ����
	latest_read_idx = latest_idx + 1;
	if( latest_read_idx > ( EVENT_MAX_STORE_NUM - 1 ) )
		latest_read_idx = 0;	

    // ����ҳ����ţ�ȷ��ʵ�ʶ������
	read_idx = latest_read_idx + num_to_read * page_num;
	LOG_DEBUG_APP_1("\r\nread_idx:%d", read_idx );

	// ������ʷ��¼�������˵�������������, ѭ����ȡ
	if( is_ee_event_full( num ) )
	{
		// �����ȡ��ų����������ţ���ôѭ������С��ſ�ʼ�����ȡ�����
		if( read_idx > ( EVENT_MAX_STORE_NUM - 1 ) )
			read_idx = read_idx - EVENT_MAX_STORE_NUM;

		// ����ʵ�ʶ�����ţ�ȷ��eepromʵ�ʶ��ĵ�ַ
	    ee_read_addr = ee_base_addr + read_idx * sizeof( EE_SysEventDef );

		// ���������¼����һ�ζ���
		if( (read_idx + num_to_read - 1) > (EVENT_MAX_STORE_NUM - 1) )
		{
            // �ȶ���һ����
            part_1_num = EVENT_MAX_STORE_NUM - read_idx;
			num_actual_read = part_1_num;

		    ee_ReadBytes((uint8_t*)pevent, ee_read_addr, ( num_actual_read * sizeof( EE_SysEventDef ) ) );
	        vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

            // �ٶ��ڶ�����
			part_2_num = num_to_read - part_1_num;
			num_actual_read = part_2_num;
            ee_read_addr = ee_base_addr;  // �ӻ���ַ��ʼ��
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
	// ������ʷ��¼����û���������
	else  
	{
		// �����ȡ��ų����������Ż����������Ϊ�����ţ�˵����ҳ����Ϊ�գ�����FALSE
		if( ( read_idx > ( EVENT_MAX_STORE_NUM - 1 ) ) || ( latest_idx == ( EVENT_MAX_STORE_NUM - 1 )  ) )
		{
		    *p_num_actual_read = 0;

	        LOG_DEBUG_APP_1("\r\nin event is not full, read_idx:%d, page:%d event is empty, return false", read_idx, page_num );
	        // ����������
	        if( pdTRUE != xSemaphoreGive( xSemEventRW ) )
	        {
	            LOG_DEBUG_APP_1("\r\nread event, give mutex failed");	
	        }

			return FALSE;
		}

		// ����ʵ�ʶ�����ţ�ȷ��eepromʵ�ʶ��ĵ�ַ
	    ee_read_addr = ee_base_addr + read_idx * sizeof( EE_SysEventDef );

		// �����ҳ����ʷ��¼��������num_to_read����ô�Ͱ�ʵ�ʴ��˶��ٸ���
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

		// ����ʵ�ʶ�ȡ������
	    *p_num_actual_read = num_actual_read;
	}

	// ����������
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

	// ��ȡ��ǰʱ��
	get_sys_time( &time );

	pdev->event_cache.time[event_code] = time;

    LOG_TRACE_1("\r\nread time:20%02x-%02x-%02x %02x:%02x:%02x", \
					  time.year, time.month, time.day, \
					  time.hour, time.min, time.sec );

    LOG_TRACE_1("\r\ncache time:20%02x-%02x-%02x %02x:%02x:%02x", \
					  pdev->event_cache.time[ event_code ].year, pdev->event_cache.time[ event_code ].month, pdev->event_cache.time[ event_code ].day, \
					  pdev->event_cache.time[ event_code ].hour, pdev->event_cache.time[ event_code ].min, pdev->event_cache.time[ event_code ].sec );

	// ��¼���ö�Ӧ���¼�λ
	set_bit( pdev->event_cache.code, event_code );

	LOG_TRACE_1("\r\n... cache_event finish---------------end" );
}

void deal_with_chgr_booking( PU_InfoDef *pdev )
{
	// δ���
	if( pdev->basic.chgr == BASIC_CHGR_OFF )
	{
		// �ж�ԤԼ���
		if( pdev->chgr.st_booking_time == ENABLE )
		{
			pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ] ++;
//			LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d ԤԼʱ�䣬��ʱ��", pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ], pdev->basic.num );

			// ���ԤԼʱ�䵽
			if( pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ] >= ( pdev->chgr.booking_time * 60 ) )
//			if( pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ] >= ( pdev->chgr.booking_time ) )
			{
//				LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d ԤԼʱ�䵽���������", pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ], pdev->basic.num );
//				LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d booking finish, start chgr", pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ], pdev->basic.num );

				// �ر�ԤԼ���
				pdev->chgr.st_booking_time = DISABLE;

				// ���������
				pdev->chgr.cnt_box[ IDX_CNT_BOOKING_TIME ] = 0;

				// ��ʾԤԼʱ�䵽��
				pdev->chgr.flag_booking = TRUE;

				// ִ����������
//				pub_start( START_TYPE_BOOKING, pdev->basic.num );
		    }
        }
	}
}

void deal_with_chgr_timing( PU_InfoDef *pdev )
{
  	// �ڳ��
  	if( ( pdev->basic.chgr == BASIC_CHGR_ING ) || ( pdev->basic.chgr == BASIC_CHGR_FINISH ) )
  	{
  		// �ж϶�ʱ���
  	    if( pdev->chgr.st_timing_time == ENABLE )
  		{
  		    pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ] ++;
//  			LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d  ��ʱʱ���ʱ��", pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ], pdev->basic.num );

  			// �����ʱʱ�䵽
  		    if( pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ] >= ( pdev->chgr.timing_time * 60 ) )
//    		if( pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ] >= ( pdev->chgr.timing_time ) )
  			{
//				LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d  ��ʱʱ�䵽��ֹͣ���", pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ], pdev->basic.num );
//				LOG_DEBUG_APP("\r\ncnt(s):%ld, num:%d  timing finish��stop chgr", pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ], pdev->basic.num );

				// �رն�ʱ���
                pdev->chgr.st_timing_time = DISABLE;

               // ��ն�ʱ������
                pdev->chgr.cnt_box[ IDX_CNT_TIMING_TIME ] = 0;

				// ��ʾ��ʱʱ�䵽��
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
				
		// �ֶ�ģʽ�£������ģ����������������ô�͹ر�ģ�飬Ȼ����ʱ8����ٴ�
		if( pdev->chgr.flag_full == TRUE )
		{
			LOG_DEBUG_APP("\r\n�������������ر�ac����, num:%d", pdev->basic.num);
		    power_off( pdev->basic.num );
			if( pdev->chgr.cnt_box[IDX_CNT_DELAY_CLOSE_AC] >= 9 )
			{
				pdev->basic.flag_delay_close_ac = OFF;

				pdev->chgr.flag_full = FALSE;
				LOG_DEBUG_APP("\r\n���´�AC��������ɢ�ȣ�num:%d", pdev->basic.num);
			    power_on( pdev->basic.num );
				pdev->chgr.cnt_box[IDX_CNT_DELAY_CLOSE_AC] = 0;
			}
		}

		// ��ȡģ���¶�
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
			LOG_DEBUG_APP("\r\n��ʱ�ر�ģ��ʱ�䵽���ر�ģ��AC����, num:%d", pdev->basic.num);
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
	// �����
    open_output( pdev->basic.num );

	// ��ռ�����
	memset( pdev->chgr.cnt_box, 0, sizeof(pdev->chgr.cnt_box ) );
	
	// ������ʱ�ر�ACΪOFF����ֹ����������йر�ģ��
	pdev->basic.flag_delay_close_ac = OFF;
	
	// ���������־λ
	pdev->chgr.flag_full = FALSE;

	// ���ó��״̬Ϊ������
	pdev->basic.chgr = BASIC_CHGR_STARTING;
	
	// ����open output �׶�
    pdev->chgr.step = CHGR_STEP_OPEN_OUTPUT;
	
    add_to_chgr_list( pdev );

	// ...
	pdev->chgr.flag_in_chgr = TRUE;
}

static void start_success( PU_InfoDef *pdev )
{
	uint8_t *local_bat_info = NULL;
	
	// ����ģ��״̬Ϊ��û���쳣
	pdev->basic.err = BASIC_ERR_NO;
	pdev->basic.warn = BASIC_WARN_NO;
	// ����ģ���־λ����ʾ����ڳ����
	pdev->basic.bat = BASIC_BAT_CHGR_ING;
	// ����ģ���־λ����ʾ�����
//	pdev->basic.chgr = BASIC_CHGR_ING;
	// ������ʱ�ر�ACΪOFF
	pdev->basic.flag_delay_close_ac = OFF;
	
	// ���õ�ǰop״̬Ϊ start״̬
	pdev->chgr.op = op_start;

    // ���µ�ػ�����Ϣ����ʾ�����
	local_bat_info = get_local_bat_info();
	set_bit( local_bat_info[pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_CHGRING );
	
	// ��������¼�
	cache_event( pdev, EVENT_START );

	// ��������־
    reset_bit( pdev->basic.err, ERR_DC_OK );
	
	LOG_DEBUG_APP_1( " ....start num:%d success", pdev->basic.num );
	
	// ����ǽ������������͸�touch_screen_entry֪ͨ�������ɹ�
	if( pdev->chgr.start_type == START_TYPE_HMI )
	{
	    xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_START_SUCCESS, eSetBits );
	}
}

static void start_failed( PU_InfoDef *pdev )
{
	// ��ռ�����
	memset( pdev->chgr.cnt_box, 0, sizeof(pdev->chgr.cnt_box ) );
		
	// stop������ʧ�ܵ��豸
	stop( pdev );
	
	// ����ʧ�ܣ�ֱ�ӹر�ac����
	power_off( pdev->basic.num );

	// ������ʱ�ر�ACΪOFF
	pdev->basic.flag_delay_close_ac = OFF;
	
	// ����ǽ��������ģ����͸�touch_screen_entry֪ͨ������ʧ��
	if( pdev->chgr.start_type == START_TYPE_HMI )
	{
	    xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_START_FAILED, eSetBits );
	}
	
	LOG_DEBUG_APP_1( " ....start num:%d failed", pdev->basic.num );
}

static void stop( PU_InfoDef *pdev )
{	
	uint8_t *local_bat_info = NULL;
	
	// �رն�Ӧģ���DC���
	close_output( pdev->basic.num );
	
	// ����ģ���־λ����ʾδ���
	pdev->basic.chgr = BASIC_CHGR_OFF;
    // ���÷���ת��Ϊ0����Ϊ��ʱģ���Ѿ��ر���
	pdev->basic.fan_speed = 0;
	// ������ʱ�ر�ACΪON����ʼ��ʱ
	pdev->basic.flag_delay_close_ac = ON;
	pdev->basic.temp = 25;

	// ���ó�粽��Ϊ���У���ʾδ���
    pdev->chgr.step = CHGR_STEP_IDLE;
	// ����ģ��״̬Ϊ:0
    pdev->chgr.status = (uint16_t)0;
	
	// ���õ�ǰop״̬Ϊ stop״̬
	pdev->chgr.op = op_stop;

    // ���µ�ػ�����Ϣ����ʾδ���
	local_bat_info = get_local_bat_info();
	reset_bit( local_bat_info[pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_CHGRING );
	
	// ��ն�Ӧģ�������
    pdev->chgr.vout = 0;
	pdev->chgr.iout = 0;
	pdev->chgr.time = 0;
	memset( pdev->chgr.cnt_box, 0, sizeof(pdev->chgr.cnt_box ) );

	pdev->chgr.iout_ori = 0;
	pdev->chgr.iout_disp = 0;
	memset( &pdev->chgr.iout_queue, 0, sizeof( Queue_Float ) );
	
	   // �Զ���粿��
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

	// ����Ƿ���Ҫcache��ضϿ��¼��ı�־λ
	pdev->flag_bat_disconn_event_cached = FALSE;

	/* ֹͣ����
	 * 1.ֱ���Ƴ����ɣ���Ϊ����stop֮�󣬲������ִ�и��豸�ĳ�����( ����ʱ��Ҫȷ����һ�� )
	 */
	LOG_DEBUG_APP_1("\r\nremove unit:%d from chgr list", pdev->basic.num);
	remove_from_chgr_list( pdev->basic.num );

	// ...
	pdev->chgr.flag_in_chgr = FALSE;

	// ���ֹͣ�¼�
	cache_event( pdev, EVENT_STOP );

	LOG_DEBUG_APP_1(" ....num:%d stop finish", pdev->basic.num );
}

static void restart_dev( PU_InfoDef *pdev )
{
	uint32_t timer = 0;
    // �ȴ���������ִ����ϣ�ÿ10ms��ѯһ�Σ��ʱ��ȴ�4s
	while( (__chgr_damon_step != __END) || (timer > 400) ) { timer ++; vTaskDelay( pdMS_TO_TICKS(10) ); };
	
	// ����������
	if( eSuspended != eTaskGetState( xChargeDamonHandle ) )
		vTaskSuspend( xChargeDamonHandle );
	
	// �ر�AC����
	close_ac_input( pdev->basic.num );
	
	// ��ʱ 10s��
	vTaskDelay( pdMS_TO_TICKS(10000) );
	
	// ��ac����
	open_ac_input( pdev->basic.num );
	
	// ��ʱ3s���ȴ�ģ���ȶ�
	vTaskDelay( pdMS_TO_TICKS(3000) );
	
	if( eSuspended == eTaskGetState( xChargeDamonHandle ) )
	{
	    vTaskResume( xChargeDamonHandle );
	}
	
    // ���͸�touch_screen_entry֪ͨ���������	
	LOG_TRACE("\r\nsend restart finish to touch screen.... ");
	xTaskNotify( h_touch_screen_entry, SET_NOTIFY_RESTART_FINISHED, eSetBits );
}

static void restart_dev_all( void )
{
	// �ر�����ģ���AC����
	close_ac_input_all();

	// ��ʱ 10s��
	vTaskDelay( pdMS_TO_TICKS(10000) );

	// ������ģ���AC����
    open_ac_input_all();

	// ��ʱ3s���ȴ�ģ���ȶ�
	vTaskDelay( pdMS_TO_TICKS(3000) );

    // ���͸�touch_screen_entry֪ͨ���������
	LOG_TRACE("\r\nsend restart all finish to touch screen.... ");
	xTaskNotify( h_touch_screen_entry, SET_NOTIFY_RESTART_FINISHED, eSetBits );
}

static void chk_bat_conn_st( PU_InfoDef *pdev )
{
	float volt = 0;
	uint8_t *p_local_bat_info = NULL;

	// ��ȡ���ص����Ϣ
	p_local_bat_info = get_local_bat_info();
	
	// ��ȡ�����ĵ�ص�ѹ
    volt = pdev->basic.bat_volt;
	
	// ������ڴ�����׶λ��߳���У���ȡ������ѹֵ�󣬲��ý�������ļ��
	if( (pdev->basic.chgr == BASIC_CHGR_STARTING) || \
	    (pdev->basic.chgr == BASIC_CHGR_ING) || \
	    (pdev->basic.chgr == BASIC_CHGR_FINISH) )
		return;
	if( pdev->chgr.step == CHGR_STEP_IDLE )
        pdev->chgr.step = CHGR_STEP_CHK_BAT_REVERSE;
	
    if( pdev->chgr.step == CHGR_STEP_CHK_BAT_REVERSE )
    {
    	// ������Ƿ񷴽�
    	if( isBatReverse( volt ) )
        {
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] --; 
    	}
    	else
    	{
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] ++; 
    	}
    
    	// ������ͨ�� 
    	if( pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] >= (int32_t)FILTER_CHK_BAT_REVERSE )
    	{
    		// �������������û������
            pdev->chgr.step = CHGR_STEP_CHK_BAT_CONN;
    
    		// ��������־
            reset_bit( pdev->basic.err, ERR_BAT_REVERSE );
    		
    		// ���±��ص����Ϣ�����У���ط�����Ϣ
            reset_bit(p_local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_REVERSE);
    
    		// ���������
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] = 0;
    	}
    	// ������δͨ�� 
    	else if( pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] <= (int32_t)( FILTER_CHK_BAT_REVERSE * (-1) ) )
    	{
    		// ���õ�ط��ӱ�־
    		pdev->basic.bat = BASIC_BAT_REVERSE;
    
    		// ���ô����־
            set_bit( pdev->basic.err, ERR_BAT_REVERSE );
    
    		// ���±��ص����Ϣ�����У���ط�����Ϣ
            set_bit(p_local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_REVERSE);
    		if( !( pdev->basic.err & bit( ERR_BAT_REVERSE ) ) )
    		{
    		    // ��¼��ط����¼�
    		    cache_event( pdev, EVENT_BAT_REVERSE );
    		}
    		
    		// ���ؿ��У��������
    	    pdev->chgr.step = CHGR_STEP_IDLE;
    
    		// ���������
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] = 0;
    	}
    }
    
    if( pdev->chgr.step == CHGR_STEP_CHK_BAT_CONN )
    {
    	// ������Ƿ�����
    	if( isBatConn( volt ) )
    	{
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] ++; 
    	}
    	else
    	{
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] --; 
    	}
    
    	// ������ͨ�� 
    	if( pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] >= (int32_t)FILTER_CHK_BAT_CONN )
    	{
    		// ���õ�����ӱ�־
    		pdev->basic.bat = BASIC_BAT_CONN;
    
    		// ���±��ص����Ϣ�����У����������Ϣ
            set_bit(p_local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_CONN);
    		
            // ��ȡ��ص�ѹ,���µ�ص�ѹ������ҳ��ʾ 
            pdev->basic.bat_volt = volt;
    
    		// ���������
            pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] = 0;
    
    		// ���ؿ���״̬�����¼��
    	    pdev->chgr.step = CHGR_STEP_IDLE;
    	}
    	// ������δͨ�� 
    	else if( pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] <= (int32_t)( FILTER_CHK_BAT_CONN * (-1) ) )
    	{
    		// ���õ��δ���ӱ�־
    		pdev->basic.bat = BASIC_BAT_DISCONN;
    		
    		// ���±��ص����Ϣ�����У����������Ϣ
            reset_bit(p_local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_CONN);
    
    		// ���������
            pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] = 0;
    
    		// ���ؿ���״̬�����¼��
    	    pdev->chgr.step = CHGR_STEP_IDLE;
    	}
    }
}

static BOOL chk_vout_in_starting( PU_InfoDef *pdev )
{
	float vout, volt, set_vout;
	StatusTypeDef st_pmbus;

	// �жϼ���Ƿ�ʱ
    if( pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] >= chgrMS_TO_CNT( TIMEOUT_CHK_VOUT_IN_STARTING ) )
	{
	    LOG_WARNING_APP("%dms has been passed, chk vout failed in starting, stop",(uint16_t)TIMEOUT_CHK_VOUT_IN_STARTING );
        pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] = 0;
		stop( pdev );
		return FALSE;
	}

	// ����ص�ѹ
	volt = pdev->basic.bat_volt;
	LOG_DEBUG_APP_1("\r\nvolt:%f", volt);

	// ��ģ�������ѹ
	ReadVout_St( pdev->basic.addr, &vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif
	LOG_DEBUG_APP_1("\r\nread_vout:%f ", vout );
	disp_pmbus_comm_status( st_pmbus );
	if( ( vout < 0 ) || ( vout - pdev->chgr.data_max.vbst > 1 ) )
	{
			LOG_WARNING_APP("\r\nread vout err, return" );
			return FALSE;
	}

	// �ж������ѹ�Ƿ�����
	if( ( vout - VOUT_SUCCESS_THRESHOLD ) >= 0 )
	{
        LOG_INFO_APP("vout:%f, >= %f, chk vout success, cost:%.1fs, to next step...", vout, (float)VOUT_SUCCESS_THRESHOLD, (float)(pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ]*CHGR_DAMON_PERIOD) / 1000 );
        pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] = 0;
		// ����������������
	    pdev->chgr.step = CHGR_STEP_CHECK_OUTPUT_IOUT;
   	    // ���»�׼�����Ϣ
   	    pdev->chgr.volt_base = volt;
		/* �����Ԥ���ģʽ�����ݵ�ص�ѹ�ж�Ԥ��绹�Ǻ�����
		 * �Ӷ�ȷ���������жϵ��������У����ڵ���ʱ���޶�ֵ
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

	// ����Զ���Ԥ����Ҫ���������ѹ������ʽ����Ҫ
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
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
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

	// ����ص�ѹ
    volt = pdev->basic.bat_volt;
	LOG_DEBUG_APP_1("\r\nvolt:%f", volt);

	// �жϼ���Ƿ�ʱ
    if( pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] >= chgrMS_TO_CNT( TIMEOUT_CHK_IOUT_IN_STARTING ) )
	{
        pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] = 0;
	    LOG_WARNING_APP("%dms has been passed, chk iout failed in starting, close output, and chk bat conn",(uint16_t)TIMEOUT_CHK_IOUT_IN_STARTING );
	    // �ر����
	    close_output( pdev->basic.num );
		pdev->chgr.step = CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING;
		pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;

		return FALSE;
	}

	// �жϵ����Ƿ�����
	    // ��ȡ������
	ReadIout( pdev->basic.addr, &iout );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
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
			// ���õ�ǰͨ�����ڳ����
			pdev->basic.chgr = BASIC_CHGR_ING;
			/* ��������⴦�ڳ���е��ĸ�״̬
			 * �����Զ���磬ֱ�ӽ���cvm����
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
	
	// ���ڵ���
	// ��������ʽ����Ҫ���ڣ��Զ���Ԥ����Ҫ����
	if( pdev->chgr.mode == MODE_MANUAL  )
	{
		if( pdev->chgr.type == CHGR_TYPE_THREE_SECTION )
			return FALSE;
		else if( pdev->chgr.type == CHGR_TYPE_PRE )
			limit_iout = pdev->chgr.limit_iout;
	}
	else if( pdev->chgr.mode == MODE_AUTO  )
	{
        // ʹ�ö�ȡ�����Ƶ���
		limit_iout = pdev->chgr.auto_limit_iout; 
	}

	LOG_DEBUG_APP_1("\r\n<-----iout control----->");
	if( limit_iout - 5.5 >= 0 )  // ioc >= 5.5��ʹ��B46ָ���������Ƶ�������
	{
		LOG_DEBUG_APP_1("\r\nlimit >= 5.5, use B46");
	    // д����ѹֵ
	    if( pdev->chgr.mode == MODE_AUTO  )
			set_vout = (float)pdev->bat_info_bms.full_volt/10;
		else if( pdev->chgr.type == CHGR_TYPE_PRE )
            set_vout = pdev->chgr.para.data_pre.vbst;

        SetVout_St( pdev->basic.addr, set_vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif
	    LOG_DEBUG_APP_1("\r\nset_vout:%f ", set_vout );
        disp_pmbus_comm_status( st_pmbus );

	    // д������ֵ
	    LOG_DEBUG_APP_1("\r\nset limit_iout:%f ", limit_iout);
        SetIoutOC_FaultLimit_St( pdev->basic.addr, limit_iout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif
        disp_pmbus_comm_status( st_pmbus );
	}
	else
	{
		LOG_DEBUG_APP_1("\r\nlimit < 5.5, use mcu control");

        // ��ȡ����ѹ
        ReadVout_St( pdev->basic.addr, &vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
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
	
	// ��������ʱ
    pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] ++;
    LOG_DEBUG_APP_1("\r\ncnt:%u", pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ]);

	switch( pdev->chgr.step )
	{
		case CHGR_STEP_OPEN_OUTPUT:
	        // ��on/off�������Ҫ��ʱ1.5s������ᵼ��SetVoutʧ��
			if( pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] == chgrMS_TO_CNT( 200 ) )
			{
                start_success( pdev );
				break;
			}
			else if( pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] == chgrMS_TO_CNT( OPEN_ON_OFF_DELAY ) )
			{
	            // ��ģ�����
	            on_off_control( pdev->basic.num, ON );
				LOG_INFO_APP("enbale on/off");
				break;
			}
			else if( pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] >= chgrMS_TO_CNT(1500) )
			{
				if( ( pdev->chgr.mode == MODE_AUTO ) || ( pdev->chgr.type == CHGR_TYPE_PRE ) )
				{
					// ��������ֵ����ֹ��������ֵ��limit_iout <5.5��ʵ��ֵΪ5.5��
					SetIoutOC_FaultLimit_St( pdev->basic.addr, pdev->chgr.limit_iout, &st_pmbus );		
				}

				// �������������ֹӰ����һ���׶εļ��
	            pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] = 0;

				// ������������
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
		    // ����Ԥ��磬��ģʽ�л���PMBus����ģʽ
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
		// ����Ƿ����
		if( is_auto_chgr_bat_full( pdev ) )
	    {
	        LOG_DEBUG_APP("num:%d �Ѿ��������ر����", pdev->basic.num);

	        // �����󣬹ر����
	        close_output( pdev->basic.num );
            pdev->chgr.cnt_box[ IDX_CNT_OPEN_OUTPUT ] = 0;
	    	
	    	// ���ó�����
	    	pdev->basic.chgr = BASIC_CHGR_FINISH;
			pdev->basic.bat = BASIC_BAT_FULL;
	    	
	    	// �رպ����������ѹ���������Ϊ0
	    	pdev->chgr.vout = 0;
	    	pdev->chgr.iout = 0;

			// ���ò���Ϊcvm��﮵�س�磬ȫ�̵�ѹ���ֲ��䣬�������ﶨ��Ϊcvm
		    pdev->chgr.step = CHGR_STEP_CVM;
	    	
	    	return;
	    }

		if( !chgr_starting( pdev ) )
			return;
	}

	// ��ȡ��ص�ѹ
	volt = pdev->basic.bat_volt;
	LOG_DEBUG_APP_1("\r\nvolt:%f", volt);

	// �������쳣
	if( !chk_temperature( pdev ) )
		return;
		
	// ���������У�����Ƿ�Ͽ�
	if( is_vout_dispear_in_auto_mode( pdev ) )
	{
		LOG_DEBUG_APP("\r\n������ص�ѹ��ʧ���رճ�� ");
	    stop( pdev );
		return;
	}
	
	// �����
	if( pdev->basic.chgr == BASIC_CHGR_ING )
	{
		// ����糬ʱ
		// �Զ���磬4h��ʱ��ʱ����������翪ʼ��ʱ���������4h�������־��Ȼû����λ����Ϊ����
		if( pdev->chgr.time >= (uint32_t)TIMEOUT_MIN_AUTO_CHGR )
		{
			pdev->chgr.auto_flag_full_mcu = TRUE;
			LOG_DEBUG_APP_1("\r\n4h timeout, bms full_flag not set, then set chgr_station enter chgr finish");

		    // �����󣬹ر����
		    close_output( pdev->basic.num );
			
			// ���ó�����
			pdev->basic.chgr = BASIC_CHGR_FINISH;
			pdev->basic.bat = BASIC_BAT_FULL;
			
			// �رպ����������ѹ���������Ϊ0
			pdev->chgr.vout = 0;
			pdev->chgr.iout = 0;
	        pdev->chgr.iout_ori = 0;
	        pdev->chgr.iout_disp = 0;
	        pdev->chgr.auto_vout_step = 0;

			return;
		}

	    // �������쳣
	    if( !chk_dc_output( pdev ) )
	    	return;

		// �жϵ���Ƿ��Ѿ�����
		if( is_auto_chgr_bat_full( pdev ) )
		{
		    LOG_DEBUG_APP("num:%d �Ѿ��������ر����", pdev->basic.num);

		    // �����󣬹ر����
		    close_output( pdev->basic.num );
			
			// ���ó�����
			pdev->basic.chgr = BASIC_CHGR_FINISH;
			pdev->basic.bat = BASIC_BAT_FULL;
			
			// �رպ����������ѹ���������Ϊ0
			pdev->chgr.vout = 0;
			pdev->chgr.iout = 0;
	        pdev->chgr.iout_ori = 0;
	        pdev->chgr.iout_disp = 0;
	        pdev->chgr.auto_vout_step = 0;
			
			return;
		}

	    // ����������У��Ѿ���⵽�������Ϊ0�����
	    if( pdev->chgr.step == CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING )
	    {
			chk_bat_conn_in_charging( pdev );
	    	return;
	    }
	
		LOG_DEBUG_APP_1("\r\n\r\nnum:%d, limit iout:%f, bat volt:%f, bat_iout:%f", pdev->basic.num, pdev->chgr.auto_limit_iout, pdev->chgr.auto_bat_volt, pdev->chgr.auto_bat_iout);

		// ��ȡ������
		ReadIout( pdev->basic.addr, &iout );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif

		LOG_DEBUG_APP_1("\r\nread iout:%f", iout);
		if( (iout > pdev->chgr.data_max.ichg + 1) || ( iout < 0 ) ) { LOG_DEBUG_APP_1("\r\nread iout err, return"); return; }

		// д��iout queue��������ƽ��ֵ
		write_iout_to_queue( pdev, iout );
		calc_iout_ave( pdev );
		ave_iout = pdev->chgr.iout;
		LOG_DEBUG_APP_1("\r\nave_iout:%f", ave_iout);

		// ��ȡ����ѹ
		ReadVout_St( pdev->basic.addr, &vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif
		disp_pmbus_comm_status( st_pmbus );
		LOG_DEBUG_APP_1("\r\nread_vout:%f", vout );
		if( ( vout < 0 ) || ( vout - pdev->chgr.data_max.vbst > 1 ) )
		{
			LOG_DEBUG_APP_1("\r\nread vout err, return" );
			return;
		}
		
	    // ����������У��������Ϊ0�����������Ƿ�Ͽ��Ĳ���
        if( is_iout_disappear_in_charge( pdev, iout ) )
	    {
#if 0
			// 20190926��ӣ���������־û������������⵽����Ϊ0��Ȼ������
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

/************************* ���Ʋ��� ****************************/
		limit_iout = pdev->chgr.auto_limit_iout; // ʹ�ö�ȡ�����Ƶ���

		if( limit_iout - 5.5 >= 0 )  // ioc >= 5.5��ʹ��B46ָ���������Ƶ�������
		{
			LOG_DEBUG_APP_1("\r\nlimit >= 5.5, use B46");
			// д����ѹֵ
			set_vout = (float)pdev->bat_info_bms.full_volt/10;
			SetVout_St( pdev->basic.addr, set_vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif
			disp_pmbus_comm_status( st_pmbus );
			LOG_DEBUG_APP_1("\r\nset_vout:%f", set_vout );

			// д������ֵ
			LOG_DEBUG_APP_1("\r\nset limit_iout:%f", limit_iout);
			SetIoutOC_FaultLimit_St( pdev->basic.addr, limit_iout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif
			disp_pmbus_comm_status( st_pmbus );
//			ReadIoutOC_FaultLimit_St( pdev->basic.addr, &limit_read, &st_pmbus );
//			LOG_DEBUG_APP_1("\r\nread limit_iout:%f", limit_read);

		    // ���µ�ѹ��׼ֵ������������>5.5��������<5.5ʱ�����ڻ�׼ֵ���͵����������Ϊ0������
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

		// ���������ѹ�������������쳣��ѹ������
		if( ( vout <= pdev->chgr.data_max.vbst ) && ( vout >= 0 ) )
			pdev->chgr.vout = vout;
		
		if( ( iout <= pdev->chgr.data_max.ichg ) && ( iout >= 0 ) )
		{
			pdev->chgr.iout_ori = iout;
			
			// ������ʾ�ĵ���
			// iout��limit_iout��[-0.5, +0.5]����Χ�ڣ�����ʾlimit_iout
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
	else if( pdev->basic.chgr == BASIC_CHGR_FINISH ) // ������
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


// ���������Ϣ
	// ��ģ���¶�	
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

	// ������ת��
	pdev->basic.fan_speed = ReadFanSpeed_1( pdev->basic.addr );
	LOG_DEBUG_APP_1("\r\nfan_speed:%d\r\n\r\n", pdev->basic.fan_speed);
	
    // ����ģ��״̬
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

	// ��ȡ��ص�ѹ
	volt = pdev->basic.bat_volt;
	LOG_DEBUG_APP_1("\r\nvolt:%f\r\n", volt);

	// ����������У��Ѿ���⵽�������Ϊ0�����
	if( pdev->chgr.step == CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING )
	{
		chk_bat_conn_in_charging( pdev );
#if 0
		pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] ++;
		if( !isBatConn( volt ) )
		{
            pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;
			LOG_DEBUG_APP("\r\n����м���ضϿ��׶Σ����ȷʵ�Ͽ�");
			stop( pdev );
		}
		else if( pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] >= chgrMS_TO_CNT(4000) )
		{
            pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;

			if( volt >= (float)THRESHOLD_VOLT_IN_IOUT_DIS )
			{
				// ���ó�����
				pdev->basic.chgr = BASIC_CHGR_FINISH;
				pdev->basic.bat = BASIC_BAT_FULL;

				// ���µ�ػ�����Ϣ�У����õ�س�����־
				local_bat_info = get_local_bat_info();
				set_bit( local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_FULL );
				// �رպ����������ѹ���������Ϊ0
				pdev->chgr.vout = 0;
				pdev->chgr.iout = 0;

				pdev->chgr.step = CHGR_STEP_FVM;

				LOG_DEBUG_APP("\r\n����м���ضϿ��׶Σ����û�жϿ�����Ϊ���������븡��״̬");
			}
			else
			{
				LOG_DEBUG_APP("\r\nvolt:%f < %f", volt, (float)THRESHOLD_VOLT_IN_IOUT_DIS );
				LOG_DEBUG_APP_1(" �Ƿ�ֹͣ");
				stop( pdev );
				cache_event( pdev, EVENT_ILLEGAL_STOP );
			}
		}
#endif
		return;
	}

	// ֻ���ڳ������У���ȥ��ȡ�����͵�ѹ
	if( pdev->basic.chgr != BASIC_CHGR_FINISH )
	{
	    ReadVout( pdev->basic.addr, &vout );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif

	    ReadIout( pdev->basic.addr, &iout );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif
        if( (iout - 28.5 > 0 ) || ( iout < 0 ) ) 
        { 
	        LOG_DEBUG_APP_1("\r\nerror data, use pre iout->%f", pdev->chgr.iout_pre); 
	    	iout = pdev->chgr.iout_pre; //  ʹ���ϴεĵ���
	    }
	    pdev->chgr.iout_pre = iout;

	    // ���������ѹ�������������쳣��ѹ������
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

	// ����������У��������Ϊ0�����������Ƿ�Ͽ��Ĳ���
	if( is_iout_disappear_in_charge( pdev, iout ) )
	{
		LOG_DEBUG_APP_1("\r\nģ��%d�������Ϊ0���ر������������һ��", pdev->basic.num);

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
	

	// �������쳣
	if( !chk_dc_output( pdev ) )
		return;

	// �������쳣
	if( !chk_temperature( pdev ) )
		return;
	
	// ��ģ���¶�	
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

	// ������ת��
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

			LOG_DEBUG_APP("num:%d �Ѿ��������ر����", pdev->basic.num);

			// ���µ�ػ�����Ϣ�У����õ�س�����־
			local_bat_info = get_local_bat_info();
			set_bit( local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_FULL );

			// ���븡��󣬹ر�ģ�����
            close_output( pdev->basic.num );

			// һֱ�������û�жϿ�
			pdev->chgr.step = CHGR_STEP_CHK_BAT_DISCONN_IN_FLOATING;
			break;
		case CHGR_STEP_IDLE:
			break;
		case CHGR_STEP_CHK_BAT_DISCONN_IN_FLOATING:
			LOG_DEBUG_APP_1("\r\nstep:chk_bat_dis_in_floating");
			// �ر�����󣬼�����Ƿ�����
			// ������û�����ӣ�ֹͣ���
			if( !isBatConn( volt ) )
			{
			    LOG_DEBUG_APP("\r\n�����м��������״̬�����ȷʵ�Ͽ�");
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

	// ��ȡ��ص�ѹ
	volt = pdev->basic.bat_volt;
	LOG_DEBUG_APP_1("\r\nvolt:%f", volt);

	// ����������У��Ѿ���⵽�������Ϊ0�����
	if( pdev->chgr.step == CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING )
	{
		chk_bat_conn_in_charging( pdev );
#if 0
		pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] ++;
		if( !isBatConn( volt ) )
		{
            pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;
			LOG_DEBUG_APP("\r\n����м���ضϿ��׶Σ����ȷʵ�Ͽ�");
			stop( pdev );
		}
		else if( pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] >= chgrMS_TO_CNT(4000) )
		{
            pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;

			if( volt >= (float)THRESHOLD_VOLT_IN_IOUT_DIS )
			{
				// ���ó�����
				pdev->basic.chgr = BASIC_CHGR_FINISH;
				pdev->basic.bat = BASIC_BAT_FULL;

				// ���µ�ػ�����Ϣ�У����õ�س�����־
				local_bat_info = get_local_bat_info();
				set_bit( local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_FULL );
				// �رպ����������ѹ���������Ϊ0
				pdev->chgr.vout = 0;
				pdev->chgr.iout = 0;

				pdev->chgr.step = CHGR_STEP_FVM;

				LOG_DEBUG_APP("\r\n����м���ضϿ��׶Σ����û�жϿ�����Ϊ���������븡��״̬");
			}
			else
			{
				LOG_DEBUG_APP("\r\nvolt:%f < %f", volt, (float)THRESHOLD_VOLT_IN_IOUT_DIS );
				LOG_DEBUG_APP_1(" �Ƿ�ֹͣ");
				stop( pdev );
				cache_event( pdev, EVENT_ILLEGAL_STOP );
			}
		}
#endif

		return;
	}

	// ��ģ���¶�	
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

	// ������ת��
	pdev->basic.fan_speed = ReadFanSpeed_1( pdev->basic.addr );

    // ����ģ��״̬
	ReadChgStatus( pdev->basic.addr, &status );
	pdev->chgr.status = status;

	// �������쳣
	if( !chk_temperature( pdev ) )
		return;

/**********************************************************/
	// ���븡���ֹͣ��ȡ
	if( pdev->chgr.step == CHGR_STEP_FVM )
	{
		// ������û�����ӣ�ֹͣ���
		if( !isBatConn( volt ) )
		{
		    LOG_DEBUG_APP_1("\r\n�����м��������״̬����ضϿ�");
			stop( pdev );
			return;
		}

		return; // ������������
	}

	// �������쳣
	if( !chk_dc_output( pdev ) )
		return;

	// ��ȡ����ѹ
	ReadVout_St( pdev->basic.addr, &vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif
	LOG_DEBUG_APP_1("\r\nread_vout:%f ", vout );
    disp_pmbus_comm_status( st_pmbus );
	if( ( vout < 0 ) || ( vout - pdev->chgr.data_max.vbst > 1 ) )
	{
		LOG_DEBUG_APP_1("\r\nvour err, return" );
		return;
	}

	// ��ȡ������
    ReadIout_St( pdev->basic.addr, &iout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif
	LOG_DEBUG_APP_1("\r\nread_iout:%f ", iout );
    disp_pmbus_comm_status( st_pmbus );

	if( st_pmbus != SMBUS_OK )
	{
		LOG_DEBUG_APP_1("\r\nsmbus err, use pre iout->%f", pdev->chgr.iout_pre);
		iout = pdev->chgr.iout_pre; //  ʹ���ϴεĵ���
	}
	
    if( (iout - 28.5 > 0 ) || ( iout < 0 ) ) 
    { 
	    LOG_DEBUG_APP_1("\r\nerror data, use pre iout->%f", pdev->chgr.iout_pre); 
		iout = pdev->chgr.iout_pre; //  ʹ���ϴεĵ���
	}
	pdev->chgr.iout_pre = iout;

	// д��iout queue��������ƽ��ֵ
    write_iout_to_queue( pdev, iout );
    calc_iout_ave( pdev );
    ave_iout = pdev->chgr.iout;
	LOG_DEBUG_APP_1("\r\nave_iout:%f", ave_iout);

	// ����������У��������Ϊ0�����������Ƿ�Ͽ��Ĳ���
    if( is_iout_disappear_in_charge( pdev, iout ) )
	{
		LOG_DEBUG_APP_1("\r\nģ��%d�������Ϊ0���ر������������һ��", pdev->basic.num);

        close_output( pdev->basic.num );
		pdev->chgr.step = CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING;
		return;
	}

	switch( pdev->chgr.step )
	{
		case CHGR_STEP_CHECK_STAGE:
			LOG_DEBUG_APP_1("... check step ...");
			// С��Ԥ���ѹ����Ҫ����Ԥ��
			if( volt < pdev->chgr.para.data_pre.pre_vout )
			{
				pdev->chgr.cnt_box[ IDX_CNT_IS_PRE_CHGR ] ++;
			}
			else
				pdev->chgr.cnt_box[ IDX_CNT_IS_PRE_CHGR ] --;

			// ����3�ε�ѹС��Ԥ���ѹ������Ԥ�䲽��
			if( pdev->chgr.cnt_box[ IDX_CNT_IS_PRE_CHGR ] >= 3 )
			{
				pdev->chgr.step = CHGR_STEP_PRE;
				pdev->basic.chgr = BASIC_CHGR_ING;

				// �����ضϿ���⣬��ֹ����
				pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;

				LOG_DEBUG_APP_1("\r\nnum:%d ��ص�ѹ:%f��С��Ԥ���ѹ:%f������Ԥ��׶�", pdev->basic.num, volt, pdev->chgr.para.data_pre.pre_vout);

			}
			else if ( pdev->chgr.cnt_box[ IDX_CNT_IS_PRE_CHGR ] <= ( 3 * (-1) ) )
			{
				pdev->chgr.step = CHGR_STEP_CCM;
				pdev->basic.chgr = BASIC_CHGR_ING;

				// �����ضϿ���⣬��ֹ����
				pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;

				LOG_DEBUG_APP_1("\r\nnum:%d ��ص�ѹ:%f������Ԥ���ѹ:%f����������׶�", pdev->basic.num, volt, pdev->chgr.para.data_pre.pre_vout);

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
		        LOG_DEBUG_APP_1("num:%d ��ص�ѹ:%f�����ں���ѹ:%f����������׶�", pdev->basic.num, volt, pdev->chgr.para.data_pre.pre_vout);

				// �����ضϿ���⣬��ֹ����
				pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;

			    // �������Ƶ���
                pdev->chgr.limit_iout = pdev->chgr.para.data_pre.pre_iout;

				return;
			}

			// �������Ƶ���
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
		        LOG_DEBUG_APP_1("num:%d ��ص�ѹ:%f�����ڵ��ں���ѹ:%f - 0.5�������ѹ�׶�", pdev->basic.num, volt, pdev->chgr.para.data_pre.vbst);
				
				// �����ضϿ���⣬��ֹ����
				pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;

			    // �������Ƶ���
                pdev->chgr.limit_iout = pdev->chgr.para.data_pre.ichg;

				return;
			}

			// �������Ƶ���
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

		        LOG_DEBUG_APP_1("num:%d ������:%f��С����ֹ����:%f�����븡��׶�", pdev->basic.num, iout, pdev->chgr.para.data_pre.itaper);
				// ��ʾ����
		        LOG_DEBUG_APP_1("\r\nnum:%d �Ѿ��������ر����", pdev->basic.num);

			    // ���µ�ػ�����Ϣ�У����õ�س�����־
			    local_bat_info = get_local_bat_info();
			    set_bit( local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_FULL );

		        // �����󣬹ر����
		        close_output( pdev->basic.num );
			    
			    // ���ó�����
			    pdev->basic.chgr = BASIC_CHGR_FINISH;
				pdev->basic.bat = BASIC_BAT_FULL;
			    
			    // �رպ����������ѹ���������Ϊ0
			    pdev->chgr.vout = 0;
			    pdev->chgr.iout = 0;

				// �����ضϿ���⣬��ֹ����
				pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;

			    // �������Ƶ���
                pdev->chgr.limit_iout = pdev->chgr.para.data_pre.ichg;
				
				return;
			}

			// �������Ƶ���
            pdev->chgr.limit_iout = pdev->chgr.para.data_pre.ichg;
		    break;
		default:
		break;
	}

/************************* ���Ʋ��� ****************************/
	LOG_DEBUG_APP_1("\r\n<-----iout control----->");
	limit_iout = pdev->chgr.limit_iout;

	if( limit_iout - 5.5 >= 0 )  // ioc >= 5.5��ʹ��B46ָ���������Ƶ�������
	{
		LOG_DEBUG_APP_1("\r\nlimit >= 5.5, use B46");
	    // д����ѹֵ
        set_vout = pdev->chgr.para.data_pre.vbst;
        SetVout_St( pdev->basic.addr, set_vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif
	    LOG_DEBUG_APP_1("\r\nset_vout:%f ", set_vout );
        disp_pmbus_comm_status( st_pmbus );

	    // д������ֵ
	    LOG_DEBUG_APP_1("\r\nset limit_iout:%f ", limit_iout);
        SetIoutOC_FaultLimit_St( pdev->basic.addr, limit_iout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif
        disp_pmbus_comm_status( st_pmbus );
        ReadIoutOC_FaultLimit_St( pdev->basic.addr, &limit_read, &st_pmbus );
	    LOG_DEBUG_APP_1("  read limit_iout:%f ", limit_read);

		// ���µ�ѹ��׼ֵ������������>5.5��������<5.5ʱ�����ڻ�׼ֵ���͵����������Ϊ0������
   	    pdev->chgr.volt_base = volt;
		pdev->chgr.auto_vout_step = 0;
	}
	else
	{
		LOG_DEBUG_APP_1("\r\nlimit < 5.5, use mcu control");
//        LOG_DEBUG_APP_1("\r\niout: 1:%f, 2:%f, 3:%f, 4:%f", pdev->chgr.iout_queue.buff[0], pdev->chgr.iout_queue.buff[1], pdev->chgr.iout_queue.buff[2], pdev->chgr.iout_queue.buff[3]);
		chgr_control_little_iout( pdev, vout, iout, limit_iout );
	}

	// ���������ѹ�������������쳣��ѹ������
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
			// ������ʾ�ĵ���
			// iout��limit_iout��[-0.5, +0.5]����Χ�ڣ�����ʾlimit_iout
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

	// ��ֹset_voutΪ0
	set_vout = pdev->basic.bat_volt;

	/****************************   ���ƹ���    *******************************/
	LOG_DEBUG_APP_1("\r\nnum:%d iout limit:%f", pdev->basic.num, limit_iout);

	/* 20191015������Զ����ģʽ���������е����⣬�������·���:
	 * ����������С��2A������2A����
	 *
	 * ʵ���ϣ�ͨ������ѹ������������ķ�ʽ����С�������ֻ����2A���������ﲻ�����ֶ�ģʽ�����Զ�ģʽ����������С2A����
	 */
	if( limit_iout < (float)2 )
	{
		LOG_DEBUG_APP_1("\r\nlimit_iout < 2A, set limit_iout = 2A");
		limit_iout = (float)2;
	}

	// ����
	// ����������������Ϊ0.5
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
	// �ж�����ѹֵ�ķ�Χ
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

	// д�����õĵ�ѹֵ
	LOG_DEBUG_APP_1("\r\nset vout:%f, vbst:%f, volt_base:%f, vout_add_step:%f ", set_vout, pdev->chgr.para.data_pre.vbst, pdev->chgr.volt_base, pdev->chgr.auto_vout_step);
	SetVout_St( pdev->basic.addr, set_vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
#endif
	disp_pmbus_comm_status( st_pmbus );
}

static void chgr_control_little_iout_in_starting( PU_InfoDef *pdev, float vout, float iout, float limit_iout )
{
	float set_vout = 0;
	StatusTypeDef st_pmbus;
	float max_set_vout = 0;

	// ��ֹset_voutΪ0
	set_vout = pdev->basic.bat_volt;

	/****************************   ���ƹ���    *******************************/
	LOG_DEBUG_APP_1("\r\nnum:%d iout limit:%f", pdev->basic.num, limit_iout);

	// ����
	// ����������������Ϊ0.5
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

	// �ж�����ѹֵ�ķ�Χ
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

	// д�����õĵ�ѹֵ
	LOG_DEBUG_APP_1("\r\nset vout:%f, vbst:%f, volt_base:%f, vout_add_step:%f ", set_vout, pdev->chgr.para.data_pre.vbst, pdev->chgr.volt_base, pdev->chgr.auto_vout_step);
	SetVout_St( pdev->basic.addr, set_vout, &st_pmbus );
#ifndef __LOG
	    vTaskDelay( PMBUS_COMM_MS_DELAY ); // ����release�汾������log��Ϣ��pmbusͨ����Ҫ�����ʱ
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
		LOG_DEBUG_APP_1("\r\n����ضϿ��׶Σ����ȷʵ�Ͽ�");
		stop( pdev );
	}
	else if( pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] >= chgrMS_TO_CNT(4000) )
	{
		pdev->chgr.cnt_box[ IDX_CNT_CHK_BAT_DISCONN_IN_CHGRING ] = 0;

		// ������û�жϿ����жϵ�ص�ѹ�������ص�ѹ>=ָ����ѹֵ��THRESHOLD_VOLT_IN_IOUT_DIS��������Ϊ����������ֹͣ�����Ƿ�ֹͣ�¼� 
		if( volt >= (float)THRESHOLD_VOLT_IN_IOUT_DIS )
		{
			LOG_DEBUG_APP("\r\nvolt:%f", volt);
			
			// ���ó�����
			pdev->basic.chgr = BASIC_CHGR_FINISH;
			pdev->basic.bat = BASIC_BAT_FULL;

			// ���µ�ػ�����Ϣ�У����õ�س�����־
//			local_bat_info = get_local_bat_info();
//			set_bit( local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_FULL );
			// �رպ����������ѹ���������Ϊ0
			pdev->chgr.vout = 0;
			pdev->chgr.iout = 0;

			pdev->chgr.step = CHGR_STEP_FVM;
			
			if( pdev->chgr.mode == MODE_AUTO )
			{
				// mcu������Ƿ����
				pdev->chgr.auto_flag_full_mcu = TRUE;
				// �����ضϿ���⣬��ֹ����
				pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] = 0;
			}

			LOG_DEBUG_APP_1("\r\n����ضϿ��׶Σ����û�жϿ�����Ϊ���������븡��״̬");
		}
		else
		{
			LOG_DEBUG_APP_1("\r\nvolt:%f < %f���Ƿ�ֹͣ", volt, (float)THRESHOLD_VOLT_IN_IOUT_DIS );
			stop( pdev );
			cache_event( pdev, EVENT_ILLEGAL_STOP );
			
			// ���þ����־
			set_bit( pdev->basic.warn, WARN_ILLEGAL_STOP );
		}
	}
}

// pi control
#if 0
/*
 * wRTrackVolt - limit_iout
 * wRKVoltrack1, wRKVoltrack2, - pi����
 * wRInverterVoltNew , ��ȡ�ķ�������ֵ
 * wRNewSinAmp����ѹ���ڻ�׼ֵ
 * wRVoltError0�� ��һ�ε�ƫ��
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

	/****************************   ���ƹ���    *******************************/
	LOG_DEBUG_APP_1("\r\nnum:%d iout limit:%f", pdev->basic.num, limit_iout);
	LOG_DEBUG_APP_1("\r\nset_vout_before:%f, iout_err_before:%f", pdev->chgr.set_vout, pdev->chgr.iout_deviation_before);

    set_vout = pdev->chgr.set_vout;
    iout_deviation_before = pdev->chgr.iout_deviation_before;
	
	volt = pdev->basic.bat_volt;

    sRVoltRegu( limit_iout, 0.06, 0.05, &iout_deviation_before, iout, &set_vout, volt );

	pdev->chgr.set_vout = set_vout;
    pdev->chgr.iout_deviation_before = iout_deviation_before;

	LOG_DEBUG_APP_1("\r\nset_vout_current:%f, iout_err_current:%f", pdev->chgr.set_vout, pdev->chgr.iout_deviation_before);

	// д�����õĵ�ѹֵ
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
		// ���±��ص����Ϣ�����У����δ������Ϣ
	    reset_bit(local_bat_info[ pdev->basic.num * SIZE_SINGLE_BAT_INFO ], IDX_BIT_IS_BAT_CONN);

		// ���õ��δ����
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
	// �������쳣 ��������׶κͼ������׶Σ������
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
		LOG_DEBUG_APP_1("\r\nnum:%d����쳣", pdev->basic.num);

		if( ( pdev->chgr.op != op_start ) && ( pdev->chgr.step == CHGR_STEP_OPEN_OUTPUT ) ) // �����û�������ɹ�
		{
			start_failed( pdev );
		}
		else
		{
			stop( pdev );
		}

		// ���ô����־
		set_bit( pdev->basic.err, ERR_DC_OK );

		// �������쳣�¼�
		cache_event( pdev, EVENT_DC_OK_ERR );

		return FALSE;
	}
	else
	{
		// ��������־
		reset_bit( pdev->basic.err, ERR_DC_OK );
	}

	return TRUE;
}

static BOOL chk_temperature( PU_InfoDef *pdev )
{
	// ������, ������£�ֱ��stop��
	if( is_t_alarm_in_charge( pdev ) )
	{

	    if( ( pdev->chgr.op != op_start ) && ( pdev->chgr.step == CHGR_STEP_OPEN_OUTPUT ) ) // �����û�������ɹ�
            start_failed( pdev );
		else
		    stop( pdev );

		// ���ô����־
		set_bit( pdev->basic.err, ERR_T_ALARM );

		cache_event( pdev, EVENT_T_ALARM );
		return FALSE;
	}
	else
	{
		// ��������־
		reset_bit( pdev->basic.err, ERR_T_ALARM );
	}

	return TRUE;
}

static void open_output( uint8_t num )
{
	// ��ģ���AC����
	power_on( num );

    // ��dc����̵���
    open_dc_relay( num );

	// ��ģ�����
//	on_off_control( num, ON );
}

static void close_output( uint8_t num )
{
	// �ر�on/off���
	on_off_control( num, OFF );

	// �ٹر�dc����̵���
    close_dc_relay( num );
}

static BOOL is_iout_disappear_in_charge( PU_InfoDef *pdev, float iout )
{
	// �������OpenOutPut��״̬�������
	if( pdev->chgr.step == CHGR_STEP_OPEN_OUTPUT ) return FALSE;

	// �������CHGR_STEP_OPEN_OUTPUT��״̬�������
	if( pdev->chgr.step == CHGR_STEP_CHECK_OUTPUT ) return FALSE;
	
	// ������ڸ���״̬�������
	if( pdev->chgr.step == CHGR_STEP_FVM ) return FALSE;

	// ������ڼ�����Ƿ�Ͽ��Ĳ��裬�����
	if( pdev->chgr.step == CHGR_STEP_CHK_BAT_DISCONN_IN_CHGRING ) return FALSE;

	// ����Ѿ������ˣ������
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
	else // ��ȡ��������˵��ͨ�������⣬���յ���Ϊ0����
		pdev->chgr.cnt_box[ IDX_CNT_BAT_DISCONN ] ++;
	
	// process result
	/*
	 * ����10�μ�⵽�������Ϊ0֮�󣬲������²���������Ƿ���ĶϿ�
	 * 1. �Ͽ�ģ�����(on/off)
	 * 2. ��ʱ5s
	 * 3. ʹ��AD������ص�ѹ��������������δ���ӣ���ֹͣ��磬������������ӣ���ô��ģ�����(on/off)
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
	
	// �������OpenOutPut��״̬�������
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

	// ����ʾ��ͣ����
    xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_EMERGENCY_STOP, eSetBits );

	// ����������
	suspend_charge_damon();

	// �ر��������ڳ���ģ������
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

	// �л���֮ǰ�Ľ���
    xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_START_FROM_EMERGENCY_STOP, eSetBits );
	
	// ������֮ǰ���ڳ���ģ������
	pnode = pchgr_list->next;
	while( pnode != NULL )
	{
        open_output( pnode->pdev->basic.num );	
	    pnode = pnode->next;
	}

	// �ָ��������
	resume_charge_damon();
}

static BOOL isBatReverse( float volt )
{

	// �ж��Ƿ񷴽�
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
	// �жϵ���Ƿ�����, 20v - 60v
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
	 * ���� FILTER_DC_OK ����Ϊ�͵�ƽ��˵������쳣 
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
	    LOG_DEBUG_FILE_APP("\r\nϵͳ��ǰ�ڴ��СΪ��%ld �ֽ�", mem_size);
	}

	if( NULL == (phead = (ChgrListDef *)pvPortMalloc( sizeof(DevListDef)) ) ) { LOG_DEBUG_FILE_APP("\r\nmalloc failed"); return NULL; }

	if( __LOG_DEBUG_APP )
	{
	    mem_size = xPortGetFreeHeapSize();
	    LOG_DEBUG_FILE_APP("\r\nϵͳʣ���ڴ��СΪ��%ld �ֽ�", mem_size );
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
	    LOG_TRACE("\r\nϵͳ��ǰ�ڴ��СΪ��%ld �ֽ�", mem_size);
	}

	// ���������һ���ڵ�
	while( pnode->next != NULL ) 
		pnode = pnode->next;
		
	// �����½ڵ�
	if( NULL == ( pnew = (ChgrListDef *)pvPortMalloc( sizeof( ChgrListDef ) ) ) )
	{
		LOG_TRACE_1("\r\nadd to chgr list failed, num:%d", pdev->basic.num);
		return;
	}
	
	// �Ѵ�������豸��ַ���ŵ��½ڵ�����
	pnew->pdev = pdev;
	
	// ���½ڵ���뵽������
	pnew->next = pnode->next;
	pnode->next = pnew;
	
	LOG_DEBUG_APP_1("\r\nadd_to_chgr_lis success, num:%d", pnew->pdev->basic.num );

	if( __LOG_DEBUG_APP )
	{
	    mem_size = xPortGetFreeHeapSize();
	    LOG_TRACE_1("\r\nϵͳʣ���ڴ��СΪ��%ld �ֽ�", mem_size );
	}
}

static void remove_from_chgr_list( const uint8_t num )
{
    ChgrListDef *p_pre = NULL;
	ChgrListDef *p_remove = NULL;
	uint32_t mem_size;	

	// ������ŵ���num�Ľڵ㣬����remove�Ľڵ�
	p_pre = pchgr_list;
	while( p_pre->next != NULL )
	{
		// �ҵ�֮�󣬰Ѵ�remove�Ľڵ㱣��һ�£�Ȼ��Ѵ�remove�Ľڵ���������Ƴ�
		if( p_pre->next->pdev->basic.num == num )
		{
		    p_remove = p_pre->next;
			p_pre->next = p_remove->next;
			
			// �ͷŴ�remove�ڵ�Ŀռ�
            vPortFree( p_remove );
			
	        if( __LOG_DEBUG_APP )
	        {
	            mem_size = xPortGetFreeHeapSize();
	            LOG_TRACE("\r\nremove��ϵͳʣ���ڴ��СΪ��%u �ֽ�", mem_size );
	        }

			return;
		}
		
	    p_pre = p_pre->next;
	}
}

static BOOL chk_before_start_in_manual_mode( PU_InfoDef *pdev )
{
	if( pdev->chgr.mode == MODE_AUTO ) { LOG_DEBUG_APP("\r\nnum:%d, when start, mode = auto", pdev->basic.num ); return FALSE; } // �����ǰ���Զ�ģʽ��ֱ������
	if( pdev->basic.flag_bms_conn == ON ) { LOG_DEBUG_APP("\r\nnum:%d, when start, bms conn", pdev->basic.num); return FALSE; } // ���bms���ӣ�ֱ������
	if( ( pdev->basic.chgr == BASIC_CHGR_OFF ) &&( pdev->chgr.flag_full == TRUE ) ) { LOG_DEBUG_APP("\r\nnum:%d, when start, flag_full = true", pdev->basic.num); return FALSE; } // �������ʱ�رգ�����
    if( pdev->basic.flag_mode_switch != BASIC_MODE_NONE_SWITCH ) { LOG_DEBUG_APP("\r\nnum:%d, when start, in mode switch, return", pdev->basic.num);  return FALSE; }

	// �жϵ������״̬
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

	// �ж�bms�Ƿ�����
	if( pdev->basic.flag_bms_conn == ON )
	{
		start_failed( pdev );
	    return FALSE;
	}

    // �����ִ����������������
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
		// ��ʾ��ֹ����
        xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_NO_START, eSetBits );
		LOG_DEBUG_APP_1("\r\nnum:%d not start", pdev->basic.num);
		return;
	}

	// ִ��start����
	start( pdev );
	
	// ����Ļ���Ϳ����������źţ���ʾ������
    xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_TO_START, eSetBits );

    // ֱ����ʾ�����ɹ�
//    start_success( pdev );
}

static void start_by_booking( PU_InfoDef *pdev )
{
    if( !chk_before_start_in_manual_mode( pdev ) )
		return;

	// ִ��start����
	start( pdev );

    // ֱ����ʾ�����ɹ�
//    start_success( pdev );
}

static void start_by_bms( PU_InfoDef *pdev )
{
	assert_app( (pdev->basic.num >= POWER_UNIT_NUM) );

	// �жϸ�ͨ���Ƿ��ڱ����ر�״̬������ǣ�������
	if( pdev->basic.chgr == BASIC_CHGR_STOP_BY_HOST )
	{
	    LOG_DEBUG_APP_1("\r\nstart num:%d, in stoy by host status, return", pdev->basic.num);
	}

	// �жϵ������״̬
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

		// ��ʼ������б�
		if( NULL == (pchgr_list = init_chgr_list())) { printf("\r\ninit chgr list failed"); return; }

		// ��ȡ�豸�б�
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
		// ����������
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

	// ���ÿ�����־λ
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

	// ���ùػ���־λ
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

	        // ��ʾֹͣ����ʱ
	        pdev->chgr.flag_stop = TRUE;

			// ����Ļ����ֹͣ�ź�
		    xTaskNotify( h_touch_screen_entry, CHGR_NOTIFY_TO_STOP, eSetBits);

			// ִ��ֹͣ����
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

			// ִ��ֹͣ����
			LOG_DEBUG_APP_1("...exec stop operation");
            stop( pdev );
			break;
		case STOP_TYPE_BMS:
            LOG_DEBUG_APP_1(".....stop type->bms,");

			// ֱ��ֹͣ
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

			// ֱ��ֹͣ
			LOG_DEBUG_APP_1("...exec stop operation");
            stop( pdev );

			// ����ģ���״̬Ϊ����ֹͣ״̬
			    // ������Զ���磬������ֶ����ģʽ������ʶ�ñ�־λ
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

    // ���͸�touch_screen_entry֪ͨ����ʾ�л���
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
	// ���������
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
    // �ָ�������
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

