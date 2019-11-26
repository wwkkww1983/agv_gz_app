/************************************************************************
note.
20181219��������ݽ���ģ��Ŀ���
20181220�����ϵͳ���ݽṹ����ƣ������ҳ����ҳ��ϸҳ����ʾ�Ŀ���
20181221, �����ҳ����ϸҳ��ĵ���
          �Ż����ݽṹ��ƣ������е��豸ʹ������������������ڲ������������
		  ���5��ģ��ĸ�����ʾ����ϸ��ʾ������
20181226, ���������ֹͣ���л�
        , ������ã�����ʽ�������
20181227, �޸�Ĭ�ϲ���Ϊ��RCB1600-48�Ĳ���
20190109, ��Ӽ�ͣ��ʾ
20190114, ͬ�����õ���ͨ��
          SET_NOTIFY_TO_RESTART_ALL, ->touch_screen_entry->charge_entry
          SET_NOTIFY_RESTART_FINISHED, restart_dev_all -> touch_screen_entry
20190115, ��ӳ��ʱ�����ù��ܣ�����ͨ��
          ���ԤԼ������ù��ܣ�����ͨ��

		  ���ģ��6-8����ҳ��ϸ��Ϣˢ��
		  ���ģ��6-8����ҳ������Ϣˢ��
		  ���ģ��6-8�ĳ������
20190118, ���ά�����ܣ�������ɣ���������ͨ�� 
20190124, �����ʷ�¼���ѯ����
          - �¼���ѯ
20190216, ���"���δ����"�¼���ʾ
20190222, �޸Ļ�ȡ��Ļʱ�����
          - ���5s����һ�λ�ȡʱ��ָ��ѻ�ȡ����ʱ��洢��phmi_info����
		  - ���ģ���phmi_info�����ȡʱ��

������
1. �ǵ���Ӳ������õ���������!
2. ģʽ�л��Ѿ����Σ�������Ҫ�ǵô�
3. �Ѿ�����6,7,8ģ�����ϸ��Ϣˢ�£�������Ҫ�ǵô� - 20190115�Ѿ�������ģ�������ˢ��
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


#define    HMI_MAX_PAGE_ID    184  // ����汾20191101����Ϊ184

#define    HMI_SCREEN_SAVE_PAGE_ID 0
#define    HMI_MAIN_PAGE_ID        1
#define    HMI_SET_PAGE_ID         10
#define    HMI_EVENT_PAGE_ID       46
#define    HMI_MAINTAIN_PAGE_ID    64
#define    HMI_HELP_PAGE_ID        75

#define    HMI_NO_START_PAGE_ID    96
#define    HMI_NO_SYN_SETTING_PAGE_ID    97
#define    HMI_NO_SETTING_PAGE_ID    98

// ������Ҫ����ˢ��ҳ���ID
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


#define    FLAG_BENN_INIT            0x01  // �Ѿ���ʼ���ı�־
#define    EE_ADDR_BEEN_INIT_FLAG    0     // ����Ѿ���ʼ����־�ĵ�ַ 

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

#define    EE_OFFSET_CHGR_THRESHOLD    37 // EE_CHGR_PARA_ADDR + ��ƫ�����Ǵ����ֵ�ĵ�ַ

#define    EE_OFFSET_SMP_CALIBRATED    49 // �洢�Ƿ�У׼����־λ��ƫ�Ƶ�ַ
#define    EE_OFFSET_SMP_CALIBRATE_VAL 50 // �洢У׼��ֵ
#define    LEN_SMP_CALIBRATE_VAL       2  // У׼ֵ����


/**********************************************************
 ��ǰ�ļ���ȫ�ֱ�������
**********************************************************/
// �����������֡�洢����
static HMI_RxFrameDef hmi_rx_frame;
static HMI_RxFrameDef *phmi_rx_frame = &hmi_rx_frame;
// �洢��Ļ��Ϣ�ı���
static HMI_InfoDef hmi_info;
static HMI_InfoDef *phmi_info = &hmi_info;
// �洢����ģ����Ϣ�ı���
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
"����",
"ֹͣ",
"��ط���",
"������ʹ���",
"����쳣",
"�¶��쳣",
"���δ����",
"��ѹ",
"�Ƿ�ֹͣ"
};
const static char *Event_chgr_mode_tab[] = 
{
"�Զ�",
"�ֶ�"
};
const static char *Event_chgr_para_tab[] = 
{
"һ",
"��",
"��",
"��",
"��"
};



// ��Ӧ��ά�±�
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

/* �������ֵ����Сֵ��[0] - ���ֵ��[1]-��Сֵ
 * ���У�max_itaper <= max_ichg, max_vfloat <= max_vbst
 *       min_itaper <= min_ichg, min_vfloat <= min_vbst
 * �ڲ�������ʱ����Ҫע�⼰ʱ���ж�
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
// Ԥ������
const float default_pre_chgr_para_data[] = { 3, 48, 27.5, 57.6, 1.5 };
const float pre_chgr_data_mm[][5] = 
{
{10, 50, 27.5, 60, 5},
{1,  32, 1,    48, 1}
};

/**********************************************************
 ��ǰ�ļ���˽�к���
**********************************************************/
// ������Ļ���յ�����֡�������������жϣ�ִ�е�
static void process_hmi_rx_frame(void);
// ��������
static BOOL frame_parse(void);
// �򵥵�����֡���
static BOOL frame_chk(void);

// ��ʼ�����
static void init_device_list(void);
static void init_power_unit_info(void);

// for debug
static void dbg_print_hmi_frame(void);


/**********
  ����+����
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
  ��ҳ
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
 * ��ҳ��ʾ��صĺ���
 */
// ��ʾ���ģʽ���Զ�/�ֶ�
static void main_disp_mode( uint8_t to, uint8_t num );
// ��ʾ������������/ֹͣ
static void main_disp_op( uint8_t to, uint8_t num );
// ��ʾ���ͼ��״̬
static void main_disp_bat( uint8_t to, uint8_t num );
// ��ʾָʾ�Ƶ�״̬
static void main_disp_led( uint8_t to, uint8_t num );
// ��ʾ�����ѹ
static void main_disp_vout( float fvalue, uint8_t num );
// ��ʾ��ص�ѹ
static void main_disp_bat_volt( float fvalue, uint8_t num );
// ��ʾ�������
static void main_disp_iout( float fvalue, uint8_t num );
// ��ʾ���״̬���쳣/����
static void main_disp_chgr_status( uint8_t st, uint8_t num );
// ��ʾ���ʱ��
static void main_disp_chgr_time( uint16_t time, uint8_t num );
// ��ʾģ���¶ȣ�-128 - 127
static void main_disp_temperature( int8_t time, uint8_t num );
// ��ʾ�ѳ����
static void main_disp_AH( uint16_t cap, uint8_t num );
// ��ʾ�����ѹ����
static void main_disp_DC_OK( uint8_t dc_ok, uint8_t num );
// ��ʾ�������
static void main_disp_starting( void );
static void main_disp_start_success( void );
static void main_disp_start_failed( void );

static void main_disp_chgr_type_switch(void);
static void main_disp_chgr_type_switch_finish(void);

/**********
  ����
**********/
static void hmi_set( void );

// ��������
static void set_enter( void );
static void set_update_tmp_input_passwd( void );
static void set_read_passwd( uint8_t *passwd ); 
static void set_save_passwd( uint8_t *passwd );

// �������
  // ����ʽ�������
static void set_disp_chgr_set( uint8_t num );
static void set_disp_curve_data( CHGR_ParaDataDef *pdata );
static void set_disp_curve( uint8_t curve );
  // Ԥ���������
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

  // ���ʱ������
static void set_disp_chgr_timing_time( uint16_t timing_time );
static void set_disp_chgr_timing_time_set( uint8_t num );
static void set_update_temp_chgr_timing_time( uint8_t num );
static void set_chgr_timing_time_set_finish( uint8_t num );
static void set_close_chgr_timing_time_set( uint8_t num );

  // ԤԼ�������
static void set_disp_chgr_booking_time( uint16_t book_time );
static void set_disp_chgr_booking_time_set( uint8_t num );
static void set_update_temp_chgr_booking_time( uint8_t num );
static void set_chgr_booking_time_set_finish( uint8_t num );
static void set_close_chgr_booking_time_set( uint8_t num );

  // ��ֵ����
static void set_update_temp_chgr_threshold( uint8_t num );
static void set_disp_chgr_threshold( uint16_t data  );
static void set_chgr_threshold_set_finish( uint8_t num );

// ��������

static void set_enter_passwd_setting( void );
static void set_passwd_setting_verify( void );

// �ָ���������
static void set_restore_factory_default( void );

/**********
  �¼�
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
  ά��
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
  ����
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
		// ÿ��1s����wdg_daemon�һ�����
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
				// ����ˢ������
                suspend_touch_screen_damon();
				// ��ʾ
			    hmi_disp_emergency_stop();
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_START_FROM_EMERGENCY_STOP )
			{
				LOG_DEBUG_APP_1("          .... ts, resume");
				// �ָ�ˢ������
                resume_touch_screen_damon();
				// �Ӽ�ͣ״̬�ָ�������֮ǰҳ��
			    hmi_disp_start_from_emergency_stop();
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_NO_START ) // ��ʾ�޷�����
			{
				hmi_switch_to_spec_page( HMI_NO_START_PAGE_ID );
				LOG_DEBUG_APP_1( "\r\ndisp no start page" );
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_NO_SYN_SET ) // ��ʾ��ֹͬ������
			{
				hmi_switch_to_spec_page( HMI_NO_SYN_SETTING_PAGE_ID );
				LOG_DEBUG_APP_1( "\r\ndisp no syn setting page" );
			}
			else if( ulNotifiedValue & CHGR_NOTIFY_NO_SET ) // ��ʾ��ֹ����
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
        // ÿ��50ms������wdg_daemon�һ�����
        xTaskNotify( h_wdg_daemon, WDG_BIT_TOUCH_SCREEN_DAEMON, eSetBits );

		/* Wait to be notified of an interrupt. */
        xResult = xTaskNotifyWait( (uint32_t)   pdFALSE, /* Don't clear bits on entry. */
                                   (uint32_t)   ULONG_MAX, /* Clear all bits on exit. */
                                   (uint32_t *) &ulNotifiedValue, /* Stores the notified value. */
                                   (TickType_t) 50 ); // 50msִ��һ��
        if( xResult == pdPASS )
        {
            /* A notification was received. See which bits were set. */
            if( ulNotifiedValue & DISP_NOTIFY_FLUSH )
			{
				flush();
				continue;
			}
		}

		// У��ҳ�棬��ֹҳ���л�������
		if( phmi_info->verify.flag != FALSE )
		{
		    if( phmi_info->page.id.cur_rd != phmi_info->verify.page_id )
			{
				LOG_DEBUG_APP_1("\r\ncur_page:%d, ҳ��У��ʧ�ܣ������л���ҳ��: %d", phmi_info->page.id.cur_rd, phmi_info->verify.page_id);
			    hmi_switch_to_spec_page( phmi_info->verify.page_id );
				hmi_read_cur_page_id();
			}
			else
			{
			    phmi_info->verify.flag = FALSE;
				LOG_DEBUG_APP_1("\r\nҳ��У��ɹ���ҳ��ID: %d", phmi_info->verify.page_id);
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

		/************ 1sִ��һ�� ***********/
		// ˢ��ϵͳָʾ��
		sys_led_indicator_flush();
		
		hmi_main_overview();
		hmi_main_detail();

	    maintain_update_detail();

		// ��Ļ��ʱ����
		phmi_info->timeout ++;
		if( phmi_info->timeout >= TIMEOUT_TOUCH_SCREEN ) // 120 ��ʱ���л�������ҳ��
		{
		    phmi_info->timeout = 0;

			if( phmi_info->page.id.cur != HMI_SCREEN_SAVE_PAGE_ID )
			{
				LOG_DEBUG_APP("\r\n��Ļ������ʱ���л�������ҳ��");
//			    hmi_switch_to_spec_page( HMI_SCREEN_SAVE_PAGE_ID );
                hmi_update_page_id( HMI_SCREEN_SAVE_PAGE_ID );
                hmi_timeout();
			}
		}

		// ��ȡʱ�䣬ÿ5���ȡһ��
		get_time_cnt ++;
		if( get_time_cnt > 5 )
		{
			get_time_cnt = 0;	
		    hmi_read_sys_time();
		}
	
		// ��ȡwifi����״̬
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
		
		// ��ʾ����ʧ�� 
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

	                // ���ҳ���л�У�����
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
		// ÿ��1s����wdg_daemon�һ�����
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
				// ����������
				set_chgr_para_setting(); 
			}
			else if( ulNotifiedValue & NOTIFY_RESTORE_FACTORY_SETTING )
			{
			    // �ָ���������
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

		/************ 1sִ��һ�� ***********/
		// ˢ��ϵͳָʾ��
		sys_led_indicator_flush();
		
		hmi_main_overview();
		hmi_main_detail();

	    maintain_update_detail();

		// ��Ļ��ʱ����
		phmi_info->timeout ++;
		if( phmi_info->timeout >= TIMEOUT_TOUCH_SCREEN ) // 120 ��ʱ���л�������ҳ��
		{
		    phmi_info->timeout = 0;

			if( phmi_info->page.id.cur != HMI_SCREEN_SAVE_PAGE_ID )
			{
				LOG_DEBUG_APP("\r\n��Ļ������ʱ���л�������ҳ��");
//			    hmi_switch_to_spec_page( HMI_SCREEN_SAVE_PAGE_ID );
                hmi_update_page_id( HMI_SCREEN_SAVE_PAGE_ID );
                hmi_timeout();
			}
		}

		// ��ȡʱ�䣬ÿ5���ȡһ��
		get_time_cnt ++;
		if( get_time_cnt > 5 )
		{
			get_time_cnt = 0;	
		    hmi_read_sys_time();
		}
	
		// ��ȡwifi����״̬
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
		
		// ��ʾ����ʧ�� 
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

	                // ���ҳ���л�У�����
                    hmi_verify_page_switch( page_id );
				}
			}
		}	
}

static void hmi_read_sys_time( void )
{
	uint8_t idx = 0;
    uint8_t buff[] = { 0x5A, 0xA5, 0x03, 0x81, 0x20, 0x07 };
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void hmi_disp_wifi( uint8_t st )
{
	uint8_t idx = 0;
    uint8_t buff[] = { 0x5A, 0xA5, 0x05, 0x82, 0x00, 0xB9, 0x00, 0x00 };

	// д0����ʾ�Ͽ���д1����ʾ����
	if( st == ON )
	    buff[7] = 0x01;
	else if ( st == OFF )
	    buff[7] = 0x00;
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void set_restart( uint8_t num )
{
	// ������ʱ��ʱ��
//    hmi_main_start_timer( ID_SW_TIMER_0 );
	
	// �������� ��־λ
	if( num != POWER_UNIT_SY )
	    pdev[num]->basic.restart = BASIC_RESTART_YES;	
}

static void set_restart_finished( uint8_t num )
{
	// ֹͣ��ʱ��ʱ��
//	hmi_main_stop_timer( ID_SW_TIMER_0 );
	
    // �������״̬
	if( num != POWER_UNIT_SY )
	    pdev[num]->basic.restart = BASIC_RESTART_NO;
	
	// ���ض�Ӧģ�����õ�ҳ��
	hmi_switch_to_spec_page( Set_NumToPageId[num] );
	
	// ֹͣ��ʾ����
	hmi_stop_animate();
}

static void set_chgr_para_setting( void )
{
	// ��ʾ���������еĽ���
	phmi_info->flag_setting = TRUE;

    // ��ʾ����
    hmi_start_animate();
				
	// ִ�б�������Ĳ���
	LOG_TRACE("\r\nsave chgr para   ........");

	if( phmi_info->set.chgr_type == CHGR_TYPE_THREE_SECTION )
	{
	    LOG_TRACE_1("\r\n����ʽ��������:");
        // �������ͬ��ģʽ������ʽ��������ҳ��
        // ͬ��ģʽ�Ĳ���
	    if( phmi_info->set.cur_num == POWER_UNIT_SY ) 
	    {
	    	LOG_TRACE_1("\r\nͬ��ģʽ");
	    	// ������ģ���AC����
            power_on_all();
	    	vTaskDelay( pdMS_TO_TICKS( POWER_ON_MS_DELAY ) );
	    	LOG_TRACE_1("\r\nopen all, after 3s");

	    	// �������
	        set_save_chgr_para_syn();

	    	// delay 2s
	    	vTaskDelay( pdMS_TO_TICKS(2000) );
	    	LOG_TRACE_1("\r\ndelay 2s finish");

	    	// �ر�����ģ��
	    	power_off_all();
	    	vTaskDelay( pdMS_TO_TICKS( POWER_OFF_MS_DELAY ) );

	    	LOG_TRACE_1("\r\nclose all, after 8s");
	    } 
	    else // ������ǵ�ģʽ������ʽ��������ҳ��
	    {
	    	LOG_TRACE_1("\r\n����ģʽ");
	    	// �򿪶�Ӧģ���AC����
	    	power_on( phmi_info->set.cur_num );
	    	vTaskDelay( pdMS_TO_TICKS( POWER_ON_MS_DELAY ) );
	    	LOG_TRACE_1("\r\nopen, after 3s");

	    	// �������
	    	set_save_chgr_para( phmi_info->set.cur_num );

	    	// delay 2s
	    	vTaskDelay( pdMS_TO_TICKS(2000) );
	    	LOG_TRACE_1("\r\ndelay 2s finish");

	    	// �رն�Ӧģ��
	    	power_off( phmi_info->set.cur_num );
	    	vTaskDelay( pdMS_TO_TICKS( POWER_OFF_MS_DELAY ) );
	    	LOG_TRACE_1("\r\nclose after 8s");
	    }
    }
	else if ( phmi_info->set.chgr_type == CHGR_TYPE_PRE )
	{
	    LOG_TRACE_1("\r\nԤ����������:");
        // ͬ��ģʽ�Ĳ���
	    if( phmi_info->set.cur_num == POWER_UNIT_SY ) 
		{
	    	LOG_TRACE_1("\r\nͬ��ģʽ");
            set_save_pre_chgr_para_syn();
		}
	    else // ������ǵ�ģʽ������ʽ��������ҳ��
		{
	    	LOG_TRACE_1("\r\n����ģʽ");
            set_save_pre_chgr_para( phmi_info->set.cur_num );
		}
	}
	else if ( phmi_info->set.chgr_type == CHGR_TYPE_AUTO )
	{
	    // �����趨�Ĳ���
	    set_chgr_threshold_set_finish( phmi_info->set.cur_num );

    	// �л�����Ӧ���õ�������
//        hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );

		if( phmi_info->set.cur_num == POWER_UNIT_SY ) // �������ͬ��ģʽ������
		    LOG_DEBUG_APP("\r\n�����ֵ�������, ͬ�����ã���ֵ:%d%%", pdev[0]->chgr.para.data_auto.threshold );
		else
			LOG_DEBUG_APP("\r\n�����ֵ�������, num:%d, ��ֵ:%d%%", phmi_info->set.cur_num, pdev[phmi_info->set.cur_num]->chgr.para.data_auto.threshold );
	}

    // �����������
    set_save_chgr_para_finished( phmi_info->set.cur_num );

	// ��ʾ�������
	phmi_info->flag_setting = FALSE;

    LOG_TRACE_1("    ....finish");
}

static void set_save_chgr_para_finished( uint8_t num )
{
	// ���ض�Ӧģ�����õ�ҳ��
	hmi_switch_to_spec_page( Set_NumToPageId[num] );
	// ����л�У��
	hmi_verify_page_switch( Set_NumToPageId[num] );
	LOG_DEBUG_APP_1("\r\n��ʼУ��ҳ���л���ҳ��ID:%d", Set_NumToPageId[num] );
	
	// ֹͣ��ʾ����
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
			if( key_value == 0x0001 )  // ��ʾ�л�������ҳ-Ԥ��
			{
			    hmi_switch_to_spec_page( HMI_MAIN_PAGE_ID );
                hmi_main_overview();
			}
			break;
		case 0x0012:
			if( key_value == 0x0001 )  // ģ��1����ϸ��Ϣ
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_1_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // ��ʾ�л��������
			{
                exec_bat_type_switch( POWER_UNIT_1 );
			}
			else if( key_value == 0x0003 )  // ��ʾ����������/ֹͣ��ť
			{
				exec_chgr_op_switch( POWER_UNIT_1 );
			}
			break;
		case 0x0013:
			if( key_value == 0x0001 )  // ģ��2����ϸ��Ϣ
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_2_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // ��ʾ�л��������
			{
                exec_bat_type_switch( POWER_UNIT_2 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_2 );
			}
			break;
		case 0x0014:
			if( key_value == 0x0001 )  // ģ��3����ϸ��Ϣ
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_3_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // ��ʾ�л��������
			{
                exec_bat_type_switch( POWER_UNIT_3 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_3 );
			}
			break;
		case 0x0015:
			if( key_value == 0x0001 )  // ģ��4����ϸ��Ϣ
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_4_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // ��ʾ�л��������
			{
                exec_bat_type_switch( POWER_UNIT_4 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_4 );
			}
			break;
		case 0x0016:
			if( key_value == 0x0001 )  // ģ��5����ϸ��Ϣ
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_5_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // ��ʾ�л��������
			{
                exec_bat_type_switch( POWER_UNIT_5 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_5 );
			}
			break;
		case 0x0017:
			if( key_value == 0x0001 )  // ģ��6����ϸ��Ϣ
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_6_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // ��ʾ�л��������
			{
                exec_bat_type_switch( POWER_UNIT_6 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_6 );
			}
			break;
		case 0x0018:
			if( key_value == 0x0001 )  // ģ��7����ϸ��Ϣ
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_7_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // ��ʾ�л��������
			{
                exec_bat_type_switch( POWER_UNIT_7 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_7 );
			}
			break;
		case 0x0019:
			if( key_value == 0x0001 )  // ģ��8����ϸ��Ϣ
			{
			    hmi_switch_to_spec_page( HMI_MAIN_DETAIL_8_ID );
				hmi_main_detail();
			}
			else if( key_value == 0x0002 )  // ��ʾ�л��������
			{
                exec_bat_type_switch( POWER_UNIT_8 );
			}
			else if( key_value == 0x0003 )
			{
				exec_chgr_op_switch( POWER_UNIT_8 );
			}
			break;
		case 0x001A:
			// �л���֮ǰ��ҳ��
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
	
	// �������main_overview��ҳ�棬ֱ���˳�����ˢ������
    if( phmi_info->page.id.cur != HMI_MAIN_OVERVIEW_ID ) { return; }	
	
	for( idx = 0; idx < POWER_UNIT_NUM; idx ++)
	{
		// 1. �ж������쳣
	    if( (pdev[idx]->basic.err == BASIC_ERR_NO) && (pdev[idx]->basic.warn == BASIC_WARN_NO) )
		{
	        main_disp_led( LED_GREEN, idx );
		}
	    else
		{
		    main_disp_led( LED_RED, idx );
		}
	
	    // 2. ��ʾ���״̬
	    main_disp_bat( pdev[idx]->basic.bat, idx );
	
	    // 3. ��ʾ��ص�ѹ�͵���
	    main_disp_bat_volt( pdev[idx]->basic.bat_volt, idx ); // �������޸�Ϊ��ʾ��ص�ѹ
	    main_disp_iout( pdev[idx]->chgr.iout_disp, idx );	

	    //4.����ģʽ��ʾ
	    main_disp_mode( pdev[idx]->chgr.mode, idx );
	}
	LOG_DEBUG_APP("\r\nmain_overview flush");
}

static void hmi_main_detail(void)
{
	uint8_t num = 0;
	
	// �������ģ��1-8����ϸ��Ϣҳ�棬ֱ���˳�����ˢ������
    if( (phmi_info->page.id.cur < HMI_MAIN_DETAIL_1_ID) |
        (phmi_info->page.id.cur > HMI_MAIN_DETAIL_8_ID) )
    { return; }
	
	// ȷ��ˢ���ĸ�ģ�������
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
	
	//1.��ʾ���״̬
	main_disp_chgr_status( pdev[num]->basic.err, num );
	//2.��ʾ����ѹ
	main_disp_vout( pdev[num]->chgr.vout, num );
	//3.��ʾ������
	main_disp_iout( pdev[num]->chgr.iout_disp, num );
	//4.��ʾ���ʱ��
	main_disp_chgr_time( pdev[num]->chgr.time, num );
	//5.��ʾģ���¶�
	main_disp_temperature( pdev[num]->basic.temp, num );
	//6.��ʾ�ѳ����
	main_disp_AH( pdev[num]->chgr.AH, num );
	//7.��ʾ�����ѹ����
	main_disp_DC_OK( pdev[num]->basic.err, num );
	
	//8.���²�����ʾ
	  // ��������Զ�ģʽ������������ֹͣͼ��
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

	//9.��ʾ�����Ϣ
    main_disp_bat_info( num );

	// 10.��ʾ�������
	  // ��������Զ�ģʽ������������ֹͣͼ��
	if( pdev[num]->chgr.mode == MODE_AUTO )
	{
		main_disp_bat_type( 0xff, num );
	}
	else
		main_disp_bat_type( pdev[num]->chgr.bat_type, num );


	// 11.��ʾ�л�ͼ�꣬�����Զ�ģʽ����
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
	// ��ص�ѹ����ص�����ʣ������������¶ȣ���������������־
    uint16_t addr_list[] = {0x0821, 0x0822, 0x0823, 0x0824, 0x0826, 0x0827, 0x0832};
	uint8_t warn_buff[22] = {0}; //ʵ�ʳ���19���ַ�+1('\0')
	uint16_t data[6] = {0};
	uint8_t idx = 0;
	
	if( pdev[num]->chgr.mode == (uint8_t)MODE_MANUAL  )
	{
	    memset(data, 0, (sizeof(data[0])*5) );

		// �ֶ�ģʽ����ص�ѹ��ʾΪ�����ĵ�ص�ѹ����ص�����ʾΪ������
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

	// ��ʾ�澯��
	sprintf( (char *)warn_buff, "0x%02x 0x%02x 0x%02x 0x%02x", pdev[num]->bat_info_bms.warn_1, pdev[num]->bat_info_bms.warn_2, pdev[num]->bat_info_bms.warn_3, pdev[num]->bat_info_bms.warn_4);
	hmi_disp_string( addr_list[6], warn_buff, 19 );
}

static void main_disp_bat_type( uint8_t to, uint8_t num )
{
	uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x31, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	// ��ʾ�������ͼ�������ַ����Ϊ0x0813,0x0913...0x0A13
	buff[4] = addr_h[ num ];
	
	// д0����ʾǦ���أ�д1����ʾ﮵��
	if( to == CHGR_BAT_TYPE_LI ) 
	    buff[7] = 0x01;
	else if( to == CHGR_BAT_TYPE_LEAD_ACID )  
		buff[7] = 0x00;
	else
		buff[7] = 0x02;

	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );

	// ���ͳ�ȥ
	com_touch_screen->send();

}

static void main_disp_bat_switch_ico( uint8_t to, uint8_t num )
{
	uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x30, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	// ��ʾ�������ͼ�������ַ����Ϊ0x0813,0x0913...0x0A13
	buff[4] = addr_h[ num ];
	
	// д0����ʾ�л�ͼ�꣬д1�������л�ͼ��
	if( to == 1 ) 
	    buff[7] = 0x00;
	else if( to == 0 )  
		buff[7] = 0x01;

	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );

	// ���ͳ�ȥ
	com_touch_screen->send();

}

static void exec_chgr_op_switch( uint8_t num )
{
	if( pdev[num]->chgr.op == op_start )      // ֮ǰִ����������������ô����ִ�еľ���ֹͣ������������ʾ����
	{
		LOG_DEBUG_APP("\r\nexec stop, stop num:%d .... .... ", num);

		// ��¼�����ĸ�ģ�鰴����ֹͣ
		phmi_info->op.stop_num  = num;

		// ִ��ֹͣ����
		pub_stop( STOP_TYPE_HMI, num );
	} 
	else if( pdev[num]->chgr.op == op_stop )  // ֮ǰִ����ֹͣ��������ô����ִ�еľ�������������������ʾֹͣ
	{
		LOG_DEBUG_APP("\r\nexec start, start num:%d .... .... ", num);

		// ��¼�����ĸ�ģ�鰴��������
		phmi_info->op.start_num = num;

		// ִ����������
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
	
	// ҳ�����
	if( page_id > HMI_MAX_PAGE_ID ) return;
	
	buff[5] = ( ( page_id >> 8 ) & 0xff );
	buff[6] = ( page_id & 0xff );
	
//taskENTER_CRITICAL();
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();

//taskEXIT_CRITICAL();
	
	hmi_update_page_id( page_id ); // 20181219�޸�Ϊ�л�����������ҳ��id
}

static void hmi_verify_page_switch( uint16_t page_id )
{
	/* ���ҳ���л�У�����
	 * 1.����У���ҳ��idֵ
	 * 2.���Ͷ�ҳ���ָ��
	 * 3.�� prvTouchScreenDamonTask ���棬�ж϶�ȡҳ���Ƿ���ȷ
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
	
	// ��ռ�¼��ҳ��id��Ϣ
	phmi_info->page.id.pre = 0;
	phmi_info->page.id.cur = 0;
	phmi_info->page.id.cur_rd = 0;

	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void hmi_start_animate( void )
{
    uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x10, 0xA5, 0x00, 0x01}; 
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void hmi_stop_animate( void )
{
    uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x10, 0xA5, 0x00, 0x00}; 
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void hmi_disp_emergency_stop( void )
{
    // ��ȡ��ǰҳ��id
	hmi_read_cur_page_id();
	delay_ms( 20 );
	phmi_info->page.id.pre = phmi_info->page.id.cur_rd;
	hmi_switch_to_spec_page( HMI_MAIN_EMERGENCY_STOP_ID );
}	

static void hmi_disp_start_from_emergency_stop( void )
{
    uint8_t num = 0;
	/* ����Ƶ�����½���ֹͣ���½����л����������
	 *   - �����������֮ǰ��ҳ���ǽ���ֹͣҳ�棬��ô���л�������ҳ��
	 */
	if( phmi_info->page.id.pre == HMI_MAIN_EMERGENCY_STOP_ID )
	{
	    hmi_switch_to_spec_page( HMI_SCREEN_SAVE_PAGE_ID );
	}
	else
	{
		num = phmi_info->page.id.pre;
	    // �л���֮ǰ��ҳ��
        hmi_switch_to_spec_page( num );
//		
//		// ����л�У��
//		hmi_verify_page_switch( num );
	}
}

static void hmi_read_cur_page_id(void)
{
	uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x03, 0x81, 0x03, 0x02};
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void hmi_start_chgr( void )
{
}

static void hmi_stop_chgr( void )
{
	uint8_t idx = 0;

    // ������ʾֹͣ��
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
	
	// ���ͼ��ı�����ַ����Ϊ0811,0911,...0F11
	buff[4] = addr_h[ num ];
	
	/* �������ַд0����ʾδ���ӣ�
	            д1����ʾ������,
	            д2����ʾ���ӣ�
	            ����д3��4��5��6��7������ʾ����е�ͼ�꣬д8����ʾ����ͼ�� // 20190911���޸ĳ���ͼ��Ϊ������ͼ��
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

	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void main_disp_led( uint8_t st, uint8_t num )
{
	uint8_t idx;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x12, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	// led�ı�����ַ����Ϊ0812,0912,...0F12
	buff[4] = addr_h[ num ];
	
	// �������ַд0����ʾ�̵ƣ�д1����ʾ���
	if( st & LED_GREEN )
	    buff[7] = 0x00;
	else if(  st & LED_RED )
		buff[7] = 0x01;
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void main_disp_vout( float fvalue, uint8_t num )
{
    uint8_t idx;
	int16_t uwValue;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x19, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	buff[4] = addr_h[ num ];
	
	// ���ݸ�ʽת��������3λ��С��1λ��������Ļ��Ҫ��ʾ56.7����ô��Ҫ����567����56.7ת����567��Ȼ��ȡ��8λ�͵�8λ
	uwValue = fvalue * 10;
	
	buff[6] = (int8_t)((uwValue>>8) & 0xFF);
	buff[7] = (int8_t)(uwValue & 0xFF);
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void main_disp_bat_volt( float fvalue, uint8_t num )
{
    uint8_t idx;
	int16_t uwValue;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x26, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	buff[4] = addr_h[ num ];
	
	// ���ݸ�ʽת��������3λ��С��1λ��������Ļ��Ҫ��ʾ56.7����ô��Ҫ����567����56.7ת����567��Ȼ��ȡ��8λ�͵�8λ
	uwValue = fvalue * 10;
	
	buff[6] = (int8_t)((uwValue>>8) & 0xFF);
	buff[7] = (int8_t)(uwValue & 0xFF);
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void main_disp_iout( float fvalue, uint8_t num )
{
    uint8_t idx;
	uint16_t uwValue;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x1A, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	buff[4] = addr_h[ num ];
	
	// ���ݸ�ʽת���������56.7ת����567��Ȼ��ȡ��8λ�͵�8λ
	uwValue = fvalue * 10;
	
	buff[6] = (uint8_t)((uwValue>>8) & 0xFF);
	buff[7] = (uint8_t)(uwValue & 0xFF);
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void main_disp_chgr_status( uint8_t st, uint8_t num )
{
	uint8_t idx;
    uint8_t buff[] = {0x5A, 0xA5, 0x07, 0x82, 0x08, 0x15};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	char *str1 = "����";
	char *str2 = "�쳣";
	
	buff[4] = addr_h[ num ];
	
	// ����Ҫ���͵�����д�뻺����
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
	
	// ���ͳ�ȥ
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
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
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
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
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
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void main_disp_DC_OK( uint8_t st, uint8_t num )
{
	uint8_t idx;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x1F};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	char *str1 = "��";
	char *str2 = "��";
	char clr[4] = {' ', ' ', ' ', ' '};
	
	buff[4] = addr_h[ num ];
	
	// �����ʾ����
	idx = 0;
	while( idx < 4)
		com_touch_screen->write( clr[idx++] );
	com_touch_screen->send();
	
	// ����Ҫ���͵�����д�뻺����
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
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void main_disp_mode( uint8_t to, uint8_t num )
{
	uint8_t idx = 0;
	
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x13, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	// ��ʾ�ֶ�/�Զ���ͼ�������ַ����Ϊ0x0813,0x0913...0x0A13
	buff[4] = addr_h[ num ];
	
	// �������ַд1����ʾ�ֶ���д0����ʾ�Զ�
	if( to == MODE_MANUAL )
	    buff[7] = 0x01;
	else if( to == MODE_AUTO )
		buff[7] = 0x00;

	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );

	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void main_disp_op( uint8_t to, uint8_t num )
{
	uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x05, 0x82, 0x08, 0x14, 0x00, 0x00};
	uint8_t addr_h[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	
	// ��ʾֹͣ/������ͼ�������ַ����Ϊ0x0814,0x0914...0x0A14
	buff[4] = addr_h[ num ];
	
	// �������ַд1����ʾ������д0����ʾֹͣ
	if( to == OP_DISP_START ) 
	    buff[7] = 0x01;
	else if( to == OP_DISP_STOP )  
		buff[7] = 0x00;
	else if( to == OP_DISP_NONE )  
		buff[7] = 0x02;


	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < 8; idx++ )
	    com_touch_screen->write( buff[idx] );

	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void main_disp_starting( void )
{
	/* ��ʾ�����У�
	 * 1.�л���������ҳ��
	 * 2.����ҳ��id
     * 3.��������	
	 */
    hmi_switch_to_spec_page( HMI_MAIN_STARTING_ID );
	hmi_start_animate();
}

static void main_disp_start_success( void )
{
    /* �����ɹ���:
     * 1. ��ʾֹͣ
     * 2. ����֮ǰ��ҳ�棬������ҳ��id
	 * 3. �رն���
     */
    main_disp_op( OP_DISP_STOP, phmi_info->op.start_num );
}

static void main_disp_start_failed( void )
{
    /* ����ʧ�ܺ�:
	 * 1. ��ʾ����ͼ��
	 * 2. ��ʾ ����ʧ�� ҳ�棬������ҳ��id
	 * 3. ����2s
	 * 4. ����֮ǰ��ҳ�棬������ҳ��id
	 * 5. �رն���
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
	// �жϸ�ģ���Ƿ��ڳ�磬����ڣ���ô��ֹ��ת����ϸ��������ҳ��
    if( pdev[num]->basic.chgr != BASIC_CHGR_OFF )
	{
	    hmi_switch_to_spec_page( HMI_NO_SETTING_PAGE_ID );
	}
	// �жϸ�ģ���Ƿ����Զ�����ģʽ������ǣ���ô��ֹ��ת����ϸ��������ҳ��
    if( pdev[phmi_info->set.cur_num]->chgr.mode == MODE_AUTO )
	{
	    hmi_switch_to_spec_page( HMI_NO_SETTING_PAGE_ID );
	}
}

static void set_enter_syn_detail_settting(void)
{
	uint8_t idx = 0;
    //  �ж��Ƿ����ڳ���ģ�飬����У���ô��������ϸ��������ҳ��
	for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
	{
        //  �ж��Ƿ����ڳ���ģ�飬����У���ô��������ϸ��������ҳ��
	    if( pdev[idx]->basic.chgr != BASIC_CHGR_OFF )
		{
            hmi_switch_to_spec_page( HMI_NO_SYN_SETTING_PAGE_ID );
		}
	    // �ж��Ƿ���ģ�鴦���Զ�����ģʽ������ǣ���ô��ֹ��ת����ϸ��������ҳ��
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
	
//	if( addr == 0x0022 ) // ˵���ǰ���
	switch( addr )
	{
		case 0x0022:
			if( value == 0x0002 )  // ��ʾ�л������ã�����������ɣ�׼�������������棬��ʱ�ж�������Ƿ���ȷ
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
			else if(value == 0x0021)    // ��ʾ�л����ˣ��������-ģ��1����
			{
				phmi_info->set.cur_num = POWER_UNIT_1;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0032 )  // ��ʾ�л����ˣ��������-ģ��2����
			{
			    phmi_info->set.cur_num = POWER_UNIT_2;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0043 )  // ��ʾ�л����ˣ��������-ģ��3����
			{
			    phmi_info->set.cur_num = POWER_UNIT_3;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0054 )  // ��ʾ�л����ˣ��������-ģ��4����
			{
			    phmi_info->set.cur_num = POWER_UNIT_4;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0065 )  // ��ʾ�л����ˣ��������-ģ��5����
			{
			    phmi_info->set.cur_num = POWER_UNIT_5;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0076 )  // ��ʾ�л����ˣ��������-ģ��6����
			{
			    phmi_info->set.cur_num = POWER_UNIT_6;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0087 )  // ��ʾ�л����ˣ��������-ģ��7����
			{
			    phmi_info->set.cur_num = POWER_UNIT_7;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x0098 )  // ��ʾ�л����ˣ��������-ģ��8����
			{
			    phmi_info->set.cur_num = POWER_UNIT_8;
				set_enter_detail_setting( phmi_info->set.cur_num );
			}
			else if( value == 0x00A9 )  // ��ʾ�л����ˣ��������-ͬ������
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
					 ( value == 0x00AA ) )  // ��������ʽ��������ҳ��
			{
				phmi_info->set.chgr_type = CHGR_TYPE_THREE_SECTION;  // ��ǣ���ǰ���÷�ʽΪ����ʽ

				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // �������ͬ��ģʽ������ʽ��������ҳ��
				{
				    //  �ж��Ƿ����ڳ���ģ�飬����У���ô��������ϸ��������ҳ��
					for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
					{
				        //  �ж��Ƿ����ڳ���ģ�飬����У���ô��������ϸ��������ҳ��
					    if( pdev[idx]->basic.chgr != BASIC_CHGR_OFF )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}
					    // �ж��Ƿ���ģ�鴦���Զ�����ģʽ������ǣ���ô��ֹ��ת����ϸ��������ҳ��
					    if( pdev[idx]->chgr.mode == MODE_AUTO )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}
					}

					// ����ͬ��ģʽ����ģ��1����
	                set_update_temp_curve_data( POWER_UNIT_1, pdev[ POWER_UNIT_1 ]->chgr.para.curve );
				    set_disp_chgr_set( POWER_UNIT_1 );
				}
				else  // �������1-8ģ�飬�����Ĳ�������
				{	
    				// �жϸ�ģ���Ƿ��ڳ�磬����ڣ���ô��ֹ��ת����ϸ��������ҳ��
    			    if( pdev[phmi_info->set.cur_num]->basic.chgr != BASIC_CHGR_OFF )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}
					// �жϸ�ģ���Ƿ����Զ�����ģʽ������ǣ���ô��ֹ��ת����ϸ��������ҳ��
    			    if( pdev[phmi_info->set.cur_num]->chgr.mode == MODE_AUTO )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}
    				
    	            set_update_temp_curve_data( phmi_info->set.cur_num, pdev[ phmi_info->set.cur_num ]->chgr.para.curve );
    				set_disp_chgr_set( phmi_info->set.cur_num );
				}
			}
			else if( value == 0x00BA )  // ��ʾ����һ
			{
				LOG_TRACE_1("\r\n����һ");
				set_update_temp_curve_data( phmi_info->set.cur_num, CHGR_PARA_CURVE_1 );
			    set_disp_curve( CHGR_PARA_CURVE_1 );
			}
			else if( value == 0x00CB )  // ��ʾ���߶�
			{
				set_update_temp_curve_data( phmi_info->set.cur_num, CHGR_PARA_CURVE_2 );
			    set_disp_curve( CHGR_PARA_CURVE_2 );
			}
			else if( value == 0x00DC )  // ��ʾ������
			{
				set_update_temp_curve_data( phmi_info->set.cur_num, CHGR_PARA_CURVE_3 );
			    set_disp_curve( CHGR_PARA_CURVE_3 );
			}
			else if( value == 0x00ED )  // ��ʾ������
			{
				set_update_temp_curve_data( phmi_info->set.cur_num, CHGR_PARA_CURVE_4 );
			    set_disp_curve( CHGR_PARA_CURVE_4 );
			}
			else if( value == 0x00FE )  // ��ʾ���Ĳ���
			{
				set_update_temp_curve_data( phmi_info->set.cur_num, CHGR_PARA_CURVE_U );
			    set_disp_curve( CHGR_PARA_CURVE_U );
			}
			else if( ( value == 0x00BB ) || \
                     ( value == 0x00CC ) || \
                     ( value == 0x00DD ) || \
                     ( value == 0x00EE ) || \
                     ( value == 0x00FF ) ) // ��ʾ�ڳ������ҳ�棬���˷��أ���ô���ص�֮ǰ��ҳ��
			{
			    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num ] );
			}
			else if( ( value == 0x00BC ) || \
				     ( value == 0x00CD ) || \
                     ( value == 0x00DE ) || \
                     ( value == 0x00EF ) || \
                     ( value == 0x0100 ) || \
                     ( value == 0x00FF ) )  // ��ʾ�Ѿ�����������ϣ�ȷ����Ч
			{
				// ���������������ò���֪ͨ
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
                     ( value == 0x00AB ) )  // �����˳��ʱ������
			{
				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // �������ͬ��ģʽ������
				{
					for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
					{
				        //  �ж��Ƿ����ڳ���ģ�飬����У���ô������
					    if( pdev[idx]->basic.chgr != BASIC_CHGR_OFF )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}

					    // �ж��Ƿ���ģ�鴦���Զ�����ģʽ������ǣ���ô��ֹ��ת����ϸ��������ҳ��
					    if( pdev[idx]->chgr.mode == MODE_AUTO )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}
					}
				}
				else  // �������1-8ģ�飬����������
				{	
    				// �жϸ�ģ���Ƿ��ڳ�磬����ڣ���ô��ֹ����
    			    if( pdev[phmi_info->set.cur_num]->basic.chgr != BASIC_CHGR_OFF )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}

					// �жϸ�ģ���Ƿ����Զ�����ģʽ������ǣ���ô��ֹ��ת����ϸ��������ҳ��
    			    if( pdev[phmi_info->set.cur_num]->chgr.mode == MODE_AUTO )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}
				}

				// ������ʱ����
				set_update_temp_chgr_timing_time( phmi_info->set.cur_num );
				// ��ʾ��Ӧҳ��
                set_disp_chgr_timing_time_set( phmi_info->set.cur_num );
                // ��ʾ��Ӧ�Ķ�ʱ���ʱ��
                set_disp_chgr_timing_time( phmi_info->set.tmp_chgr_timing_time );
			}
			else if ( ( value == 0x0120 ) | \
					  ( value == 0x0122) )  // ��ʾ���˷��أ����ص���Ӧģ��ĳ��������ҳ�� 
			{
			    hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );
			}
			else if ( value == 0x011F )  // ��ʾ�����˶�ʱ��繦��
			{
                // ��ʾ��Ӧ�Ķ�ʱ���ʱ��
                set_disp_chgr_timing_time( phmi_info->set.tmp_chgr_timing_time );
				LOG_DEBUG_APP("\r\n������ʱ���");
			}
			else if ( value == 0x0121 )  // ��ʾ�����˶�ʱ���ʱ�书�ܣ������趨��ɣ�ȷ��
			{
			    // �����趨�Ĳ���
			    set_chgr_timing_time_set_finish( phmi_info->set.cur_num );

            	// �л�����Ӧ���õ�������
                hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );

				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // �������ͬ��ģʽ������
				    LOG_DEBUG_APP("\r\n��ʱ����������, ͬ������, ��ʱʱ��:%d����ʱ���״̬��%02X", pdev[0]->chgr.timing_time, pdev[phmi_info->set.cur_num]->chgr.st_timing_time );
				else
					LOG_DEBUG_APP("\r\n��ʱ����������, num:%d, ��ʱʱ��:%d����ʱ���״̬��%02X", phmi_info->set.cur_num, pdev[phmi_info->set.cur_num]->chgr.timing_time, pdev[phmi_info->set.cur_num]->chgr.st_timing_time );
			}
			else if ( value == 0x0123 )  // ��ʾ�رն�ʱ���ʱ�书��
			{
			    // �����趨�Ĳ���
			    set_close_chgr_timing_time_set( phmi_info->set.cur_num );

				LOG_DEBUG_APP("\r\n�رն�ʱ���");
			}
			else if( ( value == 0x0024 ) || \
                     ( value == 0x0035 ) || \
                     ( value == 0x0046 ) || \
                     ( value == 0x0057 ) || \
                     ( value == 0x0068 ) || \
                     ( value == 0x0079 ) || \
                     ( value == 0x008A ) || \
                     ( value == 0x009B ) || \
                     ( value == 0x00AC ) )  // ������ԤԼ�������
			{
				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // �������ͬ��ģʽ������
				{
				    //  �ж��Ƿ����ڳ���ģ�飬����У���ô������
					for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
					{
					    if( pdev[idx]->basic.chgr != BASIC_CHGR_OFF )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}
					}

					// �ж��Ƿ���ģ�鴦���Զ�����ģʽ������ǣ���ô��ֹ��ת����ϸ��������ҳ��
					if( pdev[idx]->chgr.mode == MODE_AUTO )
					{
				        hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
					    return;
					}
				}
				else  // �������1-8ģ�飬����������
				{	
    				// �жϸ�ģ���Ƿ��ڳ�磬����ڣ���ô��ֹ����
    			    if( pdev[phmi_info->set.cur_num]->basic.chgr != BASIC_CHGR_OFF )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}

					// �жϸ�ģ���Ƿ����Զ�����ģʽ������ǣ���ô��ֹ��ת����ϸ��������ҳ��
    			    if( pdev[phmi_info->set.cur_num]->chgr.mode == MODE_AUTO )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}
				}
				// ������ʱ����
				set_update_temp_chgr_booking_time( phmi_info->set.cur_num );
				// ��ʾ��Ӧҳ��
                set_disp_chgr_booking_time_set( phmi_info->set.cur_num );
                // ��ʾ��Ӧ�Ķ�ʱ���ʱ��
                set_disp_chgr_booking_time( phmi_info->set.tmp_chgr_booking_time );
			}
			else if ( ( value == 0x0128 ) | \
					  ( value == 0x012A) )  // ��ʾ���˷��أ����ص���Ӧģ��ĳ��������ҳ�� 
			{
			    hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );
			}
			else if ( value == 0x011F )  // ��ʾ������ԤԼ��繦��
			{
                // ��ʾ��Ӧ��ԤԼ���ʱ��
                set_disp_chgr_booking_time( phmi_info->set.tmp_chgr_booking_time );
				LOG_DEBUG_APP("\r\n����ԤԼ���");
			}
			else if ( value == 0x0129 )  // ��ʾ������ԤԼ���ʱ�书�ܣ������趨��ɣ�ȷ��
			{
			    // �����趨�Ĳ���
			    set_chgr_booking_time_set_finish( phmi_info->set.cur_num );

            	// �л�����Ӧ���õ�������
                hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );

				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // �������ͬ��ģʽ������
				    LOG_DEBUG_APP("\r\nԤԼ����������, ͬ������, ��ʱʱ��:%d", pdev[0]->chgr.booking_time );
				else
					LOG_DEBUG_APP("\r\nԤԼ����������, num:%d, ��ʱʱ��:%d", phmi_info->set.cur_num, pdev[phmi_info->set.cur_num]->chgr.booking_time );
			}
			else if ( value == 0x012B )  // ��ʾ�رն�ʱ���ʱ�书��
			{
			    // �����趨�Ĳ���
			    set_close_chgr_booking_time_set( phmi_info->set.cur_num );

				LOG_DEBUG_APP("\r\n�ر�ԤԼ���");
			}
			else if( ( value == 0x0025 ) || \
                     ( value == 0x0036 ) || \
                     ( value == 0x0047 ) || \
                     ( value == 0x0058 ) || \
                     ( value == 0x0069 ) || \
                     ( value == 0x007A ) || \
                     ( value == 0x008B ) || \
                     ( value == 0x009C ) || \
                     ( value == 0x00AD ) )  // ������﮵�ز�������
			{
				phmi_info->set.chgr_type = CHGR_TYPE_PRE;  // ��ǣ���ǰ����Ԥ������

				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // �������ͬ��ģʽ������
				{
					for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
					{
				        //  �ж��Ƿ����ڳ���ģ�飬����У���ô������
					    if( pdev[idx]->basic.chgr != BASIC_CHGR_OFF )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}

					    // �ж��Ƿ���ģ�鴦���Զ�����ģʽ������ǣ���ô��ֹ��ת����ϸ��������ҳ��
					    if( pdev[idx]->chgr.mode == MODE_AUTO )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}
					}
					// ����ͬ��ģʽ����ģ��1����
	                set_update_temp_curve_data( POWER_UNIT_1, pdev[ POWER_UNIT_1 ]->chgr.para.curve );
				    set_disp_chgr_set( POWER_UNIT_1 );
				}
				else  // �������1-8ģ�飬����������
				{	
    				// �жϸ�ģ���Ƿ��ڳ�磬����ڣ���ô��ֹ����
    			    if( pdev[phmi_info->set.cur_num]->basic.chgr != BASIC_CHGR_OFF )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}

					// �жϸ�ģ���Ƿ����Զ�����ģʽ������ǣ���ô��ֹ��ת����ϸ��������ҳ��
    			    if( pdev[phmi_info->set.cur_num]->chgr.mode == MODE_AUTO )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}

    	            set_update_temp_curve_data( phmi_info->set.cur_num, pdev[ phmi_info->set.cur_num ]->chgr.para.curve );
    		        set_disp_chgr_set( phmi_info->set.cur_num );
				}
			}
			else if ( ( value == 0x012F ) )  // ��ʾ��﮵�ز����������水�˷��أ����ص���Ӧģ��ĳ��������ҳ�� 
			{
			    hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );
			}
			else if ( value == 0x0130 )  // ��ʾ﮵�ز���������ɣ�ȷ����Ч
			{
				// ���������������ò���֪ͨ
		        xTaskNotify( xSettingDamonHandle, NOTIFY_CHGR_PARA_SETTING, eSetBits);
			}
			else if ( value == 0x2233 ) // ������������
			{
			    set_enter_passwd_setting();
			}
			else if ( value == 0x2234 ) // ȷ���޸�����
			{
			    set_passwd_setting_verify();
			}
			else if ( value == 0x2235 ) // ��ʾ�޸�ʧ��
			{
			    set_enter_passwd_setting();
			}
			else if ( value == 0x4457 ) // ���˻ָ��������õ�ִ��
			{
				for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
				{
					// �ж��Ƿ���ģ�鴦���Զ�����ģʽ������ǣ���ô��ֹ��ת��ȷ�Ͻ���
					if( pdev[idx]->chgr.mode == MODE_AUTO )
					{
				        hmi_switch_to_spec_page( HMI_SET_CHGR_RESTROE_FACTORY_ID );
					    return;
					}
				
				}

		        hmi_switch_to_spec_page( HMI_SET_CHGR_RESTROE_FACTORY_CONFIRM_ID ); // ����ȷ�Ͻ���
			}
			else if ( value == 0x4456 ) // ����ȷ�ϻָ���������
			{
				// ���������������ò���֪ͨ
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
                     ( value == 0x00AE ) )  // �������Զ������ֵ����
			{
				if( phmi_info->set.cur_num == POWER_UNIT_SY ) // �������ͬ��ģʽ������
				{
					for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
					{
				        //  �ж��Ƿ����ڳ���ģ�飬����У���ô������
					    if( pdev[idx]->basic.chgr != BASIC_CHGR_OFF )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}

					    // �ж��Ƿ���ģ�鴦���Զ�����ģʽ������ǣ���ô��ֹ��ת����ϸ��������ҳ��
					    if( pdev[idx]->chgr.mode == MODE_AUTO )
						{
				            hmi_switch_to_spec_page( HMI_SET_CHGR_SET_SY_ID );
						    return;
						}
					}
				}
				else  // �������1-8ģ�飬����������
				{
    				// �жϸ�ģ���Ƿ��ڳ�磬����ڣ���ô��ֹ����
    			    if( pdev[phmi_info->set.cur_num]->basic.chgr != BASIC_CHGR_OFF )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}

					// �жϸ�ģ���Ƿ����Զ�����ģʽ������ǣ���ô��ֹ��ת����ϸ��������ҳ��
    			    if( pdev[phmi_info->set.cur_num]->chgr.mode == MODE_AUTO )
    				{
    				    hmi_switch_to_spec_page( Set_NumToPageId[phmi_info->set.cur_num] );
    					return;
    				}
				}

				// ������ʱ����
				set_update_temp_chgr_threshold( phmi_info->set.cur_num );
                // ��ʾ��Ӧ�Ķ�ʱ���ʱ��
                set_disp_chgr_threshold( phmi_info->set.tmp_chgr_threshold );
			}
			else if ( value == 0x0135 )  // ��ʾ����ֵ�����У����˷��أ����ص���Ӧģ��ĳ��������ҳ�� 
			{
			    hmi_switch_to_spec_page( Set_NumToPageId[ phmi_info->set.cur_num ] );
			}
			else if ( value == 0x0136 )  // ��ʾ����ֵ�����У�ȷ���������
			{
				phmi_info->set.chgr_type = CHGR_TYPE_AUTO;  // ��ǣ���ǰ�����Զ�������
				// ���������������ò���֪ͨ
		        xTaskNotify( xSettingDamonHandle, NOTIFY_CHGR_PARA_SETTING, eSetBits);
			}
			else if( value == 0x00AF ) // �޷�ͬ������ҳ���ȷ��
			{
				hmi_switch_to_spec_page( HMI_SET_ENTER_ID );
			}
			else if( value == 0x010F ) // ��ʾ�����ò�����Ч�еĽ��棬����˳������
			{
				if( phmi_info->flag_setting != FALSE )
				{
					LOG_DEBUG_APP("\r\nin set ing status, do not switch");
					hmi_switch_to_spec_page( HMI_SET_SET_ING_ID );
				}
			}
			else if( value == 0x4457 ) // ��ʾ�ڻָ����������еĽ��棬����˳������
			{
				// �������õ��ǣ��������Ļ�Զ��л�
				// ���������Ч�е�״̬�������ͨ��������л�����Ч�еĽ���
				if( phmi_info->flag_setting != FALSE )
				{
					LOG_DEBUG_APP("\r\nin restore factory status, do not switch");
					hmi_switch_to_spec_page( HMI_SET_CHGR_RESTROE_FACTORY_ING_ID );
				}
			}
			break;
		case 0x1055:  // ��ʾ�����˺�����
			phmi_info->set.tmp_para.data.ichg = (float)value/10; LOG_TRACE("\r\n3-section, ichg: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1057:  // ��ʾ�����˾�ת����
			phmi_info->set.tmp_para.data.itaper = (float)value/10; LOG_TRACE("\r\n3-section, itaper: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1059:  // ��ʾ�����˺���ѹ
			phmi_info->set.tmp_para.data.vbst = (float)value/10; LOG_TRACE("\r\n3-section, vbst: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x105B:  // ��ʾ�����˸����ѹ
			phmi_info->set.tmp_para.data.vfloat = (float)value/10; LOG_TRACE("\r\n3-section, vfloat: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1170:  // ��ʾ������Ԥ�����
			phmi_info->set.tmp_para.data_pre.pre_iout = (float)value/10; LOG_TRACE("\r\npre_chgr, pre_iout: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1172:  // ��ʾ������Ԥ���ѹ
			phmi_info->set.tmp_para.data_pre.pre_vout = (float)value/10; LOG_TRACE("\r\npre_chgr, pre_vout: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1174:  // ��ʾ�����˺�����
			phmi_info->set.tmp_para.data_pre.ichg = (float)value/10; LOG_TRACE("\r\npre_chgr, ichg: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1176:  // ��ʾ��������ֹ��ѹ
			phmi_info->set.tmp_para.data_pre.vbst = (float)value/10; LOG_TRACE("\r\npre_chgr, vbst: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1178:  // ��ʾ�����˾�ת����
			phmi_info->set.tmp_para.data_pre.itaper = (float)value/10; LOG_TRACE("\r\npre_chgr, itaper: %f", (float)value/10);
            set_chk_input_para( (ChgrParaDef *)&phmi_info->set.tmp_para );
			break;
		case 0x1011: //  ��ʾ���������룬������������룬����������뱣�浽��ʱ��������
			set_update_tmp_input_passwd();
			break;
		case 0x115D: //  ��ʾ�������������õ����룬1.������������룬����������뱣�浽��ʱ��������; 2.�л����޸ĵ�ҳ��
			set_update_tmp_input_passwd();
			hmi_switch_to_spec_page( HMI_SET_PASSWD_SETTING_MODIFY_ID );
			break;
		case 0x1065: // ��ʾ�����˶�ʱ���ʱ�� 
			phmi_info->set.tmp_chgr_timing_time = value; LOG_DEBUG_APP("\r\nchgr_timing_time:%d", value);
			break;
		case 0x106D: // ��ʾ�����˶�ʱ���ʱ�� 
			phmi_info->set.tmp_chgr_booking_time = value; LOG_DEBUG_APP("\r\nchgr_booking_time:%d", value);
			break;
		case 0x1180: // ��ʾ�������Զ����ĵ�����ֵ
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
		    tmp[ input_len ++ ] = 'f';  // ����յ�0xff�������һ���ַ�����Ϊf���ж����������У��ж������Ƿ���ȷ������
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

    // ��ȡ��ǰ�洢��eeprom�е���������
    set_read_passwd( passwd );
    passwd[ PASSWD_LEN_SET ] = '\0';

	// �������������Ƿ���ȷ
	if( strcmp( (char *)phmi_info->set.tmp_set_passwd, (char *)passwd ) == 0 )
	{
		// ��������
	    hmi_switch_to_spec_page( HMI_SET_ENTER_ID );
	}
	else
	{
		// ������������������
	    hmi_switch_to_spec_page( HMI_SET_PASSWD_ERR_ID );
	}

	// �����ʱ������������Ļ�����
	memset((char *)phmi_info->set.tmp_set_passwd, 0xff, PASSWD_LEN_SET+1 );
}

static void set_read_passwd( uint8_t *passwd ) 
{
    ee_ReadBytes( passwd, EE_PASSWD_SET, PASSWD_LEN_SET );

	delay_ms( EE_MS_DELAY );
}

static void set_save_passwd( uint8_t *passwd )
{
	// �������õ�����
    ee_WriteBytes( passwd, EE_PASSWD_SET, PASSWD_LEN_SET );
	vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );
}

static void set_enter_passwd_setting( void )
{
	uint16_t addr = 0x1163;

	// �л�����������ҳ��
	hmi_switch_to_spec_page( HMI_SET_PASSWD_SETTING_ID );

	// ��ʾδ�޸�״̬
	hmi_disp_string( addr, (uint8_t *)"δ�޸�", 6 );
}

static void set_passwd_setting_verify( void )
{
	uint16_t addr = 0x1163;
	uint8_t len_passwd_input = 0;

	len_passwd_input = strlen( (const char *)phmi_info->set.tmp_set_passwd );

	// �ж�����ĳ����Ƿ���ȷ
    if( phmi_info->set.tmp_set_passwd[ len_passwd_input-1 ] == 'f')
	{
		// �л���������ʾ����
		hmi_switch_to_spec_page( HMI_SET_PASSWD_SETTING_FAIL_ID );
		LOG_DEBUG_APP("\r\nmodify passwd failed, passwd_input_len:%d, input passwd:%s", len_passwd_input, phmi_info->set.tmp_set_passwd );
	}
	else
	{
		// �������������
		set_save_passwd( (uint8_t *)phmi_info->set.tmp_set_passwd );

		// ����ά��������
		maintain_save_passwd( (uint8_t *)phmi_info->set.tmp_set_passwd );
		
		LOG_DEBUG_APP("\r\nmodify passwd success, input passwd:%s", phmi_info->set.tmp_set_passwd );
	    
	    // �л����������ý��棬Ȼ��״̬��ʾ���޸�
		hmi_switch_to_spec_page( HMI_SET_PASSWD_SETTING_ID );
	    hmi_disp_string( addr, (uint8_t *)"���޸�", 6 );
	}

	// �����ʱ������������Ļ�����
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
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void set_disp_chgr_set( uint8_t num )
{
	uint16_t cur_page_id_tab[] = { HMI_SET_CURVE1_ID, HMI_SET_CURVE2_ID, HMI_SET_CURVE3_ID, HMI_SET_CURVE4_ID, HMI_SET_CURVE_U_ID };
	
	if( phmi_info->set.chgr_type == CHGR_TYPE_THREE_SECTION )
	{
	    // �л�����Ӧ���ߵ�ҳ��
	    hmi_switch_to_spec_page( cur_page_id_tab [pdev[num]->chgr.para.curve] );
	    LOG_DEBUG_APP("\r\n�л���ҳ�棬 ID��%d", cur_page_id_tab [pdev[num]->chgr.para.curve]);
	    
	    // ��ʾ��Ӧ������
	    set_disp_curve( pdev[num]->chgr.para.curve );
	}
	else if( phmi_info->set.chgr_type == CHGR_TYPE_PRE )
	{
	    // ��ʾ��Ӧ�Ĳ���
        set_disp_pre_chgr_para_data( &pdev[num]->chgr.para.data_pre );
	}
}

static void set_disp_curve_data( CHGR_ParaDataDef *pdata )
{
	uint16_t uwValue = 0;
	uint16_t addr[] = { 0x1055, 0x1057, 0x1059, 0x105B }; // ichg, itaper, vbst, vfloat;
	
    // ��ʾ������
	uwValue = pdata->ichg * 10;
	hmi_disp_para_data( addr[IDX_ICHG], uwValue);
	
	// ��ʾ��ת����
	uwValue = pdata->itaper * 10;
	hmi_disp_para_data( addr[IDX_ITAPER], uwValue);
	
	// ��ʾ��ֹ��ѹ
	uwValue = pdata->vbst * 10;
	hmi_disp_para_data( addr[IDX_VBST], uwValue);
	
	// ��ʾ�����ѹ
	uwValue = pdata->vfloat * 10;
	hmi_disp_para_data( addr[IDX_VFLOAT], uwValue);
}

static void set_disp_pre_chgr_para_data( PreChgrParaDataDef *pdata )
{
	uint16_t uwValue = 0;
	uint16_t addr[] = { 0x1170, 0x1172, 0x1174, 0x1176, 0x1178 }; // pre_iout, pre_vout, ichg, vbst, itaper;
	
    // ��ʾԤ�����
	uwValue = pdata->pre_iout * 10;
	hmi_disp_para_data( addr[IDX_PRE_CHGR_PRE_IOUT], uwValue);

    // ��ʾԤ���ѹ
	uwValue = pdata->pre_vout * 10;
	hmi_disp_para_data( addr[IDX_PRE_CHGR_PRE_VOUT], uwValue);

    // ��ʾ������
	uwValue = pdata->ichg * 10;
	hmi_disp_para_data( addr[IDX_PRE_CHGR_ICHG], uwValue);
	
    // ��ʾ����ѹ
	uwValue = pdata->vbst * 10;
	hmi_disp_para_data( addr[IDX_PRE_CHGR_VBST], uwValue);

    // ��ʾ��ֹ����
	uwValue = pdata->itaper * 10;
	hmi_disp_para_data( addr[IDX_PRE_CHGR_ITAPER], uwValue);
}

static void set_disp_curve( uint8_t curve )
{
	CHGR_ParaDataDef data;
	
	// ������û����õĲ�������ʾ���õĲ���������ֱ�Ӵ�Ԥ�����������ȡ����
	if( curve == CHGR_PARA_CURVE_U )
	{
        data = phmi_info->set.tmp_para.data;
#if 0		
		printf("\r\n�Զ���, cur_num:%d %f, %f, %f ,%f", phmi_info->set.cur_num, pdev[phmi_info->set.cur_num]->chgr.para.data.ichg, \
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
	
	// ��ʾ���ߵ�����
	set_disp_curve_data( &data );
}

static void set_disp_chgr_timing_time( uint16_t data )
{
	uint8_t idx = 0;
    uint8_t buff[] = { 0x5A, 0xA5, 0x05, 0x82, 0x10, 0x65, 0x00, 0x00 };
	
	buff[6] = (data>>8) & 0xff;
	buff[7] = data & 0xff;
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void set_disp_chgr_timing_time_set( uint8_t num )
{
	// ͬ�����õĲ���������ģ��1����
	if( num == POWER_UNIT_SY )
	{
		num = POWER_UNIT_1;
	}

   	if( pdev[ num ]->chgr.st_timing_time != DISABLE )
   	{
		// �л�����ʱ��磬������ҳ��
   	    hmi_switch_to_spec_page( HMI_SET_CHGR_TIME_ON_ID ); 
   	}
	else
	{
		// �л�����ʱ��磬�رյ�ҳ��
   	    hmi_switch_to_spec_page( HMI_SET_CHGR_TIME_OFF_ID ); 
	}
}

static void set_update_temp_chgr_timing_time( uint8_t num )
{
	// ͬ�����õĲ���������ģ��1����
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

	// �����ͬ�����ã���ô��������ģ��
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

	// �����ͬ�����ã���ô��������ģ��
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
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}

static void set_disp_chgr_booking_time_set( uint8_t num )
{
	// ͬ�����õĲ���������ģ��1����
	if( num == POWER_UNIT_SY )
	{
		num = POWER_UNIT_1;
	}

   	if( pdev[ num ]->chgr.st_booking_time != DISABLE )
   	{
		// �л�����ʱ��磬������ҳ��
   	    hmi_switch_to_spec_page( HMI_SET_CHGR_BOOKING_ON_ID ); }
	else
	{
		// �л�����ʱ��磬�رյ�ҳ��
   	    hmi_switch_to_spec_page( HMI_SET_CHGR_BOOKING_OFF_ID ); 
	}
}

static void set_update_temp_chgr_booking_time( uint8_t num )
{
	// ͬ�����õĲ���������ģ��1����
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

	// �����ͬ�����ã���ô��������ģ��
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

	// �����ͬ�����ã���ô��������ģ��
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
	// ͬ�����õĲ���������ģ��1����
	if( num == POWER_UNIT_SY )
	{
		num = POWER_UNIT_1;
	}

	// ������ʱ����
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

	// �����ͬ�����ã���ô��������ģ��
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
	// ��ʾ��Ч��
	phmi_info->flag_setting = TRUE;

	// ��������
	hmi_start_animate();

	// �ر�����ģ��ĳ��
	pub_stop_all();
	
	// ������ģ��
	power_on_all();
	vTaskDelay( pdMS_TO_TICKS( POWER_ON_MS_DELAY ) );

	// �����eeprom�б���ĳ�ʼ����Ϣ�����³�ʼ��
    erase_ee_flag();

	// ���³�ʼ��ģ����Ϣ
	init_power_unit_info();

	// �޸���ʱ������Ϣ��ֱ�ӵ���ͬ��ģʽ�ı���������������ó�����
	phmi_info->set.tmp_para.curve = pdev[0]->chgr.para.curve;
	phmi_info->set.tmp_para.data = pdev[0]->chgr.para.data;
	phmi_info->set.tmp_para.data_pre = pdev[0]->chgr.para.data_pre;

	// ��Ч����
	set_save_chgr_para_syn();
    set_save_pre_chgr_para_syn();

	// �ر�����ģ��
	power_off_all();

	// �رն���
	hmi_stop_animate();

	// �л������ҳ��
	hmi_switch_to_spec_page( HMI_SET_CHGR_RESTROE_FACTORY_FINISH_ID );

	// ����л�У��
	hmi_verify_page_switch( HMI_SET_CHGR_RESTROE_FACTORY_FINISH_ID );
	LOG_DEBUG_APP_1("\r\n��ʼУ��ҳ���л���ҳ��ID:%d", (uint16_t)HMI_SET_CHGR_RESTROE_FACTORY_FINISH_ID );

	// ��ʾ��Ч���
	phmi_info->flag_setting = FALSE;
}

static void set_update_temp_curve_data( uint8_t num, uint8_t curve )
{
	LOG_DEBUG_APP("\r\nupdate tmp curve data, ");
	// �����ͬ��ģʽ����ʾģ��1�Ĳ���
	if( num == POWER_UNIT_SY )
	{
	    LOG_DEBUG_APP_1("\r\nin syn mode");
	    num = POWER_UNIT_1;
	}

	// ������ʱ���ݣ�����curve, data, !!!ee_addr����
	// !!!һ��Ҫ����ee_addr���������ᵼ�±�������ĵ�ַ����(20181228������һ��������ϲ��ҵ���������⣡����)
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
	    // ������ʱ����
	    phmi_info->set.tmp_para.curve = curve;
	    
	    LOG_DEBUG_APP_1("\r\nthree-section type");
	    if( curve == CHGR_PARA_CURVE_U )
	    {
	        LOG_DEBUG_APP_1("\r\n�Զ������");
	    }
        else
	    {
	        // �����Ԥ�õģ�ֱ�Ӵ�Ԥ���������������
	    	LOG_DEBUG_APP_1("\r\nԤ�����߲�����curve:%d, ", curve);
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
	
	/****************** ������ʽ�������� *********************/
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


	/****************** ��Ԥ�������� *********************/
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

	/****************** ���Զ�������� *********************/
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
	
	/****************** д����ʽ�������� *********************/
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

	/****************** дԤ�������� *********************/
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

	/****************** д�Զ������� *********************/
	// 
	addr += 4;
	ee_WriteBytes( (uint8_t *)&( para->data_auto.threshold ), addr, 2 );
    vTaskDelay( pdMS_TO_TICKS( EE_MS_DELAY ) );

	
//	LOG_TRACE("\r\nin write, ee_addr:%04X, curve:%d, %f, %f, %f, %f", para->ee_addr, para->curve, para->data.ichg, para->data.itaper, para->data.vbst, para->data.vfloat );
	LOG_TRACE("\r\nin write, ee_addr:%04X(H)-(%d(D)), curve:%d", para->ee_addr, para->ee_addr, para->curve );
}

static void set_save_chgr_para( uint8_t num )
{
	float range = 1; // ��д���
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
	
    // ��һ���� �Ȱ���ʱ������ֵ��ʵ�ʲ���(ee_addr, curve���ĸ�data)
	// ע�⣺����ĸ�ֵ��ʽһ��Ҫ�� set_update_temp_curve_data()����������Ӧ���������ĺ�����ֱ��ȫ����ֵ��para�ģ��������ֱ�Ӹ�ֵpara
	// �������ĺ�������ֱ�Ӹ�ֵpara����ô�����Ҫע���ˣ�һ����������ee_addr�ĸ�ֵ������ᵼ�����ò����󣬵���eeprom��������ݴ���
	pdev[num]->chgr.para = phmi_info->set.tmp_para;
	
	LOG_TRACE_1("\r\nreal para, dev:%d, ee_addr:%04X, curve:%d, ichg:%f, itaper:%f, vbst:%f, vfloat:%f", num, pdev[num]->chgr.para.ee_addr, \
	                                                                      pdev[num]->chgr.para.curve, \
	                                                                      pdev[num]->chgr.para.data.ichg, \
	                                                                      pdev[num]->chgr.para.data.itaper, \
		                                                                  pdev[num]->chgr.para.data.vbst, \
	                                                                      pdev[num]->chgr.para.data.vfloat );
	
	// �ڶ����� ��ʵ�ʲ���д�빦��ģ��
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
	// �������� �Ѳ���д��eeprom����
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
	float range = 1; // ��д���
#ifdef __LOG_TRACE
	ChgrParaDef para;
	uint8_t flag = 0;
#endif
	LOG_TRACE_1("\r\nͬ��ģʽ����:");
	LOG_TRACE_1("\r\nbefore save, tmp_para, dev:%d, curve:%d, ichg:%f, itaper:%f, vbst:%f, vfloat:%f", num, \
	                                                                      phmi_info->set.tmp_para.curve, \
	                                                                      phmi_info->set.tmp_para.data.ichg, \
	                                                                      phmi_info->set.tmp_para.data.itaper, \
		                                                                  phmi_info->set.tmp_para.data.vbst, \
	                                                                      phmi_info->set.tmp_para.data.vfloat );
	
    // ��һ���� �Ȱ���ʱ������ֵ��ʵ�ʲ���(ע�⣺��curve��data��������ee_addr!!!)
	// ע�⣺����ĸ�ֵ��ʽһ��Ҫ�� set_update_temp_curve_data()����������Ӧ���������ĺ�����ֱ��ȫ����ֵpara�ģ��������para
	// �������ĺ����Ƿֱ�ֵ�ģ���ô�����Ҫע���ˣ���Ҫ�÷ֱ�ֵ�����ﲻ��Ҫ����ee_addr����Ϊͬ��ģʽָ����ģ��1��ee_addr
	// ���θ�ֵ����ģ��
	// ͬ��ģʽ�����³���������
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
	
	// �ڶ����� ��ʵ�ʲ���д�빦��ģ��
	// ���ģ������д�������
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

	    // �������� �Ѳ���д��eeprom���棬�ֱ�д�����ģ���eeprom�ռ�
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
		
		// �����ȡ�����ݸ�д�����������Χ�ڣ�����Ϊд��ɹ�
		if ( (((para_wr - para_rd) >= (float)0) && ((para_wr - para_rd) < range)) || \
             (((para_rd - para_wr) >= (float)0) && ((para_rd - para_wr) < range)) )
		{
			return TRUE;
		}
		else // �������д�����д10��
		{
			LOG_TRACE_1("\r\nwrite failed!!!,wr:%f, rd:%f", para_wr, para_rd);
		}
	}
	
	return FALSE;
}

static void set_save_pre_chgr_para( uint8_t num )
{
	// ����ʱ�������浽ģ��
	pdev[ num ]->chgr.para.data_pre = phmi_info->set.tmp_para.data_pre;	

	LOG_TRACE_1("\r\nreal para, dev:%d, pre_iout:%f, pre_vout:%f, ichg:%f, vbst:%f, itaper:%f", num, \
	                                                                      pdev[num]->chgr.para.data_pre.pre_iout, \
	                                                                      pdev[num]->chgr.para.data_pre.pre_vout, \
	                                                                      pdev[num]->chgr.para.data_pre.ichg, \
		                                                                  pdev[num]->chgr.para.data_pre.vbst, \
	                                                                      pdev[num]->chgr.para.data_pre.itaper );

	// �Ѳ���д��eeprom����
	LOG_TRACE_1("\r\nbefore write, ee_addr:%04X, num:%d, ichg:%f", pdev[num]->chgr.para.ee_addr, num, pdev[num]->chgr.para.data.ichg );
	set_write_chgr_para_to_ee( &( pdev[num]->chgr.para ) );
}

static void set_save_pre_chgr_para_syn( void )
{
	uint8_t idx = 0;

	// ͬ��ģʽ�����³�����
	for(idx = 0; idx < POWER_UNIT_NUM; idx++ )
	{
	    pdev[ idx ]->chgr.para.data_pre = phmi_info->set.tmp_para.data_pre;	
	    LOG_TRACE_1("\r\nreal para, dev:%d, pre_iout:%f, pre_vout:%f, ichg:%f, vbst:%f, itaper:%f", idx, \
	                                                                      pdev[idx]->chgr.para.data_pre.pre_iout, \
	                                                                      pdev[idx]->chgr.para.data_pre.pre_vout, \
	                                                                      pdev[idx]->chgr.para.data_pre.ichg, \
		                                                                  pdev[idx]->chgr.para.data_pre.vbst, \
	                                                                      pdev[idx]->chgr.para.data_pre.itaper );
	    // �Ѳ���д��eeprom���棬�ֱ�д�����ģ���eeprom�ռ�
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
		case 0x0033:    // ֱ���л���ĳ��ģ��
			if( value == 0x0011 ) // ��ʾ��ʾģ��1�ĵ�ǰ����
			{
			    event_disp_cur_setting( POWER_UNIT_1 );
				event_disp_cur_setting_num( POWER_UNIT_1 );
			}
			else if( value == 0x0022 )  // ��ʾ��ʾģ��2�ĵ�ǰ����
			{
			    event_disp_cur_setting( POWER_UNIT_2 );
				event_disp_cur_setting_num( POWER_UNIT_2 );
			}
			else if( value == 0x0033 )  // ��ʾ��ʾģ��3�ĵ�ǰ����
			{
			    event_disp_cur_setting( POWER_UNIT_3 );
				event_disp_cur_setting_num( POWER_UNIT_3 );
			}
			else if( value == 0x0044 )  // ��ʾ��ʾģ��4�ĵ�ǰ����
			{
			    event_disp_cur_setting( POWER_UNIT_4 );
				event_disp_cur_setting_num( POWER_UNIT_4 );
			}
			else if( value == 0x0055 )  // ��ʾ��ʾģ��5�ĵ�ǰ����
			{
			    event_disp_cur_setting( POWER_UNIT_5 );
				event_disp_cur_setting_num( POWER_UNIT_5 );
			}
			else if( value == 0x0066 )  // ��ʾ��ʾģ��6�ĵ�ǰ����
			{
			    event_disp_cur_setting( POWER_UNIT_6 );
				event_disp_cur_setting_num( POWER_UNIT_6 );
			}
			else if( value == 0x0077 )  // ��ʾ��ʾģ��7�ĵ�ǰ����
			{
			    event_disp_cur_setting( POWER_UNIT_7 );
				event_disp_cur_setting_num( POWER_UNIT_7 );
			}
			else if( value == 0x0088 )  // ��ʾ��ʾģ��8�ĵ�ǰ����
			{
			    event_disp_cur_setting( POWER_UNIT_8 );
				event_disp_cur_setting_num( POWER_UNIT_8 );
			}
			else if( value == 0x0099 ) // ��ʾ��ʾģ��1����ʷ�¼�
			{
				phmi_info->event.cur_num = POWER_UNIT_1;

				event_disp_cur_history_num( POWER_UNIT_1 );
			    event_disp_cur_history( POWER_UNIT_1, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x00AA )  // ��ʾ��ʾģ��2����ʷ�¼�
			{
				phmi_info->event.cur_num = POWER_UNIT_2;

				event_disp_cur_history_num( POWER_UNIT_2 );
			    event_disp_cur_history( POWER_UNIT_2, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x00BB )  // ��ʾ��ʾģ��3����ʷ�¼�
			{
				phmi_info->event.cur_num = POWER_UNIT_3;

				event_disp_cur_history_num( POWER_UNIT_3 );
			    event_disp_cur_history( POWER_UNIT_3, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x00CC )  // ��ʾ��ʾģ��4����ʷ�¼�
			{
				phmi_info->event.cur_num = POWER_UNIT_4;

				event_disp_cur_history_num( POWER_UNIT_4 );
			    event_disp_cur_history( POWER_UNIT_4, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x00DD )  // ��ʾ��ʾģ��5����ʷ�¼�
			{
				phmi_info->event.cur_num = POWER_UNIT_5;

				event_disp_cur_history_num( POWER_UNIT_5 );
			    event_disp_cur_history( POWER_UNIT_5, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x00EE )  // ��ʾ��ʾģ��6����ʷ�¼�
			{
				phmi_info->event.cur_num = POWER_UNIT_6;

				event_disp_cur_history_num( POWER_UNIT_6 );
			    event_disp_cur_history( POWER_UNIT_6, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x00FF )  // ��ʾ��ʾģ��7����ʷ�¼�
			{
				phmi_info->event.cur_num = POWER_UNIT_7;

				event_disp_cur_history_num( POWER_UNIT_7 );
			    event_disp_cur_history( POWER_UNIT_7, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x0100 )  // ��ʾ��ʾģ��8����ʷ�¼�
			{
				phmi_info->event.cur_num = POWER_UNIT_8;

				event_disp_cur_history_num( POWER_UNIT_8 );
			    event_disp_cur_history( POWER_UNIT_8, 0 );
                phmi_info->event.page_num = 0;
			}
			else if( value == 0x0121 )  // �� 
			{
                event_history_sw_page( phmi_info->event.cur_num, SW_TO_LEFT );
			}
			else if( value == 0x0122 )  // �ҷ� 
			{
                event_history_sw_page( phmi_info->event.cur_num, SW_TO_RIGHT );
			}
			break;
		case 0x1818: // ͨ����ť�����ƶ��л���ĳ��ģ�飬��ʾ��Ӧ�ĵ�ǰ���� 
			event_disp_cur_setting( (value - 1) );
			break;
		case 0x189B: // ͨ����ť�����ƶ��л���ĳ��ģ�飬��ʾ��Ӧ����ʷ�¼� 
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

	// ��ʾ�������
	addr = var_addr_list[0];  // ������ַ
	data = (uint16_t)(pdev[num]->chgr.para.data.ichg * 10) & 0xffff;  // �Ѹ�����ת����16������������һλС��
	hmi_disp_para_data( addr, data );
	// ��ʾ��ת����
	addr = var_addr_list[1];  // ������ַ
	data = (uint16_t)(pdev[num]->chgr.para.data.itaper * 10) & 0xffff;  // �Ѹ�����ת����16������������һλС��
	hmi_disp_para_data( addr, data );
	// ��ʾ��ֹ��ѹ
	addr = var_addr_list[2];  // ������ַ
	data = (uint16_t)(pdev[num]->chgr.para.data.vbst * 10) & 0xffff;  // �Ѹ�����ת����16������������һλС��
	hmi_disp_para_data( addr, data );
	// ��ʾ�����ѹ
	addr = var_addr_list[3];  // ������ַ
	data = (uint16_t)(pdev[num]->chgr.para.data.vfloat * 10) & 0xffff;  // �Ѹ�����ת����16������������һλС��
	hmi_disp_para_data( addr, data );


	// ��ʾԤ�����
	addr = ver_addr_list_pre[0];  // ������ַ
	data = (uint16_t)(pdev[num]->chgr.para.data_pre.pre_iout * 10) & 0xffff;  // �Ѹ�����ת����16������������һλС��
	hmi_disp_para_data( addr, data );
	// ��ʾԤ���ѹ
	addr = ver_addr_list_pre[1];  // ������ַ
	data = (uint16_t)(pdev[num]->chgr.para.data_pre.pre_vout * 10) & 0xffff;  // �Ѹ�����ת����16������������һλС��
	hmi_disp_para_data( addr, data );
	// ��ʾ������
	addr = ver_addr_list_pre[2];  // ������ַ
	data = (uint16_t)(pdev[num]->chgr.para.data_pre.ichg * 10) & 0xffff;  // �Ѹ�����ת����16������������һλС��
	hmi_disp_para_data( addr, data );
	// ��ʾ����ѹ
	addr = ver_addr_list_pre[3];  // ������ַ
	data = (uint16_t)(pdev[num]->chgr.para.data_pre.vbst * 10) & 0xffff;  // �Ѹ�����ת����16������������һλС��
	hmi_disp_para_data( addr, data );
	// ��ʾ��ֹ����
	addr = ver_addr_list_pre[4];  // ������ַ
	data = (uint16_t)(pdev[num]->chgr.para.data_pre.itaper * 10) & 0xffff;  // �Ѹ�����ת����16������������һλС��
	hmi_disp_para_data( addr, data );



	// ��ʾ��ʱ���ʱ��
	addr = var_addr_list[4];
	if( pdev[num]->chgr.st_timing_time != DISABLE )
		data = pdev[num]->chgr.timing_time;
	else
		data = 0;
	hmi_disp_para_data( addr, data );
	LOG_DEBUG_APP_1("timing time:%d, ", data);
	
	// ��ʾԤԼ���ʱ��	
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

    //  ÿҳ�ĵ�һ�����
	line_base_num = page_num * EVENT_EACH_PAGE_NUM; 

	// ��ʾǰ����Ҫ�����ǰҳ����������ݣ����۸�ҳ�Ƿ�����ʷ��¼
	for( idx = 0; idx < EVENT_EACH_PAGE_NUM; idx++ )
	{
        event_disp_clear_one_line( idx, (line_base_num++) );
	}

	// �������ʷ��¼����ʾ 
	if( read_event( pdev[num], event, page_num, EVENT_EACH_PAGE_NUM, &num_actual_read ) )
	{
	    line_base_num = page_num * EVENT_EACH_PAGE_NUM; 

	    for( idx = 0; idx < num_actual_read; idx++ )
		{
			// ��ʾ��ʷ�¼�
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

	// ��ʾ��ţ����ʱ�������Ȼ������ʾ
	LOG_DEBUG_APP("\r\nclear event, line:%d, disp num:%d", line, num);
	hmi_disp_para_data( num_addr_list[ line ], (uint16_t)num );

	// ����¼�
	hmi_disp_string( event_addr_list[ line ], temp_str, sizeof( temp_str ) );

	// ���ʱ��
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

	// ��ʾǰ����������У���ֹ����
    event_disp_clear_one_line( line, num );


	// ��ʾ���
	hmi_disp_para_data( num_addr_list[ line ], num );

	// ��ʾ�¼�
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

		// ���ģʽ
		len = strlen( Event_chgr_mode_tab[ mode ] );
		for( idx = 0; idx < len; idx ++ )
		{
		    temp_str[ temp_str_idx++ ]  = Event_chgr_mode_tab[mode][idx];
		}
		temp_str[ temp_str_idx++ ] = '-';

		// ��Ӳ���
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

	// ��ʾʱ��
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
		
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );

	while( *str != '\0' )
	{
	    com_touch_screen->write( *str++ );
	}
	
	// ���ͳ�ȥ
	com_touch_screen->send();

}

static void hmi_maintain()
{
	uint16_t addr = phmi_rx_frame->data.addr;
	uint16_t value = phmi_rx_frame->data.data[0];


	switch( addr )
	{
		case 0x0044:
			if( value == 0x0002 )  // ��ʾ�л���ά��������������ɣ�׼������ά�����棬��ʱ�ж�����������Ƿ���ȷ
			{
			    maintain_enter();
				hmi_update_page_id ( HMI_MAINTAIN_ENTER_ID );
	            phmi_info->maintain.cur_num = POWER_UNIT_1;
			}
			else if ( value == 0x0011 )  // ��ʾ��ʾģ��1��ά����Ϣ
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_1;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_1_ID );

			    maintain_disp_detail( POWER_UNIT_1 );
			}
			else if ( value == 0x0022 )  // ��ʾ��ʾģ��2��ά����Ϣ
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_2;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_2_ID );

			    maintain_disp_detail( POWER_UNIT_2 );
			}
			else if ( value == 0x0033 )  // ��ʾ��ʾģ��3��ά����Ϣ
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_3;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_3_ID );

			    maintain_disp_detail( POWER_UNIT_3 );
			}
			else if ( value == 0x0044 )  // ��ʾ��ʾģ��4��ά����Ϣ
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_4;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_4_ID );

			    maintain_disp_detail( POWER_UNIT_4 );
			}
			else if ( value == 0x0055 )  // ��ʾ��ʾģ��5��ά����Ϣ
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_5;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_5_ID );

			    maintain_disp_detail( POWER_UNIT_5 );
			}
			else if ( value == 0x0066 )  // ��ʾ��ʾģ��6��ά����Ϣ
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_6;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_6_ID );

			    maintain_disp_detail( POWER_UNIT_6 );
			}
			else if ( value == 0x0077 )  // ��ʾ��ʾģ��7��ά����Ϣ
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_7;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_7_ID );

			    maintain_disp_detail( POWER_UNIT_7 );
			}
			else if ( value == 0x0088 )  // ��ʾ��ʾģ��8��ά����Ϣ
			{
	            phmi_info->maintain.cur_num = POWER_UNIT_8;
				hmi_update_page_id ( HMI_MAINTAIN_DETAIL_8_ID );

			    maintain_disp_detail( POWER_UNIT_8 );
			}
			break;
		case 0x2011: //  ��ʾ���������룬������������룬����������뱣�浽��ʱ��������
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

    // ��ȡ��ǰ�洢��eeprom�е���������
    maintain_read_passwd( passwd );
    passwd[ PASSWD_LEN_MAINTAIN ] = '\0';

	// �������������Ƿ���ȷ
	if( strcmp( (char *)phmi_info->set.tmp_maintain_passwd, (char *)passwd ) == 0 )
	{
		// ��������
	    hmi_switch_to_spec_page( HMI_MAINTAIN_ENTER_ID );
	}
	else
	{
		// ������������������
	    hmi_switch_to_spec_page( HMI_MAINTAIN_PASSWD_ERR_ID );
	}

	// �����ʱ������������Ļ�����
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
	
	// �䱥��״̬, ������״̬������ѹ״̬������״̬��������ӣ��������׶γ�ʱ������ѹ�׶γ�ʱ��������׶γ�ʱ
	uint16_t chgr_status_disp_addr[]={0x2041, 0x2045, 0x2049, 0x204D, 0x2051, 0x2055, 0x2059, 0x205D};
	uint8_t idx;
	
	// ��ʾ�����ѹ
	disp_addr = 0x2031;
	if( pdev[num]->basic.flag_ac_power != BASIC_POWER_OFF )
	    disp_data = ReadVin( pdev[num]->basic.addr );
	else
	    disp_data = 0;
	hmi_disp_para_data( disp_addr, disp_data );
	LOG_DEBUG_APP("\r\nvin:%d", disp_data);

	// ��ʾ�����ѹ
	disp_addr = 0x2033;
	if( pdev[num]->basic.flag_ac_power != BASIC_POWER_OFF )
	    disp_data = pdev[num]->chgr.vout * 10;  // ����һλС����������ʾ��ʱ�����10
	else
	    disp_data = 0; 
	hmi_disp_para_data( disp_addr, disp_data );
	LOG_DEBUG_APP_1( "vout: %f, ", pdev[num]->chgr.vout );

	// ��ʾ����ת��, �ݶ�Ϊ��ʾ����1��ת��
	disp_addr = 0x2035;
	if( pdev[num]->basic.flag_ac_power != BASIC_POWER_OFF )
//	    disp_data = ReadFanSpeed_1( pdev[num]->basic.addr );
	    disp_data = pdev[num]->basic.fan_speed;
	else
	    disp_data = 0; 
	hmi_disp_para_data( disp_addr, disp_data );
	LOG_DEBUG_APP_1( "fan_speed_1: %d, ", disp_data );

	// ģ���¶�
	disp_addr = 0x2037;
	if( pdev[num]->basic.flag_ac_power != BASIC_POWER_OFF )
//	    disp_data = ReadTemperature( pdev[num]->basic.addr );
	    disp_data = pdev[num]->basic.temp;
	else
	    disp_data = 0;
	hmi_disp_para_data( disp_addr, disp_data );
	LOG_DEBUG_APP_1( "temp: %d, ", disp_data );

	// �����ѹ
//	disp_addr = 0x2039;
//	maintain_disp_chgr_status( disp_addr, "��" );

	// �����ѹ
//	disp_addr = 0x203B;
//	maintain_disp_chgr_status( disp_addr, "��" );

	// ģ�����
	disp_addr = 0x203D;
	if( pdev[num]->basic.err & bit(ERR_T_ALARM) )
	    maintain_disp_chgr_status( disp_addr, "��" );
	else
	    maintain_disp_chgr_status( disp_addr, "��" );	

	// �����쳣
	//disp_addr = 0x203F;
	//maintain_disp_chgr_status( disp_addr, "��" );

	// ��ʾ���״̬
	if( pdev[num]->basic.chgr == BASIC_CHGR_OFF )    // δ���ڳ��״̬��ȫ����ʾ��
	{
	    for( idx = 0; idx < sizeof(chgr_status_disp_addr)/sizeof(uint16_t); idx ++ )
	        maintain_disp_chgr_status( chgr_status_disp_addr[idx], "��" ); 
	}
	else if( pdev[num]->basic.bat == BASIC_BAT_CONN )    // ���������ӣ�����������ʾ����
	{
	    maintain_disp_chgr_status( chgr_status_disp_addr[4], "��" );
	}
	else    // ���ڳ��״̬��
	{
		// �Ƿ�䱥
	    if( pdev[num]->chgr.status & FULLM )
	        maintain_disp_chgr_status( chgr_status_disp_addr[0], "��" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[0], "��" );

		// ������״̬
	    if( pdev[num]->chgr.status & CCM )
	        maintain_disp_chgr_status( chgr_status_disp_addr[1], "��" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[1], "��" );

		// ����ѹ״̬
	    if( pdev[num]->chgr.status & CVM )
	        maintain_disp_chgr_status( chgr_status_disp_addr[2], "��" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[2], "��" );

		// ����״̬
	    if( pdev[num]->chgr.status & FVM )
	        maintain_disp_chgr_status( chgr_status_disp_addr[3], "��" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[3], "��" );

		// �������״̬
	    if( pdev[num]->chgr.status & BTNC )
	        maintain_disp_chgr_status( chgr_status_disp_addr[4], "��" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[4], "��" );

		// �������׶γ�ʱ
	    if( pdev[num]->chgr.status & CCTOF )
	        maintain_disp_chgr_status( chgr_status_disp_addr[5], "��" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[5], "��" );

		// ����ѹ�׶γ�ʱ
	    if( pdev[num]->chgr.status & CVTOF )
	        maintain_disp_chgr_status( chgr_status_disp_addr[6], "��" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[6], "��" );

		// ������׶γ�ʱ
	    if( pdev[num]->chgr.status & FVTOF )
	        maintain_disp_chgr_status( chgr_status_disp_addr[7], "��" );
		else
	        maintain_disp_chgr_status( chgr_status_disp_addr[7], "��" );
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
	
	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof(buff); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	while (*str != '\0')
	    com_touch_screen->write( *str ++ );
	
	// ���ͳ�ȥ
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
	    LOG_DEBUG_FILE_APP("\r\nϵͳ��ǰ�ڴ��СΪ��%ld �ֽ�", mem_size);
	}
	
	LOG_DEBUG_APP("\r\ncreate_devicelist, node_size:%d, node_whole_num:%d", sizeof(DevListDef), num);
	if( NULL == (phead = (DevListDef *)pvPortMalloc(sizeof(DevListDef))) ) { LOG_DEBUG_FILE_APP("\r\nget mem failed"); return NULL; }
	
	// �����豸��������ͷ�巨
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
	    LOG_DEBUG_FILE_APP("\r\nϵͳʣ���ڴ��СΪ��%ld �ֽ�", mem_size );
	}
	
	return phead;
}

static void init_device_list(void)
{ uint8_t idx = 0;
	DevListDef *pnode;

	// �����豸�б�
	if( NULL == ( phead_device_list = create_dev_list( POWER_UNIT_NUM ) ) )
	{
		LOG_DEBUG_APP("\r\n�豸�б���ʧ��");
        return;
	}

	// ��ʼ��ָ����Щ�豸��ָ����ֵ
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

	uint8_t latest_idx = EVENT_MAX_STORE_NUM - 1; // ������Ŵ����ֵ��ʼ
	uint16_t latest_idx_base_addr = EE_EVENT_LATEST_IDX_BASE_ADDR;

    // ����¼����
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
	// ����У��
	pdev[num]->basic.smp_bat_calibrate_value = 0;
	return TRUE;
#endif

	// ��ȡ�Ƿ�ִ�й�У׼�ı�־
	ee_ReadBytes( &flag, (ee_chgr_para_addr[num] + EE_OFFSET_SMP_CALIBRATED), 1 );
	delay_ms( EE_MS_DELAY );

	if( flag == TRUE )
	{
		// ��ʾִ�й�����У׼
		pdev[num]->basic.flag_smp_calibrated = TRUE; 

		// ��ȡУ׼ֵ
	    ee_ReadBytes( (uint8_t *)&rd_calib_val, (ee_chgr_para_addr[num]+ EE_OFFSET_SMP_CALIBRATE_VAL), LEN_SMP_CALIBRATE_VAL );
	    delay_ms( EE_MS_DELAY );
		calib = (float)(int16_t)rd_calib_val/100;
		LOG_DEBUG_APP("\r\nִ�й���ѹ����У׼, У׼ֵΪ:%.2f", calib);
	}
	else
	{
        // ��ʾû��ִ�й�����У׼
		pdev[num]->basic.flag_smp_calibrated = FALSE; 

		calib = 0;
		LOG_DEBUG_APP("\r\nû��ִ�й���ѹ����У׼, ����У׼ֵΪ0" );
	}

	// ���˷Ƿ���У׼ֵ
	if( (calib > 10) || (calib + 10 < 0) )
	{
		pdev[num]->basic.flag_smp_calibrated = FALSE; 
		calib = 0;
		LOG_DEBUG_APP("\r\nУ׼ֵ=%.2f��Ϊ�Ƿ�ֵ������Ϊ0", calib );
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


		// ��ʼ������ѹ����У׼
        init_smp_calibrated( idx );

		// ��ʼ���������Ĵ洢��ַ
		// һ��Ҫ���ڳ�ʼ��������֮ǰ
		pdev[idx]->chgr.para.ee_addr = ee_chgr_para_addr[idx]; 
		
		// ��ʼ�������������޺����ޣ�һ��Ҫ���ڳ�ʼ��������֮ǰ����Ϊis_been_init����Ҫ��
		    // ����ʽ���߳����� 
		pdev[idx]->chgr.data_max.ichg   = curve_data_mm[CHGR_PARA_DATA_MAX][IDX_ICHG];
		pdev[idx]->chgr.data_max.itaper = curve_data_mm[CHGR_PARA_DATA_MAX][IDX_ITAPER];
		pdev[idx]->chgr.data_max.vbst   = curve_data_mm[CHGR_PARA_DATA_MAX][IDX_VBST];
		pdev[idx]->chgr.data_max.vfloat = curve_data_mm[CHGR_PARA_DATA_MAX][IDX_VFLOAT];
		pdev[idx]->chgr.data_min.ichg   = curve_data_mm[CHGR_PARA_DATA_MIN][IDX_ICHG];
		pdev[idx]->chgr.data_min.itaper = curve_data_mm[CHGR_PARA_DATA_MIN][IDX_ITAPER];
		pdev[idx]->chgr.data_min.vbst   = curve_data_mm[CHGR_PARA_DATA_MIN][IDX_VBST];
		pdev[idx]->chgr.data_min.vfloat = curve_data_mm[CHGR_PARA_DATA_MIN][IDX_VFLOAT];	
		
		    // Ԥ��������
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

		/* ��ʼ��������
		 */
		// �����ʼ������ֱ�Ӵ�eeprom�����ȡ����
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
		else // ���û�г�ʼ������ִ�����²���
		{
		    // ��ʼ��ΪĬ�ϳ�����
			pdev[idx]->chgr.para.curve = CHGR_PARA_CURVE_U;
			pdev[idx]->chgr.para.data.ichg = default_curve_data[CHGR_PARA_CURVE_U][IDX_ICHG];
			pdev[idx]->chgr.para.data.itaper = default_curve_data[CHGR_PARA_CURVE_U][IDX_ITAPER];
			pdev[idx]->chgr.para.data.vbst = default_curve_data[CHGR_PARA_CURVE_U][IDX_VBST];
			pdev[idx]->chgr.para.data.vfloat = default_curve_data[CHGR_PARA_CURVE_U][IDX_VFLOAT];

			    // Ԥ������
			pdev[idx]->chgr.para.data_pre.pre_iout = default_pre_chgr_para_data[ IDX_PRE_CHGR_PRE_IOUT ];
			pdev[idx]->chgr.para.data_pre.pre_vout = default_pre_chgr_para_data[ IDX_PRE_CHGR_PRE_VOUT ];
			pdev[idx]->chgr.para.data_pre.ichg = default_pre_chgr_para_data[ IDX_PRE_CHGR_ICHG ];
			pdev[idx]->chgr.para.data_pre.vbst = default_pre_chgr_para_data[ IDX_PRE_CHGR_VBST ];
			pdev[idx]->chgr.para.data_pre.itaper = default_pre_chgr_para_data[ IDX_PRE_CHGR_ITAPER ];
			
			    // �Զ�������
			pdev[idx]->chgr.para.data_auto.threshold = 100;
			
			// �Ѳ���д��eeprom
			set_write_chgr_para_to_ee( &( pdev[idx]->chgr.para) );
			
			// ��Ĭ�Ͻ������õ�����д��eeprom
            set_save_passwd( set_passwd_default );

			// ��Ĭ�Ͻ���ά��������д��eeprom
            maintain_save_passwd( maintain_passwd_default );

			// �����Ѿ���ʼ���ı�־λ
			set_been_init( idx );
			
            LOG_TRACE("\r\nfirst init, cur_num:%d , curve:%d, %f, %f, %f ,%f", pdev[idx]->basic.num, pdev[idx]->chgr.para.curve, pdev[idx]->chgr.para.data.ichg, \
		    pdev[idx]->chgr.para.data.itaper, \
		    pdev[idx]->chgr.para.data.vbst, \
		    pdev[idx]->chgr.para.data.vfloat );

			// ����¼���־λ
			clear_history_event();
		}
		
		// �������
//		pdev[idx]->chgr.bat_type = CHGR_BAT_TYPE_LEAD_ACID;
		pdev[idx]->chgr.bat_type = CHGR_BAT_TYPE_LI;

		// ������ͣ�Ĭ������ʽ
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
		pdev[idx]->chgr.step = CHGR_STEP_IDLE; // ���У���ʾδ���
		memset( pdev[idx]->chgr.cnt_box, 0, ( sizeof(pdev[idx]->chgr.cnt_box) ) ); // ���м�������ʼ��Ϊ0
		pdev[idx]->chgr.notify = CHGR_NOTIFY_NONE;

		// �Զ���粿��
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

		// �����bms��ȡ�ĵ����Ϣ
		memset( &pdev[idx]->bat_info_bms, 0, sizeof( BatInfoDefFromBMS ) );
		pdev[idx]->bat_info_bms.flag_full_volt = FALSE;
		pdev[idx]->bat_info_bms.full_volt = 576; // Ĭ��ֵ�趨Ϊ57.6V

		pdev[idx]->chgr.iout_queue.head = 0;
//		pdev[idx]->chgr.iout_queue.tail = 0;
		pdev[idx]->chgr.iout_queue.full_flag = FALSE;

		pdev[idx]->chgr.flag_timing = FALSE;
		pdev[idx]->chgr.flag_booking = FALSE;
		pdev[idx]->chgr.flag_bms_timeout = FALSE;

		pdev[idx]->chgr.flag_in_chgr = FALSE;
		
		// ����ģʽ
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
	
	// ��ȡ��ʼ���ı�־λ
	ee_ReadBytes( &flag, EE_ADDR_BEEN_INIT_FLAG, 1 );
	delay_ms( EE_MS_DELAY );
	
	// ���ж�eeprom�г�ʼ���ı�־λ
	// ���numģ���Ӧ��λû�г�ʼ��������ֱ�ӷ���FALSE
	if( ( (flag>>num) & FLAG_BENN_INIT ) != FLAG_BENN_INIT )
	{
		LOG_TRACE("\r\ninit flag:%02x, ģ��%dδ��ʼ����", flag,  num);
	    return FALSE;
	}
	
	// ���жϲ����Ƿ���ȷ����Ҫ�жϳ�����
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


	// �ж�Ԥ������
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

	// �ж��Զ�������
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
	
	// ��ȡ��ʼ���ı�־λ
	ee_ReadBytes( &flag, EE_ADDR_BEEN_INIT_FLAG, 1 );
	delay_ms( EE_MS_DELAY );
    LOG_TRACE("\r\nset been init, init flag:%02X", flag);
    // ���ö�Ӧnumģ��ĳ�ʼ��λΪ FLAG_BENN_INIT
    flag = flag | (FLAG_BENN_INIT << num );	
		
	// д�뵽eeprom
	ee_WriteBytes( &flag, EE_ADDR_BEEN_INIT_FLAG, 1 );
	delay_ms( EE_MS_DELAY );
}

void hmi_init(void)
{
BaseType_t xReturn = pdPASS;

	// ����ģ����ʾ���ֶ���
	main_disp_mode( MODE_MANUAL, POWER_UNIT_1 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_2 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_3 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_4 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_5 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_6 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_7 );
	main_disp_mode( MODE_MANUAL, POWER_UNIT_8 );
	
	// ����ģ����ʾ��������
	main_disp_op( OP_DISP_START, POWER_UNIT_1 );
	main_disp_op( OP_DISP_START, POWER_UNIT_2 );
	main_disp_op( OP_DISP_START, POWER_UNIT_3 );
	main_disp_op( OP_DISP_START, POWER_UNIT_4 );
	main_disp_op( OP_DISP_START, POWER_UNIT_5 );
	main_disp_op( OP_DISP_START, POWER_UNIT_6 );
	main_disp_op( OP_DISP_START, POWER_UNIT_7 );
	main_disp_op( OP_DISP_START, POWER_UNIT_8 );

	// ��ȡ��Ʒ������������
	ee_ReadBytes( phmi_info->serial_num, EE_SERIAL_NUM_ADDR, EE_SERIAL_NUM_LEN );
	ee_ReadBytes( phmi_info->product_code, EE_PRODUCT_CODE_ADDR, EE_PRODUCT_CODE_LEN );

    // ��¼��Ļ��ʱ�ļ򵥼�����
    phmi_info->timeout = 0;

	// ��ʾ�Ƿ���������Ч��
    phmi_info->flag_setting = FALSE;

	phmi_info->page.id.pre = HMI_SCREEN_SAVE_PAGE_ID;
	phmi_info->page.id.cur = HMI_SCREEN_SAVE_PAGE_ID;
	phmi_info->page.id.cur_rd = HMI_SCREEN_SAVE_PAGE_ID;

	// �洢У�����Ϣ
    phmi_info->verify.page_id = 0;
    phmi_info->verify.flag = FALSE;

	// ������Ļ
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
	// ��ʼ���豸�б�
    init_device_list();

	// ��ʼ���豸��Ϣ
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
	
	phmi_info->timeout = 0; // ����������ʱ

	// ����������ϵͳҳ��
	if( phmi_rx_frame->data.addr == HMI_KEY_ADDR_START_SYSTEM )
	{
		LOG_DEBUG_APP("\r\nϵͳ");
		hmi_update_page_id( HMI_SCREEN_SAVE_PAGE_ID );
//		hmi_screen_save();
	}
	// ����ҳҳ��
    else if( (phmi_rx_frame->data.addr >= HMI_KEY_ADDR_START_MAIN) &&\
        (phmi_rx_frame->data.addr <= HMI_KEY_ADDR_END_MAIN) )
	{
		LOG_DEBUG_APP("\r\n��ҳ");
//		hmi_update_page_id( HMI_MAIN_PAGE_ID );
		hmi_main();
	}
	// ������ҳ��
	else if( (phmi_rx_frame->data.addr == HMI_KEY_ADDR_SET) || \
             ((phmi_rx_frame->data.addr >= 0x1011) && (phmi_rx_frame->data.addr <= 0x1810)) )
	{
		LOG_DEBUG_APP("\r\n����");
		hmi_update_page_id( HMI_SET_PAGE_ID );
		hmi_set();
	}
	else if( (phmi_rx_frame->data.addr == HMI_KEY_ADDR_EVENT) || \
			 ( (phmi_rx_frame->data.addr >= HMI_VAR_ADDR_START_EVENT) &&  (phmi_rx_frame->data.addr <= HMI_VAR_ADDR_END_EVENT) ) )
	{
		LOG_DEBUG_APP("\r\n�¼�");
		hmi_update_page_id( HMI_EVENT_PAGE_ID );
	    hmi_event();
	}
	else if( (phmi_rx_frame->data.addr == HMI_KEY_ADDR_MAINTAIN) || \
			 ( (phmi_rx_frame->data.addr >= HMI_VAR_ADDR_START_MAINTAIN) &&  (phmi_rx_frame->data.addr <= HMI_VAR_ADDR_END_MAINTAIN) ) )
	{
		LOG_DEBUG_APP("\r\nά��");
		hmi_update_page_id( HMI_MAINTAIN_PAGE_ID );
	    hmi_maintain();
	}
	else if( (phmi_rx_frame->data.addr == HMI_KEY_ADDR_HELP) || \
			 ( (phmi_rx_frame->data.addr >= HMI_VAR_ADDR_START_HELP) &&  (phmi_rx_frame->data.addr <= HMI_VAR_ADDR_END_HELP) ) )
	{
		LOG_DEBUG_APP("\r\n����");
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
	// ��ʼ���洢֡��λ��
	memset( phmi_rx_frame, 0, sizeof( *phmi_rx_frame ));

// for debug
//    uart_debug( com_touch_screen, 50 );

	// ��������֡
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
    if( phmi_rx_frame->ins == CMD_RD_VAR )  // ����Ƕ�����
	{
	    // ��������ַ
	    phmi_rx_frame->data.addr = (rx_buffer[idx++] << 8 ) & 0xFF00;
	    phmi_rx_frame->data.addr |= (rx_buffer[idx++] & 0x00FF);
		LOG_DEBUG_APP_1("\r\ndata.addr:%04X ", phmi_rx_frame->data.addr);

        // ��ȡ���ַ����ȡʵ�ʵ����ݳ���
    	phmi_rx_frame->data.len = rx_buffer[idx++];
		LOG_DEBUG_APP_1("data len:%d ", phmi_rx_frame->data.len);

		// ��ȡʵ�ʵ�����ֵ
		LOG_DEBUG_APP_1("data: ");
		len = phmi_rx_frame->data.len;
		while( len -- )
		{
		    phmi_rx_frame->data.data[data_idx] = (rx_buffer[idx++] << 8) & 0xFF00;
		    phmi_rx_frame->data.data[data_idx++] |= (rx_buffer[idx++] & 0x00FF);
		    LOG_DEBUG_APP_1("%04X ", phmi_rx_frame->data.data[data_idx-1]);
		}
	}
	else if( phmi_rx_frame->ins == CMD_RD_REG ) // ����Ƕ��Ĵ���
	{
		// ���Ĵ�����ַ
	    phmi_rx_frame->data_reg.addr = rx_buffer[idx++];
		LOG_DEBUG_APP_1("\r\ndata_reg.addr:%02X ", phmi_rx_frame->data_reg.addr);

	    // ��ȡ���ַ����ȡʵ�ʵ����ݳ���
		phmi_rx_frame->data_reg.len = rx_buffer[idx++];
		LOG_DEBUG_APP_1("data len:%d ", phmi_rx_frame->data_reg.len);

		// ��ȡʵ�ʵ�����ֵ
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
	// ����ǰҳ��id���Ĵ�����ַΪ0x03�����ݳ���Ϊ2���ֽ�	
	if( phmi_rx_frame->ins == CMD_RD_REG ) 
	{
		if( phmi_rx_frame->data_reg.addr == 0x03 )
		{
		    phmi_info->page.id.cur_rd = ((phmi_rx_frame->data_reg.data[0] << 8) | (phmi_rx_frame->data_reg.data[1]) );

	        LOG_DEBUG_APP("\r\ncur page id:%d", ((phmi_rx_frame->data_reg.data[0] << 8) | (phmi_rx_frame->data_reg.data[1]) ) );

	        // ��ջ�����
	        memset( phmi_rx_frame, 0, sizeof(hmi_rx_frame) );

		    return FALSE; // return false����ʾ�����ˣ�����Ҫ������һ��
		}
		else if( phmi_rx_frame->data_reg.addr == 0x20 ) // ��ʱ��
		{
	        // ��ʱ��
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
			 
	        // ��ջ�����
	        memset( phmi_rx_frame, 0, sizeof(hmi_rx_frame) );

		    return FALSE; // return false����ʾ�����ˣ�����Ҫ������һ��
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
	    LOG_DEBUG_APP("\r\nִ��У׼:");
		bat_volt_smp = pdev[num]->basic.smp_bat_volt_ori;
		bat_volt_bms = pdev[num]->chgr.auto_bat_volt;
	    LOG_DEBUG_APP_1("������ѹ��%f, ��bms��ȡ��ѹ:%f", bat_volt_smp, bat_volt_bms);
		calib = bat_volt_bms - bat_volt_smp;
		// ����У׼ֵ
		if( (calib-10>0) || (calib+10<0) )
		{
			LOG_DEBUG_APP_1("\r\ncalib=%.1f, �Ƿ�У׼ֵ, stop calibrate", calib);
			return FALSE;
		}

		flag = TRUE;
		pdev[num]->basic.smp_bat_calibrate_value = calib;

		// ����У׼״̬
		pdev[num]->basic.flag_smp_calibrated = flag;
	    ee_WriteBytes( &flag, (ee_chgr_para_addr[num]+EE_OFFSET_SMP_CALIBRATED), 1 );
	    delay_ms( EE_MS_DELAY );

		// ����У׼ֵ
		calib_save = calib * 100;
	    ee_WriteBytes( (uint8_t *)&calib_save, (ee_chgr_para_addr[num]+EE_OFFSET_SMP_CALIBRATE_VAL), LEN_SMP_CALIBRATE_VAL );
	    delay_ms( EE_MS_DELAY );
		LOG_DEBUG_APP_1("\r\nУ׼ֵΪ%.2f", calib);

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
	// ������ʾ���
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
    // �ָ���ʾ���
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


