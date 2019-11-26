#include "applications.h"
#include "string.h"
#include "service.h"
#include "adc.h"

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


#define    HMI_MAX_PAGE_ID    184
#define    HMI_SCREEN_SAVE_PAGE_ID 0
#define    HMI_MAIN_PAGE_ID        1
#define    HMI_MAIN_OVERVIEW_ID    1



// �����������֡�洢����
static HMI_RxFrameDef hmi_rx_frame;
static HMI_RxFrameDef *phmi_rx_frame = &hmi_rx_frame;
// �洢��Ļ��Ϣ�ı���
static HMI_InfoDef hmi_info;
static HMI_InfoDef *phmi_info = &hmi_info;
// �洢����ģ����Ϣ�ı���
static PU_InfoDef pdev[POWER_UNIT_NUM+1] = { NULL };

static void hmi_reset( void );
void hmi_init(void);
void process_hmi_rx_frame(void);
static BOOL frame_parse(void);
static BOOL frame_chk(void);
static void hmi_main(void);
static void hmi_main_overview(void);
static void hmi_switch_to_spec_page( uint16_t page_id );

static void main_disp_bat( uint8_t to, uint8_t num );
static void main_disp_bat_volt( float fvalue, uint8_t num );












/****************** chgr related********************************/
#define    CHK_BAT_DAMON_STACK    256
#define    CHK_BAT_DAMON_PRIO     3
#define    CHK_BAT_DAMON_PERIOD   500    // 500ms
static TaskHandle_t xChkBatDamonHandle = NULL;
static void prvChkBatDamonTask( void *pvParameter );

#define    FILTER_CHK_BAT_REVERSE    2
#define    FILTER_CHK_BAT_CONN       2
#define    FILTER_CHK_BAT_TYPE       2

#define    SMP_BAT_VOLT_FILTER_CNT    100
static void deal_with_smp_bat_volt( PU_InfoDef *pdev  );
static void chk_bat_conn_st( PU_InfoDef *pdev );	
BOOL isBatConn( float volt );
static BOOL isBatReverse( float volt );
////static void init_power_unit_info(void);



TaskHandle_t h_touch_screen_entry;
void touch_screen_entry( void *pvParameter )
{
BaseType_t xResult;
uint32_t ulNotifiedValue;
const TickType_t xTicksToWait = pdMS_TO_TICKS( 1000 );

	hmi_init();
	
	for( ;; )
	{
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
		}
		else
		{
		    hmi_main_overview();
		}
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
	}
}

static void hmi_main_overview(void)
{
	uint8_t idx = 0;
	
	// �������main_overview��ҳ�棬ֱ���˳�����ˢ������
    if( phmi_info->page.id.cur != HMI_MAIN_OVERVIEW_ID ) { return; }	
	
	for( idx = 0; idx < POWER_UNIT_NUM; idx ++)
	{
	    // 2. ��ʾ���״̬
	    main_disp_bat( pdev[idx].basic.bat, idx );
	
	    // 3. ��ʾ��ص�ѹ�͵���
	    main_disp_bat_volt( pdev[idx].basic.bat_volt, idx ); // �������޸�Ϊ��ʾ��ص�ѹ
	}
}

static void hmi_switch_to_spec_page( uint16_t page_id )
{
	uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x04, 0x80, 0x03, 0x00, 0x00};
	
	// ҳ�����
	if( page_id > HMI_MAX_PAGE_ID ) return;
	
	buff[5] = ( ( page_id >> 8 ) & 0xff );
	buff[6] = ( page_id & 0xff );

	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();

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

void process_hmi_rx_frame(void)
{
	if( !frame_parse() )
	{
		com_touch_screen->reset_rx();
		return;
	}
	if( !frame_chk() )
		return;
	
	// ����������ϵͳҳ��
	if( phmi_rx_frame->data.addr == HMI_KEY_ADDR_START_SYSTEM )
	{
//		hmi_screen_save();
	}
	// ����ҳҳ��
    else if( (phmi_rx_frame->data.addr >= HMI_KEY_ADDR_START_MAIN) &&\
        (phmi_rx_frame->data.addr <= HMI_KEY_ADDR_END_MAIN) )
	{
		hmi_main();
	}
		// ������ҳ��
	else if( (phmi_rx_frame->data.addr == HMI_KEY_ADDR_SET) || \
             ((phmi_rx_frame->data.addr >= 0x1011) && (phmi_rx_frame->data.addr <= 0x1810)) )
	{
		hmi_switch_to_spec_page( HMI_MAIN_PAGE_ID );
	}
	else if( (phmi_rx_frame->data.addr == HMI_KEY_ADDR_EVENT) || \
			 ( (phmi_rx_frame->data.addr >= HMI_VAR_ADDR_START_EVENT) &&  (phmi_rx_frame->data.addr <= HMI_VAR_ADDR_END_EVENT) ) )
	{
		hmi_switch_to_spec_page( HMI_MAIN_PAGE_ID );
	}
	else if( (phmi_rx_frame->data.addr == HMI_KEY_ADDR_MAINTAIN) || \
			 ( (phmi_rx_frame->data.addr >= HMI_VAR_ADDR_START_MAINTAIN) &&  (phmi_rx_frame->data.addr <= HMI_VAR_ADDR_END_MAINTAIN) ) )
	{
		hmi_switch_to_spec_page( HMI_MAIN_PAGE_ID );
	}
	else if( (phmi_rx_frame->data.addr == HMI_KEY_ADDR_HELP) || \
			 ( (phmi_rx_frame->data.addr >= HMI_VAR_ADDR_START_HELP) &&  (phmi_rx_frame->data.addr <= HMI_VAR_ADDR_END_HELP) ) )
	{
		hmi_switch_to_spec_page( HMI_MAIN_PAGE_ID );
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
	}

	return TRUE;
}


void hmi_init(void)
{
BaseType_t xReturn = pdPASS;

	// ������Ļ
	hmi_reset();
	init_power_unit_info();
	
	
	phmi_info->page.id.cur = HMI_SCREEN_SAVE_PAGE_ID;

		// create bat chk damon task
		xReturn = xTaskCreate( (TaskFunction_t) prvChkBatDamonTask,
						(const char *) "chk_bat_damon",
						(unsigned short) CHK_BAT_DAMON_STACK,
						(void *) NULL,
						(UBaseType_t) CHK_BAT_DAMON_PRIO,
						(TaskHandle_t *) &xChkBatDamonHandle );
		if( xReturn != pdPASS ) { kprintf("\r\ncreate chk bat damon failed"); return; }
}

static void hmi_reset( void )
{
    uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x04, 0x80, 0xEE, 0x5A, 0xA5}; 

	// ����Ҫ���͵�����д�뻺����
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// ���ͳ�ȥ
	com_touch_screen->send();
}











/*******************************************************************************************
 bat related
*******************************************************************************************/
static void prvChkBatDamonTask( void *pvParameter )
{
	uint32_t i;

    for( ;; )
	{
		for( i = 0; i< POWER_UNIT_NUM; i++)
		{
		    deal_with_smp_bat_volt(&pdev[i]);
			chk_bat_conn_st( &pdev[i]);
		}
		
		vTaskDelay( pdMS_TO_TICKS( CHK_BAT_DAMON_PERIOD ) );
	}
}

void init_power_unit_info(void)
{
	uint32_t idx = 0;
	
	for( idx = 0; idx < POWER_UNIT_NUM; idx ++ )
	{
		pdev[idx].basic.num = idx; // 0,1,2,3,4,5,6,7
    	pdev[idx].basic.err = BASIC_ERR_NO;
		pdev[idx].basic.warn = BASIC_WARN_NO;
    	pdev[idx].basic.chgr = BASIC_CHGR_OFF;
    	pdev[idx].basic.bat = BASIC_BAT_DISCONN;
    	pdev[idx].basic.temp = 25;
    	pdev[idx].basic.temp_before = 25;
    	pdev[idx].basic.fan_speed = 0;
    	pdev[idx].basic.bat_volt = 0;
    	pdev[idx].basic.flag_delay_close_ac = OFF;
		pdev[idx].basic.flag_bms_conn = OFF;
		pdev[idx].basic.smp_bat_calibrate_value = 0;
		
		pdev[idx].chgr.step = CHGR_STEP_IDLE;
		memset( pdev[idx].chgr.cnt_box, 0, ( sizeof(pdev[idx].chgr.cnt_box) ) );
	}
}

static void deal_with_smp_bat_volt( PU_InfoDef *pdev  )
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

static void chk_bat_conn_st( PU_InfoDef *pdev )
{
	float volt = 0;

	// ��ȡ�����ĵ�ص�ѹ
    volt = pdev->basic.bat_volt;
	
	if( pdev->chgr.step == CHGR_STEP_IDLE )
	{
	    pdev->chgr.step = CHGR_STEP_CHK_BAT_REVERSE;
	}
	
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
    
    		// ���������
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] = 0;
    	}
    	// ������δͨ�� 
    	else if( pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] <= (int32_t)( FILTER_CHK_BAT_REVERSE * (-1) ) )
    	{
    		// ���õ�ط��ӱ�־
    		pdev->basic.bat = BASIC_BAT_REVERSE;
    
    		
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
    		
    		// ���������
            pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] = 0;
    
    		// ���ؿ���״̬�����¼��
    	    pdev->chgr.step = CHGR_STEP_IDLE;
    	}
    }
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





