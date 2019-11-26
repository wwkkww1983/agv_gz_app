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



// 解析后的数据帧存储变量
static HMI_RxFrameDef hmi_rx_frame;
static HMI_RxFrameDef *phmi_rx_frame = &hmi_rx_frame;
// 存储屏幕信息的变量
static HMI_InfoDef hmi_info;
static HMI_InfoDef *phmi_info = &hmi_info;
// 存储功率模块信息的变量
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
			if( key_value == 0x0001 )  // 表示切换到了主页-预览
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
	
	// 如果不在main_overview的页面，直接退出，不刷新数据
    if( phmi_info->page.id.cur != HMI_MAIN_OVERVIEW_ID ) { return; }	
	
	for( idx = 0; idx < POWER_UNIT_NUM; idx ++)
	{
	    // 2. 显示电池状态
	    main_disp_bat( pdev[idx].basic.bat, idx );
	
	    // 3. 显示电池电压和电流
	    main_disp_bat_volt( pdev[idx].basic.bat_volt, idx ); // 主界面修改为显示电池电压
	}
}

static void hmi_switch_to_spec_page( uint16_t page_id )
{
	uint8_t idx = 0;
    uint8_t buff[] = {0x5A, 0xA5, 0x04, 0x80, 0x03, 0x00, 0x00};
	
	// 页面过滤
	if( page_id > HMI_MAX_PAGE_ID ) return;
	
	buff[5] = ( ( page_id >> 8 ) & 0xff );
	buff[6] = ( page_id & 0xff );

	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
	com_touch_screen->send();

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

void process_hmi_rx_frame(void)
{
	if( !frame_parse() )
	{
		com_touch_screen->reset_rx();
		return;
	}
	if( !frame_chk() )
		return;
	
	// 处于屏保等系统页面
	if( phmi_rx_frame->data.addr == HMI_KEY_ADDR_START_SYSTEM )
	{
//		hmi_screen_save();
	}
	// 在主页页面
    else if( (phmi_rx_frame->data.addr >= HMI_KEY_ADDR_START_MAIN) &&\
        (phmi_rx_frame->data.addr <= HMI_KEY_ADDR_END_MAIN) )
	{
		hmi_main();
	}
		// 在设置页面
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
	}

	return TRUE;
}


void hmi_init(void)
{
BaseType_t xReturn = pdPASS;

	// 重置屏幕
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

	// 把需要发送的数据写入缓冲区
	for( idx = 0; idx < sizeof( buff ); idx++ )
	    com_touch_screen->write( buff[idx] );
	
	// 发送出去
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

static void chk_bat_conn_st( PU_InfoDef *pdev )
{
	float volt = 0;

	// 读取采样的电池电压
    volt = pdev->basic.bat_volt;
	
	if( pdev->chgr.step == CHGR_STEP_IDLE )
	{
	    pdev->chgr.step = CHGR_STEP_CHK_BAT_REVERSE;
	}
	
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
    
    		// 清零计数器
    		pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] = 0;
    	}
    	// 如果检测未通过 
    	else if( pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_REVERSE] <= (int32_t)( FILTER_CHK_BAT_REVERSE * (-1) ) )
    	{
    		// 设置电池反接标志
    		pdev->basic.bat = BASIC_BAT_REVERSE;
    
    		
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
    		
    		// 清零计数器
            pdev->chgr.cnt_box[IDX_CNT_CHK_BAT_CONN] = 0;
    
    		// 返回空闲状态，重新检测
    	    pdev->chgr.step = CHGR_STEP_IDLE;
    	}
    }
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





