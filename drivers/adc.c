#include "adc.h"
#include "uart.h"

#define    MS_BASE    10280ul    // 大约1ms
#define    MS(n)      ((MS_BASE)*(n))


#ifndef __LOG
#define    __LOG_DEBUG    0
#endif
#define LOG_DEBUG(fmt,arg...)         do{\
                                          if(__LOG_DEBUG)\
											  printf("\r\n<<-DEBUG_DRIVER->> "fmt"\n",##arg);\
                                          }while(0)

void adc_delay_us( __IO uint32_t _T )
{
	uint32_t i;
	while ( _T-- )
		for ( i = 0; i < 11; i++);
}										  
										  
//__IO static uint16_t g_u16_adc_converted_value_single = 0;
__IO static uint16_t g_u16_adc_converted_value[ NOFCHANEL ] = {0};
										  
static void adc_gpio_config( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 打开 ADC IO端口时钟
	ADC_GPIO_APBxClock_FUN ( ADC_GPIO_1_CLK | ADC_GPIO_3_CLK, ENABLE );
	
	// 配置 ADC IO 引脚模式
	GPIO_InitStructure.GPIO_Pin =   ADC_PIN3|
									ADC_PIN4|
									ADC_PIN5|
									ADC_PIN6|
									ADC_PIN7|
									ADC_PIN8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	
	GPIO_Init(ADC_PORT_1, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =	ADC_PIN1 | ADC_PIN2;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AIN;	
	GPIO_Init(ADC_PORT_3, &GPIO_InitStructure);
}

#if 0
static void adc_mode_config( void )
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	// 打开DMA时钟
	RCC_AHBPeriphClockCmd(ADC_DMA_CLK, ENABLE);
	// 打开ADC时钟
	ADC_APBxClock_FUN ( ADC_CLK, ENABLE );
	
	// 复位DMA控制器
	DMA_DeInit(ADC_DMA_CHANNEL);
	
	// 配置 DMA 初始化结构体
	// 外设基址为：ADC 数据寄存器地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = ( u32 ) ( & ( ADC_x->DR ) );
	
	// 存储器地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&g_u16_adc_converted_value_single;
	
	// 数据源来自外设
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	
	// 缓冲区大小，应该等于数据目的地的大小
	DMA_InitStructure.DMA_BufferSize = NOFCHANEL;
	
	// 外设寄存器只有一个，地址不用递增
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

	// 存储器地址固定
	DMA_InitStructure.DMA_MemoryInc = DMA_PeripheralInc_Disable; 
	
	// 外设数据大小为半字，即两个字节
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	
	// 内存数据大小也为半字，跟外设数据大小相同
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	
	// 循环传输模式
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	

	// DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	
	// 禁止存储器到存储器模式，因为是从外设到存储器
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	// 初始化DMA
	DMA_Init(ADC_DMA_CHANNEL, &DMA_InitStructure);
	
	// 使能 DMA 通道
	DMA_Cmd(ADC_DMA_CHANNEL , ENABLE);
	
	// ADC 模式配置
	// 只使用一个ADC，属于单模式
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;

	// 只用一个通道，不用扫描模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 

	// 连续转换模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;

	// 不用外部触发转换，软件开启即可
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;

	// 转换结果右对齐
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	
	// 转换通道个数
	ADC_InitStructure.ADC_NbrOfChannel = NOFCHANEL;	
		
	// 初始化ADC
	ADC_Init(ADC_x, &ADC_InitStructure);
	
	// 配置ADC时钟Ｎ狿CLK2的8分频，即9MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	
	// 配置ADC 通道的转换顺序和采样时间
//	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL1, 1, ADC_SampleTime_55Cycles5);
//	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL2, 2, ADC_SampleTime_55Cycles5);
//	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL3, 3, ADC_SampleTime_55Cycles5);
//	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL4, 4, ADC_SampleTime_55Cycles5);
//	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL5, 5, ADC_SampleTime_55Cycles5);
//	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL6, 6, ADC_SampleTime_55Cycles5);
	
	// 使能ADC DMA 请求
	ADC_DMACmd(ADC_x, ENABLE);
	
	// 开启ADC ，并开始转换
	ADC_Cmd(ADC_x, ENABLE);
	
	// 初始化ADC 校准寄存器  
	ADC_ResetCalibration(ADC_x);
	// 等待校准寄存器初始化完成
	while(ADC_GetResetCalibrationStatus(ADC_x));
	
	// ADC开始校准
	ADC_StartCalibration(ADC_x);
	// 等待校准完成
	while(ADC_GetCalibrationStatus(ADC_x));
	
	// 由于没有采用外部触发，所以使用软件触发ADC转换 
//	ADC_SoftwareStartConvCmd(ADC_x, ENABLE);	

	// 默认关闭adc	
	ADC_Cmd(ADC_x, DISABLE);
}

#endif

#if 1
static void adc_mode_config( void )
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	// 打开DMA时钟
	RCC_AHBPeriphClockCmd(ADC_DMA_CLK, ENABLE);
	// 打开ADC时钟
	ADC_APBxClock_FUN ( ADC_CLK, ENABLE );
	
	// 复位DMA控制器
	DMA_DeInit(ADC_DMA_CHANNEL);
	
	// 配置 DMA 初始化结构体
	// 外设基址为：ADC 数据寄存器地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = ( u32 ) ( & ( ADC_x->DR ) );
	
	// 存储器地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_u16_adc_converted_value;
	
	// 数据源来自外设
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	
	// 缓冲区大小，应该等于数据目的地的大小
	DMA_InitStructure.DMA_BufferSize = NOFCHANEL;
	
	// 外设寄存器只有一个，地址不用递增
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

	// 存储器地址递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	
	// 外设数据大小为半字，即两个字节
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	
	// 内存数据大小也为半字，跟外设数据大小相同
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	
	// 循环传输模式
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	

	// DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	
	// 禁止存储器到存储器模式，因为是从外设到存储器
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	// 初始化DMA
	DMA_Init(ADC_DMA_CHANNEL, &DMA_InitStructure);
	
	// 使能 DMA 通道
	DMA_Cmd(ADC_DMA_CHANNEL , ENABLE);
	
	// ADC 模式配置
	// 只使用一个ADC，属于单模式
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;

	// 开启扫描模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 

	// 连续转换模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;

	// 不用外部触发转换，软件开启即可
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;

	// 转换结果右对齐
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	
	// 转换通道个数
	ADC_InitStructure.ADC_NbrOfChannel = NOFCHANEL;	
		
	// 初始化ADC
	ADC_Init(ADC_x, &ADC_InitStructure);
	
	// 配置ADC时钟Ｎ狿CLK2的8分频，即9MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	
	// 配置ADC 通道的转换顺序和采样时间
	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL1, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL2, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL3, 3, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL4, 4, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL5, 5, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL6, 6, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL7, 7, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC_x, ADC_CHANNEL8, 8, ADC_SampleTime_55Cycles5);
	
	// 使能ADC DMA 请求
	ADC_DMACmd(ADC_x, ENABLE);
	
	// 开启ADC ，并开始转换
	ADC_Cmd(ADC_x, ENABLE);
	
	// 初始化ADC 校准寄存器  
	ADC_ResetCalibration(ADC_x);
	// 等待校准寄存器初始化完成
	while(ADC_GetResetCalibrationStatus(ADC_x));
	
	// ADC开始校准
	ADC_StartCalibration(ADC_x);
	// 等待校准完成
	while(ADC_GetCalibrationStatus(ADC_x));
	
	// 由于没有采用外部触发，所以使用软件触发ADC转换 
	ADC_SoftwareStartConvCmd(ADC_x, ENABLE);	
}
#endif

void adc_init( void )
{
	adc_gpio_config();
    adc_mode_config();

	// test
//	adc_start_smp( 0 );
//	printf("\r\nconver value:%d, smp value:%f", g_u16_adc_converted_value_single, adc_get_smp_value() );
//	adc_stop_smp();
}

static void adc_start_smp( uint8_t num )
{
	uint8_t chl = 0;

    // 设置转换通道
	switch ( num )
	{
	    case 0:
			chl = ADC_CHANNEL1;
			break;
	    case 1:
			chl = ADC_CHANNEL2;
			break;
	    case 2:
			chl = ADC_CHANNEL3;
			break;
	    case 3:
			chl = ADC_CHANNEL4;
			break;
	    case 4:
			chl = ADC_CHANNEL5;
			break;
	    case 5:
			chl = ADC_CHANNEL6;
			break;
	    case 6:
			chl = ADC_CHANNEL7;
			break;
	    case 7:
			chl = ADC_CHANNEL8;
			break;
		default:
			break;
	}

	ADC_RegularChannelConfig(ADC_x, chl, 1, ADC_SampleTime_55Cycles5);

	// 开启adc并开始转换
	ADC_Cmd(ADC_x, ENABLE);

	// 软件触发ADC转换 
	ADC_SoftwareStartConvCmd(ADC_x, ENABLE);	

	// 检测启动标志
	while( ADC_GetFlagStatus( ADC_x, ADC_FLAG_STRT ) == RESET );
}

static void adc_stop_smp(void)
{
	ADC_Cmd(ADC_x, DISABLE);
}

#if 0
static uint16_t _adc_get_convert_value( uint8_t num )
{
	uint32_t time = 0;
	uint16_t ret = 0;
	uint32_t filter_value = 0;

	// 启动对应通道的转换
	adc_start_smp( num );

    // 检测转换完成，添加超时检测机制
    while( DMA_GetFlagStatus( DMA1_FLAG_TC1 ) == RESET )
	{
		if( time >= MS(5) )
		{
			LOG_DEBUG("\r\nsmp %d timeout!(0-base)", num);
			g_u16_adc_converted_value_single = 0;
			adc_stop_smp();
		    return 0;
		}
		time ++;
	}
    DMA_ClearFlag( DMA1_FLAG_TC1 );
	LOG_DEBUG("\r\nsmp %d(0-base) finish, smp convert value:0x%04X, time cnt:%lu", num, g_u16_adc_converted_value_single, time);

	// 停止
    adc_stop_smp();

    ret = g_u16_adc_converted_value_single;

    g_u16_adc_converted_value_single = 0;

	return ( ret );
}

#define    ADC_FILTER_CNT    100
uint16_t adc_get_convert_value( uint8_t num )
{
	uint16_t idx = 0;
	uint32_t sum = 0;
	uint16_t smp = 0;

	for( idx = 0; idx < ADC_FILTER_CNT; idx ++ )
	{
	   smp = _adc_get_convert_value( num );
//	   smp = g_u16_adc_converted_value[ num ];
	   sum += smp;
	}

	return ( sum / ADC_FILTER_CNT );
}


float adc_get_smp_value( uint8_t num )
{
	uint32_t time = 0;
	float ret = 0;

	// 启动对应通道的转换
	adc_start_smp( num );

    // 检测转换完成，添加超时检测机制
    while( DMA_GetFlagStatus( DMA1_FLAG_TC1 ) == RESET )
	{
		if( time >= MS(5) )
		{
			LOG_DEBUG("\r\nsmp %d timeout!(0-base)", num);
			g_u16_adc_converted_value_single = 0;
			adc_stop_smp();
		    return 0;
		}
		time ++;
	}
    DMA_ClearFlag( DMA1_FLAG_TC1 );
	LOG_DEBUG("\r\nsmp %d(0-base) finish, smp convert value:0x%04X, time cnt:%lu", num, g_u16_adc_converted_value_single, time);

	// 停止
    adc_stop_smp();

	ret = ( (float)g_u16_adc_converted_value_single / 4096 ) * 3.3;
    g_u16_adc_converted_value_single = 0;
	LOG_DEBUG("\r\nsmp:%f", ret);
	
	return ( ret );
}
#endif


#define    ADC_FILTER_CNT    100
uint16_t adc_get_convert_value( uint8_t num )
{
	uint16_t idx = 0;
	uint32_t sum = 0;
	uint16_t smp = 0;

	for( idx = 0; idx < ADC_FILTER_CNT; idx ++ )
	{
		smp = g_u16_adc_converted_value[ num ];
		sum += smp;
		adc_delay_us(10);
	}

	return ( sum / ADC_FILTER_CNT );
}

uint16_t adc_get_convert_value_single( uint8_t num )
{
    return g_u16_adc_converted_value[ num ];
}

float adc_get_smp_value( uint8_t num )
{
	uint32_t time = 0;
	float ret = 0;
	
	ret = ( (float)g_u16_adc_converted_value[num] / 4096 ) * 3.3;
	LOG_DEBUG("\r\nsmp:%f", ret);
	
	return ( ret );
}
