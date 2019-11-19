/*
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-21     YangLi       1.����豸��ʼ������ջ���������; 
                               2.�޸�serial_send()�У���鷢�����ʱ����д���󣬰�uartд����uart4����������uart���ͳ�����
 * 2019-01-15     YangLi       1.�޸�gnuc�����£�printf��ӡ�����⣬��Ҫ�ڵ���printf�󣬼���fflush(stdout);
 *                             2.���printf֧�ָ�������ӡ, -u _printf_float
 * 2019-02-22     YangLi       1.��Ӷ���init�ӿڣ������޸Ĵ�������
 */

#include <stm32f10x.h>
#include "uart.h"

// macro
#define        MS_BASE        			10280      // 1ms, 72MHz
#define        MS_MAX					5
#define        MS_WAIT_SEND_FINISH      ( (MS_BASE)*(100) )
#define        UART_MAX_WAIT_TIME		( (MS_BASE)*(MS_MAX) )     // 5ms

// for ext wifi
#define			USART1_RX_BUFFER_SIZE	128
#define			USART1_TX_BUFFER_SIZE	1024

// for touch_screen
#define			USART2_RX_BUFFER_SIZE	50
#define			USART2_TX_BUFFER_SIZE	100

// for debug
#define			UART4_RX_BUFFER_SIZE	100
#define			UART4_TX_BUFFER_SIZE	100

uint8_t g_usart2_tx_dma_buffer[ USART2_TX_BUFFER_SIZE ];


// variable
uart_dev dev_usart1;
uart_dev dev_usart2;
uart_dev dev_usart3;
uart_dev dev_uart4;

uart_dev * usart1 = &dev_usart1;
uart_dev * usart2 = &dev_usart2;
uart_dev * usart3 = &dev_usart3;
uart_dev * uart4 = &dev_uart4;

uint8_t usart1_rx_buffer[USART1_RX_BUFFER_SIZE];
uint8_t usart1_tx_buffer[USART1_TX_BUFFER_SIZE];

uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
uint8_t usart2_tx_buffer[USART2_TX_BUFFER_SIZE];

uint8_t uart4_rx_buffer[UART4_RX_BUFFER_SIZE];
uint8_t uart4_tx_buffer[UART4_TX_BUFFER_SIZE];

/****************************************************************************
                              private funciton
 ****************************************************************************/
// usart1 info
static void usart1_send_byte( uint8_t byte );
static void usart1_send_halfword( uint16_t ch );
static void usart1_send_string( char *str);

static uint8_t usart1_read(void);
static uint16_t usart1_read_word(void);
static void usart1_write( uint8_t data );
static void usart1_send(void);
static uint16_t usart1_get_tx_cnt(void);
static uint16_t usart1_get_rx_cnt(void);
static void usart1_reset_rx(void);

static void usart1_drv_init(void);

// usart2 info
static void usart2_send_byte( uint8_t byte );
static void usart2_send_halfword( uint16_t ch );
static void usart2_send_string( char *str);

static uint8_t usart2_read(void);
static uint16_t usart2_read_word(void);
static void usart2_write( uint8_t data );
static void usart2_send(void);
static uint16_t usart2_get_tx_cnt(void);
static uint16_t usart2_get_rx_cnt(void);
static void usart2_reset_rx(void);

static void usart2_drv_init(void);

// uart4 info
static void uart4_send_byte( uint8_t byte );
static void uart4_send_halfword( uint16_t ch );
static void uart4_send_string( char *str);

static uint8_t uart4_read(void);
static uint16_t uart4_read_word(void);
static void uart4_write( uint8_t data );
static void uart4_send(void);
static uint16_t uart4_get_tx_cnt(void);
static uint16_t uart4_get_rx_cnt(void);
static void uart4_reset_rx(void);

static void uart4_drv_init(void);


static void send_buffer( uart_dev *uart );
static void serial_send( uart_dev *uart );

static void chk_send_status( USART_TypeDef *port, uint16_t flag, FlagStatus status);
// Writes one byte to the TX serial buffer. Called by main program.
static void serial_write( uart_dev *uart, uint8_t data);
// Fetches the first byte in the serial read buffer. Called by main program.
static uint8_t serial_read( uart_dev *uart );
// Reset and empty data in read buffer. Used by e-stop and reset.
static void serial_reset_read_buffer( uart_dev *uart );
// Reset and empty data in send buffer. Used by e-stop and reset.
static void serial_reset_send_buffer( uart_dev *uart );
// Returns the number of bytes used in the RX serial buffer.
// before serial_read, call for once!!!
static uint16_t serial_get_rx_buffer_count( uart_dev *uart );
// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
static uint16_t serial_get_tx_buffer_count( uart_dev *uart );
static uint8_t serial_read_tx( uart_dev *uart );
	
static void delay_ms_uart( __IO uint32_t _T );

static void uart_drv_init( uart_dev *uart )
{
    assert_param( uart != NULL );
	
// ���ô���
   USART_DeInit( uart->def.port_uart );

/* ��ʼ����������
 */
// ���ø���
	if ( uart->def.state_gpio_af == ENABLE )
	{
		GPIO_PinRemapConfig( uart->def.af_remap_x, ENABLE );
	}
// ����GPIO
	// �򿪴���GPIO��ʱ��
	uart->def.fun_clk_cmd_tx( uart->def.clk_tx_port, ENABLE );
	uart->def.fun_clk_cmd_rx( uart->def.clk_rx_port, ENABLE );
	// ��USART Tx��GPIO����Ϊ���츴��ģʽ
	GPIO_Init( uart->def.port_tx, &(uart->def.init_structure_tx) );
    // ��USART Rx��GPIO����Ϊ��������ģʽ
	GPIO_Init( uart->def.port_rx, &(uart->def.init_structure_rx) );
	
// ���ô���
	// �򿪴��������ʱ��
	uart->def.fun_clk_cmd_uart( uart->def.clk_uart, ENABLE );
	// ��ɴ��ڵĳ�ʼ������
	USART_Init(uart->def.port_uart, &(uart->def.init_structure_uart) );
	
// ����Ƕ�������жϿ�����nvic
	if ( uart->def.init_structure_nvic.NVIC_IRQChannelCmd == ENABLE )
	{
		NVIC_PriorityGroupConfig( uart->def.nvic_group );
		/* ��ʼ������NVIC */
		NVIC_Init( &(uart->def.init_structure_nvic) );
	}
// ʹ�ܴ����ж�
	if ( uart->def.state_rx_int == ENABLE )
	{
		USART_ITConfig( uart->def.port_uart, USART_IT_RXNE, ENABLE );
		USART_ITConfig( uart->def.port_uart, USART_IT_IDLE, ENABLE );
	}
	else
	{
		USART_ITConfig( uart->def.port_uart, USART_IT_RXNE, DISABLE );
		USART_ITConfig( uart->def.port_uart, USART_IT_IDLE, DISABLE );	
	}
	
	// ʹ�ܴ���
	USART_Cmd(uart->def.port_uart, ENABLE);		

    // ���������ɱ�־
	USART_ClearFlag(uart->def.port_uart, USART_FLAG_TC);  
}

void dma_usart1_tx(uint8_t *res, uint32_t size)
{
	// ���TC��־
    USART_ClearFlag(USART1, USART_FLAG_TC);

// ��������DMA������ǰ��Ҫ�Ƚ��ò���������Ч
    // ����DMA
    DMA_Cmd(USART_1_TX_DMA_CHANNEL, DISABLE);

    // ����DMA�Ĵ���ֵ
    USART_1_TX_DMA_CHANNEL->CNDTR = size;

    // ���ô����ַ
    USART_1_TX_DMA_CHANNEL->CMAR  = (uint32_t)res;

    // ����DMA
    DMA_Cmd(USART_1_TX_DMA_CHANNEL, ENABLE);

// �������ڵ�DMA����
	// ������DMA
    USART_DMACmd(USART_1, USART_DMAReq_Tx, ENABLE);
}

static void usart1_dma_init(void)
{
    DMA_InitTypeDef DMA_InitStructure;

#if 0
    //DMA�����ж�����
    NVIC_InitStructure.NVIC_IRQChannel = USART_1_IRQ_DMA;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

	// ����TX_DMAͨ��
	DMA_DeInit( USART_1_TX_DMA_CHANNEL );

	// ����DMAʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	// ����DMAԴ��ַ���������ݼĴ�����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART_1_DR_ADDRESS;
	// �ڴ��ַ(Ҫ����ı�����ָ��)
	DMA_InitStructure.DMA_MemoryBaseAddr = 0;
	// ���򣺴��ڴ浽����	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	// �����С	
	DMA_InitStructure.DMA_BufferSize = 0;
	// �����ַ����	    
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	// �ڴ��ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	// �������ݵ�λ	
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	// �ڴ����ݵ�λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
	// DMAģʽ��һ�λ���ѭ��ģʽ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
	//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	
	// ���ȼ�����	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 
	// ��ֹ�ڴ浽�ڴ�Ĵ���
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	// ����DMAͨ��		   
	DMA_Init(USART_1_TX_DMA_CHANNEL, &DMA_InitStructure);		
	// ʹ��DMA
	DMA_Cmd (USART_1_TX_DMA_CHANNEL,ENABLE);
}

void dma_usart2_tx(uint8_t *res, uint32_t size)
{
	// ���TC��־
    USART_ClearFlag(USART2, USART_FLAG_TC);

// ��������DMA������ǰ��Ҫ�Ƚ��ò���������Ч
    // ����DMA
    DMA_Cmd(USART_2_TX_DMA_CHANNEL, DISABLE);

    // ����DMA�Ĵ���ֵ
    USART_2_TX_DMA_CHANNEL->CNDTR = size;

    // ���ô����ַ
    USART_2_TX_DMA_CHANNEL->CMAR  = (uint32_t)res;

    // ����DMA
    DMA_Cmd(USART_2_TX_DMA_CHANNEL, ENABLE);

// �������ڵ�DMA����
	// ������DMA
    USART_DMACmd(USART_2, USART_DMAReq_Tx, ENABLE);
}

static void usart2_dma_init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// ����TX_DMAͨ��
	DMA_DeInit( USART_2_TX_DMA_CHANNEL );
	
#if 1
    //DMA�����ж�����
    NVIC_InitStructure.NVIC_IRQChannel = USART_2_IRQ_DMA;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig( USART_2_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE );
#endif
	
	// ����DMAʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	// ����DMAԴ��ַ���������ݼĴ�����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART_2_DR_ADDRESS;
	// �ڴ��ַ(Ҫ����ı�����ָ��)
	DMA_InitStructure.DMA_MemoryBaseAddr = 0;
	// ���򣺴��ڴ浽����	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	// �����С	
	DMA_InitStructure.DMA_BufferSize = 0;
	// �����ַ����	    
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	// �ڴ��ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	// �������ݵ�λ	
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	// �ڴ����ݵ�λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
	// DMAģʽ��һ�λ���ѭ��ģʽ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
	//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	
	// ���ȼ�����	
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 
	// ��ֹ�ڴ浽�ڴ�Ĵ���
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	// ����DMAͨ��		   
	DMA_Init(USART_2_TX_DMA_CHANNEL, &DMA_InitStructure);		
	// ʹ��DMA
	DMA_Cmd (USART_2_TX_DMA_CHANNEL,ENABLE);
}

static void init_usart1(void)
{	
	usart1->def.port_uart = USART_1;
	usart1->def.port_tx = USART_1_TX_GPIO_PORT;
	usart1->def.port_rx = USART_1_RX_GPIO_PORT;
	usart1->def.clk_uart = USART_1_CLK;
	usart1->def.clk_tx_port = USART_1_GPIO_CLK;
	usart1->def.clk_rx_port = USART_1_GPIO_CLK;
	usart1->def.init_structure_tx.GPIO_Pin = USART_1_TX_GPIO_PIN;
	usart1->def.init_structure_tx.GPIO_Mode = GPIO_Mode_AF_PP;
	usart1->def.init_structure_tx.GPIO_Speed = GPIO_Speed_50MHz;
	usart1->def.init_structure_rx.GPIO_Pin = USART_1_RX_GPIO_PIN;
	usart1->def.init_structure_rx.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	usart1->def.init_structure_uart.USART_BaudRate = USART_1_BAUDRATE;
	usart1->def.init_structure_uart.USART_WordLength = USART_WordLength_8b;
	usart1->def.init_structure_uart.USART_StopBits = USART_StopBits_1;
	usart1->def.init_structure_uart.USART_Parity = USART_Parity_No;
	usart1->def.init_structure_uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	usart1->def.init_structure_uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart1->def.init_structure_uart.USART_Mode = USART_Mode_Tx; // Ĭ�ϲ����գ���ʼ����ɺ��ٽ���
    usart1->def.init_structure_nvic.NVIC_IRQChannel = USART_1_IRQ;
	usart1->def.init_structure_nvic.NVIC_IRQChannelPreemptionPriority = USART_1_IRQ_PRIO;
//	usart1->def.init_structure_nvic.NVIC_IRQChannelSubPriority = 1;
	usart1->def.init_structure_nvic.NVIC_IRQChannelCmd = ENABLE;
	usart1->def.fun_clk_cmd_tx = USART_1_GPIO_APBxClkCmd;
	usart1->def.fun_clk_cmd_rx = USART_1_GPIO_APBxClkCmd;
	usart1->def.fun_clk_cmd_uart = USART_1_APBxClkCmd;
	usart1->def.state_rx_int = DISABLE; // �����ж�Ĭ�Ϲرգ���ϵͳ��ʼ������ٴ�
	usart1->def.state_gpio_af = DISABLE;
	usart1->def.nvic_group = NVIC_PriorityGroup_4;
	
	uart_drv_init( usart1 );

    usart1_dma_init();

#ifdef _SUPPORT_OS
    usart1->semaphore = xSemaphoreCreateBinary();
    usart1->mutex = xSemaphoreCreateMutex();
#endif

	usart1->init = usart1_drv_init;

	usart1->send_byte = usart1_send_byte;
	usart1->send_halfword =  usart1_send_halfword;
	usart1->send_string = usart1_send_string;
	
	usart1->read = usart1_read;
	usart1->read_word = usart1_read_word;
	usart1->write = usart1_write;
	usart1->send = usart1_send;
	usart1->get_rx_cnt = usart1_get_rx_cnt;
	usart1->get_tx_cnt = usart1_get_tx_cnt;
	usart1->reset_rx = usart1_reset_rx;
	
	// ��ʼ��������
	usart1->buffer.rx = usart1_rx_buffer;
	usart1->buffer.rx_size = USART1_RX_BUFFER_SIZE;
	usart1->buffer.rx_head = 0;
	usart1->buffer.rx_tail = 0;
	
	usart1->buffer.tx = usart1_tx_buffer;
	usart1->buffer.tx_size = USART1_TX_BUFFER_SIZE;
	usart1->buffer.tx_head = 0;
	usart1->buffer.tx_tail = 0;
}

static void init_usart2(void)
{	
	usart2->def.port_uart = USART_2;
	usart2->def.port_tx = USART_2_TX_GPIO_PORT;
	usart2->def.port_rx = USART_2_RX_GPIO_PORT;
	usart2->def.clk_uart = USART_2_CLK;
	usart2->def.clk_tx_port = USART_2_GPIO_CLK;
	usart2->def.clk_rx_port = USART_2_GPIO_CLK;
	usart2->def.init_structure_tx.GPIO_Pin = USART_2_TX_GPIO_PIN;
	usart2->def.init_structure_tx.GPIO_Mode = GPIO_Mode_AF_PP;
	usart2->def.init_structure_tx.GPIO_Speed = GPIO_Speed_50MHz;
	usart2->def.init_structure_rx.GPIO_Pin = USART_2_RX_GPIO_PIN;
	usart2->def.init_structure_rx.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	usart2->def.init_structure_uart.USART_BaudRate = USART_2_BAUDRATE;
	usart2->def.init_structure_uart.USART_WordLength = USART_WordLength_8b;
	usart2->def.init_structure_uart.USART_StopBits = USART_StopBits_1;
	usart2->def.init_structure_uart.USART_Parity = USART_Parity_No;
	usart2->def.init_structure_uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	usart2->def.init_structure_uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart2->def.init_structure_uart.USART_Mode = USART_Mode_Tx; // Ĭ�ϲ����գ���ʼ����ɺ��ٽ���
    usart2->def.init_structure_nvic.NVIC_IRQChannel = USART_2_IRQ;
	usart2->def.init_structure_nvic.NVIC_IRQChannelPreemptionPriority = USART_2_IRQ_PRIO;
//	usart2->def.init_structure_nvic.NVIC_IRQChannelSubPriority = 1;
	usart2->def.init_structure_nvic.NVIC_IRQChannelCmd = ENABLE;
	usart2->def.fun_clk_cmd_tx = USART_2_GPIO_APBxClkCmd;
	usart2->def.fun_clk_cmd_rx = USART_2_GPIO_APBxClkCmd;
	usart2->def.fun_clk_cmd_uart = USART_2_APBxClkCmd;
	usart2->def.state_rx_int = DISABLE; // �����ж�Ĭ�Ϲرգ���ϵͳ��ʼ������ٴ�
	usart2->def.state_gpio_af = DISABLE;
	usart2->def.nvic_group = NVIC_PriorityGroup_4;
	
	uart_drv_init( usart2 );

//    usart2_dma_init();
	
#ifdef _SUPPORT_OS
    usart2->semaphore = xSemaphoreCreateBinary();
    usart2->mutex = xSemaphoreCreateMutex();
#endif

	usart2->init = usart2_drv_init;	
	
	usart2->send_byte = usart2_send_byte;
	usart2->send_halfword =  usart2_send_halfword;
	usart2->send_string = usart2_send_string;
	
	usart2->read = usart2_read;
	usart2->read_word = usart2_read_word;
	usart2->write = usart2_write;
	usart2->send = usart2_send;
	usart2->get_rx_cnt = usart2_get_rx_cnt;
	usart2->get_tx_cnt = usart2_get_tx_cnt;
	usart2->reset_rx = usart2_reset_rx;
	
	// ��ʼ��������
	usart2->buffer.rx = usart2_rx_buffer;
	usart2->buffer.rx_size = USART2_RX_BUFFER_SIZE;
	usart2->buffer.rx_head = 0;
	usart2->buffer.rx_tail = 0;
	
	usart2->buffer.tx = usart2_tx_buffer;
	usart2->buffer.tx_size = USART2_TX_BUFFER_SIZE;
	usart2->buffer.tx_head = 0;
	usart2->buffer.tx_tail = 0;
	
//	usart2->flag_tx = e_tx_idle;
}




static void init_uart4(void)
{
	uart4->def.port_uart = UART_4;
	uart4->def.port_tx = UART_4_TX_GPIO_PORT;
	uart4->def.port_rx = UART_4_RX_GPIO_PORT;
	uart4->def.clk_uart = UART_4_CLK;
	uart4->def.clk_tx_port = UART_4_GPIO_CLK;
	uart4->def.clk_rx_port = UART_4_GPIO_CLK;
	uart4->def.init_structure_tx.GPIO_Pin = UART_4_TX_GPIO_PIN;
	uart4->def.init_structure_tx.GPIO_Mode = GPIO_Mode_AF_PP;
	uart4->def.init_structure_tx.GPIO_Speed = GPIO_Speed_50MHz;
	uart4->def.init_structure_rx.GPIO_Pin = UART_4_RX_GPIO_PIN;
	uart4->def.init_structure_rx.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	uart4->def.init_structure_uart.USART_BaudRate = UART_4_BAUDRATE;
	uart4->def.init_structure_uart.USART_WordLength = USART_WordLength_8b;
	uart4->def.init_structure_uart.USART_StopBits = USART_StopBits_1;
	uart4->def.init_structure_uart.USART_Parity = USART_Parity_No;
	uart4->def.init_structure_uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	uart4->def.init_structure_uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	uart4->def.init_structure_uart.USART_Mode = USART_Mode_Tx; // Ĭ�ϲ����գ���ʼ����ɺ��ٽ���
    uart4->def.init_structure_nvic.NVIC_IRQChannel = UART_4_IRQ;
	uart4->def.init_structure_nvic.NVIC_IRQChannelPreemptionPriority = UART_4_IRQ_PRIO;
//	uart4->def.init_structure_nvic.NVIC_IRQChannelSubPriority = 1;
	uart4->def.init_structure_nvic.NVIC_IRQChannelCmd = ENABLE;
	uart4->def.fun_clk_cmd_tx = UART_4_GPIO_APBxClkCmd;
	uart4->def.fun_clk_cmd_rx = UART_4_GPIO_APBxClkCmd;
	uart4->def.fun_clk_cmd_uart = UART_4_APBxClkCmd;
	uart4->def.state_rx_int = DISABLE; // �����ж�Ĭ�Ϲرգ���ϵͳ��ʼ������ٴ�
	uart4->def.state_gpio_af = DISABLE;
	uart4->def.nvic_group = NVIC_PriorityGroup_4;

	uart_drv_init( uart4 );
	
#ifdef _SUPPORT_OS
    uart4->semaphore = xSemaphoreCreateBinary();
    uart4->mutex = xSemaphoreCreateMutex();
#endif
	
	uart4->init = uart4_drv_init;	

	uart4->send_byte = uart4_send_byte;
	uart4->send_halfword = uart4_send_halfword;
	uart4->send_string = uart4_send_string;
	
	uart4->read = uart4_read;
	uart4->read_word = uart4_read_word;
	uart4->write = uart4_write;
	uart4->send = uart4_send;
	uart4->get_rx_cnt = uart4_get_rx_cnt;
	uart4->get_tx_cnt = uart4_get_tx_cnt;
	uart4->reset_rx = uart4_reset_rx;
	
	// ��ʼ��������
	uart4->buffer.rx = uart4_rx_buffer;
	uart4->buffer.rx_size = UART4_RX_BUFFER_SIZE;
	uart4->buffer.rx_head = 0;
	uart4->buffer.rx_tail = 0;
	
	uart4->buffer.tx = uart4_tx_buffer;
	uart4->buffer.tx_size = UART4_TX_BUFFER_SIZE;
	uart4->buffer.tx_head = 0;
	uart4->buffer.tx_tail = 0;
}


static void usart1_drv_init(void)
{
    uart_drv_init( usart1 );

	serial_reset_read_buffer( usart1 );
	serial_reset_send_buffer( usart1 );
}

static void usart1_send_byte( uint8_t byte )
{
#ifdef _SUPPORT_OS
	if ( usart1->mutex != NULL )
	{
		xSemaphoreTake( usart1->mutex, 100 );
		
		USART_SendData( usart1->def.port_uart, byte );
		chk_send_status( usart1->def.port_uart, USART_FLAG_TXE, RESET );
		
		xSemaphoreGive( usart1->mutex );
	}
#else
	
	USART_SendData( usart1->def.port_uart, byte );
	chk_send_status( usart1->def.port_uart, USART_FLAG_TXE, RESET );
#endif
}

static void usart1_send_halfword( uint16_t ch )
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
		
	/* ���͸߰�λ */
	USART_SendData( usart1->def.port_uart, temp_h );	
	chk_send_status( usart1->def.port_uart, USART_FLAG_TXE, RESET );
	
	/* ���͵Ͱ�λ */
	USART_SendData( usart1->def.port_uart, temp_l );	
	chk_send_status( usart1->def.port_uart, USART_FLAG_TXE, RESET );
}

static void usart1_send_string( char *str)
{
    unsigned int k=0;
#ifdef _SUPPORT_OS
	if ( usart1->mutex != NULL )
	{
		xSemaphoreTake( usart1->mutex, 100 );
#endif
	
    do 
    {
        Usart_SendByte( usart1->def.port_uart, *(str + k) );
        k++;
    } while(*(str + k)!='\0');

    /* �ȴ�������� */
//    while(USART_GetFlagStatus( usart1->def.port_uart,USART_FLAG_TC )==RESET)
//    {}
	chk_send_status( usart1->def.port_uart, USART_FLAG_TC, RESET );
		
#ifdef _SUPPORT_OS
		xSemaphoreGive( usart1->mutex );
	}
#endif
}

static void usart2_drv_init(void)
{
    uart_drv_init( usart2 );
	serial_reset_read_buffer( usart2 );
    serial_reset_send_buffer( usart2 );
}

static void usart2_send_byte( uint8_t byte )
{
#ifdef _SUPPORT_OS
	if ( usart2->mutex != NULL )
	{
		xSemaphoreTake( usart2->mutex, 100 );
		
		USART_SendData( usart2->def.port_uart, byte );
		chk_send_status( usart2->def.port_uart, USART_FLAG_TXE, RESET );
		
		xSemaphoreGive( usart2->mutex );
	}
#else
	
	USART_SendData( usart2->def.port_uart, byte );
	chk_send_status( usart2->def.port_uart, USART_FLAG_TXE, RESET );
#endif
}

static void usart2_send_halfword( uint16_t ch )
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
		
	/* ���͸߰�λ */
	USART_SendData( usart2->def.port_uart, temp_h );	
	chk_send_status( usart2->def.port_uart, USART_FLAG_TXE, RESET );
	
	/* ���͵Ͱ�λ */
	USART_SendData( usart2->def.port_uart, temp_l );	
	chk_send_status( usart2->def.port_uart, USART_FLAG_TXE, RESET );
}

static void usart2_send_string( char *str)
{
    unsigned int k=0;
#ifdef _SUPPORT_OS
	if ( usart2->mutex != NULL )
	{
		xSemaphoreTake( usart2->mutex, 100 );
#endif
	
    do 
    {
        Usart_SendByte( usart2->def.port_uart, *(str + k) );
        k++;
    } while(*(str + k)!='\0');

    /* �ȴ�������� */
//    while(USART_GetFlagStatus( usart2->def.port_uart,USART_FLAG_TC )==RESET)
//    {}
	chk_send_status( usart2->def.port_uart, USART_FLAG_TC, RESET );
		
#ifdef _SUPPORT_OS
		xSemaphoreGive( usart2->mutex );
	}
#endif
}

static void uart4_drv_init(void)
{
    uart_drv_init( uart4 );
	serial_reset_read_buffer( uart4 );	
    serial_reset_send_buffer( uart4 );
}

static void uart4_send_byte( uint8_t byte )
{
#ifdef _SUPPORT_OS
	if( uart4->mutex != NULL)
	{
		xSemaphoreTake( uart4->mutex, 100 );
		
	    USART_SendData( uart4->def.port_uart, byte );
	    chk_send_status( uart4->def.port_uart, USART_FLAG_TXE, RESET );
		
		xSemaphoreGive( uart4->mutex );
	}
#else
	
	USART_SendData( uart4->def.port_uart, byte );
	chk_send_status( uart4->def.port_uart, USART_FLAG_TXE, RESET );
#endif
}

static void uart4_send_halfword( uint16_t ch )
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;

#ifdef _SUPPORT_OS
	if( uart4->mutex != NULL)
	{
		xSemaphoreTake( uart4->mutex, 100 );
		
		/* ���͸߰�λ */
		USART_SendData( uart4->def.port_uart, temp_h );	
		chk_send_status( uart4->def.port_uart, USART_FLAG_TXE, RESET );

		/* ���͵Ͱ�λ */
		USART_SendData( uart4->def.port_uart, temp_l );	
		chk_send_status( uart4->def.port_uart, USART_FLAG_TXE, RESET );	
		
		xSemaphoreGive( uart4->mutex );
	}
#else
	
	/* ���͸߰�λ */
	USART_SendData( uart4->def.port_uart, temp_h );	
	chk_send_status( uart4->def.port_uart, USART_FLAG_TXE, RESET );

	/* ���͵Ͱ�λ */
	USART_SendData( uart4->def.port_uart, temp_l );	
	chk_send_status( uart4->def.port_uart, USART_FLAG_TXE, RESET );	
#endif
}

static void uart4_send_string( char *str)
{
    unsigned int k=0;
	
#ifdef _SUPPORT_OS
	if( uart4->mutex != NULL)
	{
		xSemaphoreTake( uart4->mutex, 100 );
		
		do 
		{
			Usart_SendByte( uart4->def.port_uart, *(str + k) );
			/* �ȴ�������� */
			chk_send_status( uart4->def.port_uart, USART_FLAG_TC, RESET );
			k++;
		} while(*(str + k)!='\0');
		
		xSemaphoreGive( uart4->mutex );
	}
#else
    do 
	{
		Usart_SendByte( uart4->def.port_uart, *(str + k) );
		/* �ȴ�������� */
		chk_send_status( uart4->def.port_uart, USART_FLAG_TC, RESET );
		k++;
	} while(*(str + k)!='\0');	
#endif
}


/*
 * ��������Ƿ�����ɣ���ӳ�ʱ����
 * ��flag��״̬������statusʱ����ʾ���ݷ�����ɣ����������⣬ֱ�����ͨ�����߳�ʱ�������Ż᷵��
 * 
 * port:ָ���Ĵ��ڵ�ַ
 * flag:Ҫ����flag
 * status:ָ����flag�ĳ�ʼ״̬
 */
static void chk_send_status( USART_TypeDef *port, uint16_t flag, FlagStatus status)
{
	uint32_t timer = 0;

	do
	{
		timer ++;
	} while( (USART_GetFlagStatus( port, flag ) == status) && ( timer <= UART_MAX_WAIT_TIME ) );
}

static void send_buffer( uart_dev *uart )
{
	uint16_t len = serial_get_tx_buffer_count( uart );
//#ifdef _SUPPORT_OS
//	if ( uart->mutex != NULL )
//	{
//		xSemaphoreTake( uart->mutex, 100 );
//		while ( len -- )
//		{
//			serial_send ( uart );
//		}
//		xSemaphoreGive( uart->mutex );
//	}
//#else
	while ( len -- )
	{
		serial_send ( uart );
	}
//#endif
}

static void serial_send( uart_dev *uart )
{
	uint16_t tail = uart->buffer.tx_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)
	
	USART_SendData( uart->def.port_uart, uart->buffer.tx[tail] );
	chk_send_status( uart->def.port_uart, USART_FLAG_TXE, RESET );
	
//	uart4->send_byte( uart->buffer.tx[tail] ); // for debug
	
    tail++;
    if ( tail == uart->buffer.tx_size )
	{
		tail = 0;
	}
  
    uart->buffer.tx_tail = tail;
}


static uint8_t usart1_read(void)
{
	return serial_read( usart1 );
}
static uint16_t usart1_read_word(void)
{
	uint16_t tmp = 0;
	
	tmp = ( usart1_read() << 8 ) & 0xFF00;
    tmp |= ( usart1_read() & 0x00FF );

	return tmp;
}
static void usart1_write( uint8_t data )
{
	serial_write( usart1, data );
}
static void usart1_send(void)
{
	send_buffer( usart1 );
}
static uint16_t usart1_get_tx_cnt(void)
{
	return serial_get_tx_buffer_count( usart1 );
}
static uint16_t usart1_get_rx_cnt(void)
{
	return serial_get_rx_buffer_count( usart1 );
}
static void usart1_reset_rx(void)
{
	serial_reset_read_buffer( usart1 );
}


static uint8_t usart2_read(void)
{
	return serial_read( usart2 );
}
static uint16_t usart2_read_word(void)
{
	uint16_t tmp = 0;
	
	tmp = ( usart2_read() << 8 ) & 0xFF00;
    tmp |= ( usart2_read() & 0x00FF );

	return tmp;
}
static void usart2_write( uint8_t data )
{
#if 0
	uart_dev *uart = usart2;
	uint32_t timeout = MS_WAIT_SEND_FINISH;
	// �ȴ���һ�η�����ɣ���ʱ�ݶ�100ms
	while( (uart->flag_tx == e_tx_ing) && ( timeout ) )
		timeout --;
//	printf("\r\ntime:%u", timeout);
#endif
	
	serial_write( usart2, data );
}
static void usart2_send(void)
{
	send_buffer( usart2 );

#if 0
	uart_dev *uart = usart2;
	uint16_t len = serial_get_tx_buffer_count( uart );
	uint16_t idx;

    for(idx = 0; idx < len; idx ++ )
        g_usart2_tx_dma_buffer[idx] = serial_read_tx( uart );

	dma_usart2_tx( g_usart2_tx_dma_buffer, len );
	uart->flag_tx = e_tx_ing;
#endif
}
static uint16_t usart2_get_tx_cnt(void)
{
	return serial_get_tx_buffer_count( usart2 );
}
static uint16_t usart2_get_rx_cnt(void)
{
	return serial_get_rx_buffer_count( usart2 );
}
static void usart2_reset_rx(void)
{
	serial_reset_read_buffer( usart2 );
}

static uint8_t uart4_read(void)
{
	return serial_read( uart4 );
}
static uint16_t uart4_read_word(void)
{
	uint16_t tmp = 0;
	
	tmp = ( uart4_read() << 8 ) & 0xFF00;
	tmp |= ( uart4_read() & 0x00FF );
	
	return tmp;
}
static void uart4_write( uint8_t data )
{
	serial_write( uart4, data );
}
static void uart4_send(void)
{
	send_buffer( uart4 );
}
static uint16_t uart4_get_tx_cnt(void)
{
	return serial_get_tx_buffer_count( uart4 );
}
static uint16_t uart4_get_rx_cnt(void)
{
	return serial_get_rx_buffer_count( uart4 );
}
static void uart4_reset_rx(void)
{
	serial_reset_read_buffer( uart4 );
}

static void serial_write( uart_dev *uart, uint8_t data)
{
	// Calculate next head
	uint16_t next_head = uart->buffer.tx_head + 1;
	uint32_t timer = 0;
	BaseType_t xReturn = pdFALSE;
	
#ifdef _SUPPORT_OS
	if ( uart->mutex != NULL ) {
		if( pdFALSE == ( xReturn = xSemaphoreTake( uart->mutex, 100 ) ) )
			return;
	}
#endif
	
	if( next_head == uart->buffer.tx_size )
	{
		next_head = 0;
	}

	// Wait until there is space in the buffer
	while ( next_head == uart->buffer.tx_tail ) { 
		timer ++;
		if ( timer >= UART_MAX_WAIT_TIME )
		{
//			printf("\r\n------------------>after 5ms, rx buffer no space <--------------------- \n");
#ifdef _SUPPORT_OS
			xSemaphoreGive( uart->mutex );
#endif
			return;
		}
	}

	// Store data and advance head
	uart->buffer.tx[ uart->buffer.tx_head ] = data;
	uart->buffer.tx_head = next_head;
	
#ifdef _SUPPORT_OS
xSemaphoreGive( uart->mutex );
#endif
}

static uint8_t serial_read( uart_dev *uart )
{
	uint16_t tail = uart->buffer.rx_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
	if ( uart->buffer.rx_head == tail )
	{
		return SERIAL_NO_DATA;
	}
	else
	{
		uint8_t data = uart->buffer.rx[tail];

		tail++;
		if ( tail == uart->buffer.rx_size )
		{
			tail = 0;
		}
		uart->buffer.rx_tail = tail;

		return data;
	}
}
static uint8_t serial_read_tx( uart_dev *uart )
{
	uint16_t tail = uart->buffer.tx_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
	if ( uart->buffer.tx_head == tail )
	{
		return SERIAL_NO_DATA;
	}
	else
	{
		uint8_t data = uart->buffer.tx[tail];

		tail++;
		if ( tail == uart->buffer.tx_size )
		{
			tail = 0;
		}
		uart->buffer.tx_tail = tail;

		return data;
	}
}
static void serial_reset_read_buffer( uart_dev *uart )
{
	uart->buffer.rx_tail = uart->buffer.rx_head;
}

static void serial_reset_send_buffer( uart_dev *uart )
{
    uart->buffer.tx_tail = uart->buffer.tx_head;
}

static uint16_t serial_get_rx_buffer_count( uart_dev *uart )
{
	uint16_t rtail = uart->buffer.rx_tail; // Copy to limit multiple calls to volatile
	
	if ( uart->buffer.rx_head >= rtail )
	{
		return( uart->buffer.rx_head - rtail );
	}
	
	return ( uart->buffer.rx_size - ( rtail- uart->buffer.rx_head ) );
}

static uint16_t serial_get_tx_buffer_count( uart_dev *uart )
{
	uint16_t ttail = uart->buffer.tx_tail; // Copy to limit multiple calls to volatile
	
	if ( uart->buffer.tx_head >= ttail )
	{
		return( uart->buffer.tx_head - ttail );
	}
	
	return ( uart->buffer.tx_size - ( ttail - uart->buffer.tx_head ) );
}

static void clear_tx_rx_buffer(void)
{
	serial_reset_read_buffer( usart1 );
	serial_reset_read_buffer( usart2 );
	serial_reset_read_buffer( uart4 );
	
	serial_reset_send_buffer( usart1 );
	serial_reset_send_buffer( usart2 );
	serial_reset_send_buffer( uart4 );
}

static void delay_ms_uart( __IO uint32_t _T )
{
	uint32_t i;
	while ( _T-- )
		for ( i = 0; i < 10280; i++);
}

/****************************************************************************
                              public funtion
 ****************************************************************************/
void uart_init(void)
{
//	init_usart1();
//	init_usart2();
	init_uart4();
	
	delay_ms_uart(500); // ����ÿ���ϵ����յ�һ��0xFF����ʱ500ms��Ȼ�����0xFF
	clear_tx_rx_buffer();
	
	com_cmd->def.state_rx_int = ENABLE;
	com_cmd->def.init_structure_uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	com_cmd->init();

//	com_touch_screen->def.state_rx_int = ENABLE;
//	com_touch_screen->def.init_structure_uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	com_touch_screen->init();

//	com_ext_wifi->def.state_rx_int = ENABLE;
//	com_ext_wifi->def.init_structure_uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	com_ext_wifi->init();
}

void uart_save_from_isr( uart_dev *uart, uint8_t data )
{
	uint16_t next_head;
	
	// Write character to buffer
	next_head = uart->buffer.rx_head + 1;
	if ( next_head == uart->buffer.rx_size )
	{
		next_head = 0;
	}
	
	// Write data to buffer unless it is full.
	if ( next_head != uart->buffer.rx_tail )
	{
		uart->buffer.rx[ uart->buffer.rx_head ] = data;
		uart->buffer.rx_head = next_head;    
	}
	
}

void uart_debug( uart_dev *uart, uint16_t rx_len )
{
	uint16_t len_rx, len_tx;
	uint16_t idx = 0;
	
	len_rx = uart->get_rx_cnt();
	len_tx = uart->get_tx_cnt();
	printf("\r\n\r\n-----------uart debug--------------\n");
	printf("rx: tail:%d, \thead:%d, \trx_len:%d, \trec data from idx:%d to idx:%d\n", uart->buffer.rx_tail, uart->buffer.rx_head, len_rx, uart->buffer.rx_tail, (uart->buffer.rx_head - 1) );
	printf("tx: tail:%d, \thead:%d, \ttx_len:%d\r\n", uart->buffer.tx_tail, uart->buffer.tx_head, len_tx);
	printf("rx data( %dbyte ):\n", rx_len);
	for( idx = 0; idx < rx_len; idx ++ )
		printf("idx:%d, -> %02X\n", idx, uart->buffer.rx[idx]);
}



/*****************  ����һ���ַ� **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* ����һ���ֽ����ݵ�USART */
	USART_SendData(pUSARTx,ch);
		
	/* �ȴ��������ݼĴ���Ϊ�� */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/*****************  �����ַ��� **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* �ȴ�������� */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  ����һ��16λ�� **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
	
	/* ���͸߰�λ */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* ���͵Ͱ�λ */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}


/////�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
//int fgetc(FILE *f)
//{
//		/* �ȴ������������� */
//		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

//		return (int)USART_ReceiveData(DEBUG_USARTx);
//}



#ifdef __GNUC__

// ʹ��ʱ���ǵ���printf���棬���� fflush(stdout)
int _write(int fd, char *ptr, int len)
{
    int i = 0;

    if (fd > 2)
    {
        return -1;
    }

    while (*ptr && (i < len))
    {
        Usart_SendByte( UART4, *ptr );
		
        i++;
        ptr++;
    }

    return i;
}

#elif defined(__ARMCC__) | !defined(__MICROLIB)

// �����ʹ��΢�⣬����������´���
#pragma import(__use_no_semihosting)

struct __FILE { int handle; };

FILE __stdout;

_sys_exit(int x) { x = x; }

_ttywrch(int ch) { ch = ch; }

//�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData( UART4, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus( UART4, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

#elif defined(__MICROLIB)

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

PUTCHAR_PROTOTYPE
{
    USART_SendData( UART4, (uint8_t) ch); 

    while (USART_GetFlagStatus( UART4, USART_FLAG_TXE) == RESET)
	{
	    ;
	}

	return (ch);
}

#else // no MICROLIB


#endif
