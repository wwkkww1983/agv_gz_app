#ifndef __USART_H
#define	__USART_H


#include "stm32f10x.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"

/** 
  * ���ں궨�壬��ͬ�Ĵ��ڹ��ص����ߺ�IO��һ������ֲʱ��Ҫ�޸��⼸����
	* 1-�޸�����ʱ�ӵĺ꣬uart1���ص�apb2���ߣ�����uart���ص�apb1����
	* 2-�޸�GPIO�ĺ�
  */

// ����1-USART1
#define  USART_1                    USART1
#define  USART_1_CLK                RCC_APB2Periph_USART1
#define  USART_1_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  USART_1_BAUDRATE           57600

// USART GPIO ���ź궨��
#define  USART_1_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  USART_1_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
#define  USART_1_TX_GPIO_PORT       GPIOA   
#define  USART_1_TX_GPIO_PIN        GPIO_Pin_9
#define  USART_1_RX_GPIO_PORT       GPIOA
#define  USART_1_RX_GPIO_PIN        GPIO_Pin_10
#define  USART_1_IRQ_PRIO           13
#define  USART_1_IRQ                USART1_IRQn
#define  USART_1_IRQHandler         USART1_IRQHandler

// DMA����
#define  USART_1_TX_DMA_CHANNEL     DMA1_Channel4
#define  USART_1_DR_ADDRESS         ((uint32_t)&(USART1->DR))
#define  USART_1_IRQ_DMA            DMA1_Channel4_IRQn

// ����2-USART2
#define  USART_2                    USART2
#define  USART_2_CLK                RCC_APB1Periph_USART2
#define  USART_2_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  USART_2_BAUDRATE           115200

// USART GPIO ���ź궨��
#define  USART_2_GPIO_CLK           RCC_APB2Periph_GPIOA
#define  USART_2_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
#define  USART_2_TX_GPIO_PORT       GPIOA   
#define  USART_2_TX_GPIO_PIN        GPIO_Pin_2
#define  USART_2_RX_GPIO_PORT       GPIOA
#define  USART_2_RX_GPIO_PIN        GPIO_Pin_3
#define  USART_2_IRQ_PRIO           12
#define  USART_2_IRQ                USART2_IRQn
#define  USART_2_IRQHandler         USART2_IRQHandler

// DMA����
#define  USART_2_TX_DMA_CHANNEL     DMA1_Channel7
#define  USART_2_DR_ADDRESS         ((uint32_t)&(USART2->DR))
#define  USART_2_IRQ_DMA            DMA1_Channel7_IRQn
#define  USART_2_DMA_IRQHandler     DMA1_Channel7_IRQHandler

#if 0
// ����3-USART3
#define  USART_3                    USART3
#define  USART_3_CLK                RCC_APB1Periph_USART3
#define  USART_3_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  USART_3_BAUDRATE           115200

// USART GPIO ���ź궨��
#define  USART_3_GPIO_CLK           RCC_APB2Periph_GPIOB
#define  USART_3_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
#define  USART_3_TX_GPIO_PORT       GPIOB   
#define  USART_3_TX_GPIO_PIN        GPIO_Pin_10
#define  USART_3_RX_GPIO_PORT       GPIOB
#define  USART_3_RX_GPIO_PIN        GPIO_Pin_11
#define  USART_3_IRQ_PRIO           12
#define  USART_3_IRQ                USART3_IRQn
#define  USART_3_IRQHandler         USART3_IRQHandler
#endif


// ����4-UART4
#define  UART_4                    UART4
#define  UART_4_CLK                RCC_APB1Periph_UART4
#define  UART_4_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  UART_4_BAUDRATE           115200

// USART GPIO ���ź궨��
#define  UART_4_GPIO_CLK           RCC_APB2Periph_GPIOC
#define  UART_4_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
#define  UART_4_TX_GPIO_PORT       GPIOC   
#define  UART_4_TX_GPIO_PIN        GPIO_Pin_10
#define  UART_4_RX_GPIO_PORT       GPIOC
#define  UART_4_RX_GPIO_PIN        GPIO_Pin_11
#define  UART_4_IRQ_PRIO           14
#define  UART_4_IRQ                UART4_IRQn
#define  UART_4_IRQHandler         UART4_IRQHandler


#if 0
// ����5-UART5
#define  UART_5                    UART5
#define  UART_5_CLK                RCC_APB1Periph_UART5
#define  UART_5_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  UART_5_BAUDRATE           115200

// USART GPIO ���ź궨��
#define  UART_5_GPIO_CLK           (RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD)
#define  UART_5_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
#define  UART_5_TX_GPIO_PORT       GPIOC   
#define  UART_5_TX_GPIO_PIN        GPIO_Pin_12
#define  UART_5_RX_GPIO_PORT       GPIOD
#define  UART_5_RX_GPIO_PIN        GPIO_Pin_2
#define  UART_5_IRQ_PRIO           13
#define  UART_5_IRQ                UART5_IRQn
#define  UART_5_IRQHandler         UART5_IRQHandler
#endif

#define SERIAL_NO_DATA 0xff


typedef struct
{
    USART_TypeDef            *port_uart;
    GPIO_TypeDef             *port_tx;
    GPIO_TypeDef             *port_rx;
    GPIO_InitTypeDef         init_structure_tx;
	GPIO_InitTypeDef         init_structure_rx;
    NVIC_InitTypeDef         init_structure_nvic;
    USART_InitTypeDef        init_structure_uart;
	uint32_t                 clk_uart;
	uint32_t                 clk_tx_port;
	uint32_t                 clk_rx_port;
	uint32_t                 af_remap_x;
	uint32_t                 nvic_group;
	FunctionalState          state_rx_int;
	FunctionalState          state_gpio_af;
	void                     (*fun_clk_cmd_uart)(uint32_t periph, FunctionalState state);
	void                     (*fun_clk_cmd_tx)(uint32_t periph, FunctionalState state);
	void                     (*fun_clk_cmd_rx)(uint32_t periph, FunctionalState state);
}uart_type_def;


typedef struct {
	uint8_t *rx;
	uint16_t rx_size;
	uint16_t rx_head;
	volatile uint16_t rx_tail;
	
	uint8_t *tx;
	uint16_t tx_size;
	uint16_t tx_head;
	volatile uint16_t tx_tail;
}uart_buffer_def;

typedef enum _uart_flag {
	e_tx_idle,
	e_rx_idle,
    e_tx_ing,
	e_tx_complete,
	e_rx_ing,
	e_rx_complete
}e_uart_flag_def;

typedef struct
{
	uart_type_def def;
#ifdef _SUPPORT_OS
	SemaphoreHandle_t semaphore;  // �ź�������ͬ������
	SemaphoreHandle_t mutex;  // �����ź��������������ݴ���
#endif
	uart_buffer_def buffer;  // ���պͷ��ͻ�����
	e_uart_flag_def flag_tx; // ����״̬
	e_uart_flag_def flag_rx; // ����״̬
	// ��ʼ��
	void (*init)(void);    // ���³�ʼ�����ڣ������޸Ĳ���
	
	// ͨ�����������պͷ���
	uint8_t (*read)(void);        // �ӻ�������һ���ֽ�
	uint16_t (*read_word)(void);  // �ӻ�������һ����
	void (*write)(uint8_t data);  // �򻺳���д��һ���ֽ�
	void (*send)(void);           // �ѷ��ͻ�����������ȫ�����ͳ�ȥ
	void (*reset_rx)(void);       // ���ý��ջ�����
	uint16_t (*get_tx_cnt)(void);  // ��ȡ���ͻ����������ݳ��ȣ���λ���ֽ�
	uint16_t (*get_rx_cnt)(void);  // ��ȡ���ջ����������ݳ��ȣ���λ���ֽ�
	
	// ֱ�ӷ���
	void (*send_byte)(uint8_t byte);  // ֱ�Ӱ�����byte���ͳ�ȥ
	void (*send_string)(char *str);   // ֱ�Ӱ��ַ���str���ͳ�ȥ
	void (*send_halfword)(uint16_t word); // ֱ�Ӱ�word���ͳ�ȥ
}uart_dev;


extern uart_dev * usart1;
extern uart_dev * usart2;
extern uart_dev * uart4;

#define    com_cmd             uart4
#define    com_touch_screen    usart2
#define    com_ext_wifi        usart1

void uart_init(void);
void uart_save_from_isr( uart_dev *uart, uint8_t data );
void uart_debug( uart_dev *uart, uint16_t rx_len );

void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);



#endif /* __UART_H */




