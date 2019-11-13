#ifndef __USART_H
#define	__USART_H


#include "stm32f10x.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"

/** 
  * 串口宏定义，不同的串口挂载的总线和IO不一样，移植时需要修改这几个宏
	* 1-修改总线时钟的宏，uart1挂载到apb2总线，其他uart挂载到apb1总线
	* 2-修改GPIO的宏
  */

// 串口1-USART1
#define  USART_1                    USART1
#define  USART_1_CLK                RCC_APB2Periph_USART1
#define  USART_1_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  USART_1_BAUDRATE           57600

// USART GPIO 引脚宏定义
#define  USART_1_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  USART_1_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
#define  USART_1_TX_GPIO_PORT       GPIOA   
#define  USART_1_TX_GPIO_PIN        GPIO_Pin_9
#define  USART_1_RX_GPIO_PORT       GPIOA
#define  USART_1_RX_GPIO_PIN        GPIO_Pin_10
#define  USART_1_IRQ_PRIO           13
#define  USART_1_IRQ                USART1_IRQn
#define  USART_1_IRQHandler         USART1_IRQHandler

// DMA定义
#define  USART_1_TX_DMA_CHANNEL     DMA1_Channel4
#define  USART_1_DR_ADDRESS         ((uint32_t)&(USART1->DR))
#define  USART_1_IRQ_DMA            DMA1_Channel4_IRQn

// 串口2-USART2
#define  USART_2                    USART2
#define  USART_2_CLK                RCC_APB1Periph_USART2
#define  USART_2_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  USART_2_BAUDRATE           115200

// USART GPIO 引脚宏定义
#define  USART_2_GPIO_CLK           RCC_APB2Periph_GPIOA
#define  USART_2_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
#define  USART_2_TX_GPIO_PORT       GPIOA   
#define  USART_2_TX_GPIO_PIN        GPIO_Pin_2
#define  USART_2_RX_GPIO_PORT       GPIOA
#define  USART_2_RX_GPIO_PIN        GPIO_Pin_3
#define  USART_2_IRQ_PRIO           12
#define  USART_2_IRQ                USART2_IRQn
#define  USART_2_IRQHandler         USART2_IRQHandler

// DMA定义
#define  USART_2_TX_DMA_CHANNEL     DMA1_Channel7
#define  USART_2_DR_ADDRESS         ((uint32_t)&(USART2->DR))
#define  USART_2_IRQ_DMA            DMA1_Channel7_IRQn
#define  USART_2_DMA_IRQHandler     DMA1_Channel7_IRQHandler

#if 0
// 串口3-USART3
#define  USART_3                    USART3
#define  USART_3_CLK                RCC_APB1Periph_USART3
#define  USART_3_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  USART_3_BAUDRATE           115200

// USART GPIO 引脚宏定义
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


// 串口4-UART4
#define  UART_4                    UART4
#define  UART_4_CLK                RCC_APB1Periph_UART4
#define  UART_4_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  UART_4_BAUDRATE           115200

// USART GPIO 引脚宏定义
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
// 串口5-UART5
#define  UART_5                    UART5
#define  UART_5_CLK                RCC_APB1Periph_UART5
#define  UART_5_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  UART_5_BAUDRATE           115200

// USART GPIO 引脚宏定义
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
	SemaphoreHandle_t semaphore;  // 信号量用来同步传输
	SemaphoreHandle_t mutex;  // 互斥信号量用来保护数据传输
#endif
	uart_buffer_def buffer;  // 接收和发送缓冲区
	e_uart_flag_def flag_tx; // 发送状态
	e_uart_flag_def flag_rx; // 接收状态
	// 初始化
	void (*init)(void);    // 重新初始化串口，用于修改参数
	
	// 通过缓冲区接收和发送
	uint8_t (*read)(void);        // 从缓冲区读一个字节
	uint16_t (*read_word)(void);  // 从缓冲区读一个字
	void (*write)(uint8_t data);  // 向缓冲区写入一个字节
	void (*send)(void);           // 把发送缓冲区的数据全部发送出去
	void (*reset_rx)(void);       // 重置接收缓冲区
	uint16_t (*get_tx_cnt)(void);  // 获取发送缓冲区的数据长度，单位是字节
	uint16_t (*get_rx_cnt)(void);  // 获取接收缓冲区的数据长度，单位是字节
	
	// 直接发送
	void (*send_byte)(uint8_t byte);  // 直接把数据byte发送出去
	void (*send_string)(char *str);   // 直接把字符串str发送出去
	void (*send_halfword)(uint16_t word); // 直接把word发送出去
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




