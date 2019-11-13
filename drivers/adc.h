#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"

// ADC输入通道配置
#define    ADC_APBxClock_FUN             RCC_APB2PeriphClockCmd
#define    ADC_CLK                       RCC_APB2Periph_ADC1

#define    ADC_GPIO_APBxClock_FUN        RCC_APB2PeriphClockCmd

#define    ADC_GPIO_1_CLK                RCC_APB2Periph_GPIOC  
#define    ADC_PORT_1                    GPIOC

#define    ADC_GPIO_3_CLK                RCC_APB2Periph_GPIOB
#define    ADC_PORT_3                    GPIOB

// 每次转换通道个数
//#define    NOFCHANEL					 1
#define    NOFCHANEL					 8

// 转换通道配置
    // PB0 - ch8
#define    ADC_PIN1                      GPIO_Pin_0
#define    ADC_CHANNEL1                  ADC_Channel_8
    // PB1 - ch9
#define    ADC_PIN2                      GPIO_Pin_1
#define    ADC_CHANNEL2                  ADC_Channel_9
    // PC0 - ch10 ...
#define    ADC_PIN3                      GPIO_Pin_0
#define    ADC_CHANNEL3                  ADC_Channel_10
    // PC1 - ch11 ...
#define    ADC_PIN4                      GPIO_Pin_1
#define    ADC_CHANNEL4                  ADC_Channel_11
    // PC2 - ch12 ...
#define    ADC_PIN5                      GPIO_Pin_2
#define    ADC_CHANNEL5                  ADC_Channel_12
    // PC3 - ch13 ...
#define    ADC_PIN6                      GPIO_Pin_3
#define    ADC_CHANNEL6                  ADC_Channel_13
    // PC4 - ch14 ...
#define    ADC_PIN7                      GPIO_Pin_4
#define    ADC_CHANNEL7                  ADC_Channel_14
    // PC5 - ch15 ...
#define    ADC_PIN8                      GPIO_Pin_5
#define    ADC_CHANNEL8                  ADC_Channel_15


// ADC1 对应 DMA1通道1，ADC3对应DMA2通道5，ADC2没有DMA功能
#define    ADC_x                         ADC1
#define    ADC_DMA_CHANNEL               DMA1_Channel1
#define    ADC_DMA_CLK                   RCC_AHBPeriph_DMA1


void adc_init( void );
// num取值，0-7，分别对应模块序号
float adc_get_smp_value( uint8_t num );
uint16_t adc_get_convert_value( uint8_t num );
uint16_t adc_get_convert_value_single( uint8_t num );

#endif // adc.h

