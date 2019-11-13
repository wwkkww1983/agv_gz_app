#ifndef _BSP_GPIO_H
#define _BSP_GPIO_H

#include "stm32f10x.h"

// AC-OK, PA4-PA8 , low = err
#define               	AC_OK1_GPIO_CLK							RCC_APB2Periph_GPIOA
#define               	AC_OK1_GPIO_PORT    	                GPIOA			   
#define               	AC_OK1_GPIO_PIN		                	GPIO_Pin_4
#define               	AC_OK1_GPIO_Mode		                GPIO_Mode_IN_FLOATING

#define               	AC_OK2_GPIO_CLK                       	RCC_APB2Periph_GPIOA
#define               	AC_OK2_GPIO_PORT    	                GPIOA			   
#define               	AC_OK2_GPIO_PIN		                	GPIO_Pin_5
#define               	AC_OK2_GPIO_Mode		                GPIO_Mode_IN_FLOATING

#define               	AC_OK3_GPIO_CLK                       	RCC_APB2Periph_GPIOA
#define               	AC_OK3_GPIO_PORT    	                GPIOA			   
#define               	AC_OK3_GPIO_PIN		                	GPIO_Pin_6
#define               	AC_OK3_GPIO_Mode		                GPIO_Mode_IN_FLOATING

#define               	AC_OK4_GPIO_CLK                       	RCC_APB2Periph_GPIOA
#define               	AC_OK4_GPIO_PORT    	                GPIOA			   
#define               	AC_OK4_GPIO_PIN		                	GPIO_Pin_7
#define               	AC_OK4_GPIO_Mode		                GPIO_Mode_IN_FLOATING

#define               	AC_OK5_GPIO_CLK                       	RCC_APB2Periph_GPIOA
#define               	AC_OK5_GPIO_PORT    	                GPIOA			   
#define               	AC_OK5_GPIO_PIN		                	GPIO_Pin_8
#define               	AC_OK5_GPIO_Mode		                GPIO_Mode_IN_FLOATING

// DC-OK, PE6-PE10, high = err
#define               	DC_OK1_GPIO_CLK                       	RCC_APB2Periph_GPIOE
#define               	DC_OK1_GPIO_PORT    	                GPIOE			   
#define               	DC_OK1_GPIO_PIN		                	GPIO_Pin_0
#define               	DC_OK1_GPIO_Mode		                GPIO_Mode_IN_FLOATING

#define               	DC_OK2_GPIO_CLK                       	RCC_APB2Periph_GPIOE
#define               	DC_OK2_GPIO_PORT    	                GPIOE			   
#define               	DC_OK2_GPIO_PIN		                	GPIO_Pin_1
#define               	DC_OK2_GPIO_Mode		                GPIO_Mode_IN_FLOATING

#define               	DC_OK3_GPIO_CLK                       	RCC_APB2Periph_GPIOE
#define               	DC_OK3_GPIO_PORT    	                GPIOE			   
#define               	DC_OK3_GPIO_PIN		                	GPIO_Pin_2
#define               	DC_OK3_GPIO_Mode		                GPIO_Mode_IN_FLOATING

#define               	DC_OK4_GPIO_CLK                       	RCC_APB2Periph_GPIOE
#define               	DC_OK4_GPIO_PORT    	                GPIOE			   
#define               	DC_OK4_GPIO_PIN		                	GPIO_Pin_3
#define               	DC_OK4_GPIO_Mode		                GPIO_Mode_IN_FLOATING

#define               	DC_OK5_GPIO_CLK                       	RCC_APB2Periph_GPIOE
#define               	DC_OK5_GPIO_PORT    	                GPIOE			   
#define               	DC_OK5_GPIO_PIN		                	GPIO_Pin_4
#define               	DC_OK5_GPIO_Mode		                GPIO_Mode_IN_FLOATING

#define               	DC_OK6_GPIO_CLK                       	RCC_APB2Periph_GPIOE
#define               	DC_OK6_GPIO_PORT    	                GPIOE			   
#define               	DC_OK6_GPIO_PIN		                	GPIO_Pin_5
#define               	DC_OK6_GPIO_Mode		                GPIO_Mode_IN_FLOATING

#define               	DC_OK7_GPIO_CLK                       	RCC_APB2Periph_GPIOE
#define               	DC_OK7_GPIO_PORT    	                GPIOE			   
#define               	DC_OK7_GPIO_PIN		                	GPIO_Pin_6
#define               	DC_OK7_GPIO_Mode		                GPIO_Mode_IN_FLOATING

#define               	DC_OK8_GPIO_CLK                       	RCC_APB2Periph_GPIOE
#define               	DC_OK8_GPIO_PORT    	                GPIOE			   
#define               	DC_OK8_GPIO_PIN		                	GPIO_Pin_7
#define               	DC_OK8_GPIO_Mode		                GPIO_Mode_IN_FLOATING


// T-ALARM, PE11-PE15, high = err
#define               	T_ALARM1_GPIO_CLK                     	RCC_APB2Periph_GPIOE
#define               	T_ALARM1_GPIO_PORT    	            	GPIOE			   
#define               	T_ALARM1_GPIO_PIN		                GPIO_Pin_8
#define               	T_ALARM1_GPIO_Mode		            	GPIO_Mode_IN_FLOATING

#define               	T_ALARM2_GPIO_CLK                     	RCC_APB2Periph_GPIOE
#define               	T_ALARM2_GPIO_PORT    	            	GPIOE			   
#define               	T_ALARM2_GPIO_PIN		                GPIO_Pin_9
#define               	T_ALARM2_GPIO_Mode		            	GPIO_Mode_IN_FLOATING

#define               	T_ALARM3_GPIO_CLK                    	RCC_APB2Periph_GPIOE
#define               	T_ALARM3_GPIO_PORT    	            	GPIOE			   
#define               	T_ALARM3_GPIO_PIN		                GPIO_Pin_10
#define               	T_ALARM3_GPIO_Mode		            	GPIO_Mode_IN_FLOATING

#define               	T_ALARM4_GPIO_CLK                     	RCC_APB2Periph_GPIOE
#define               	T_ALARM4_GPIO_PORT    	            	GPIOE			   
#define               	T_ALARM4_GPIO_PIN		                GPIO_Pin_11
#define               	T_ALARM4_GPIO_Mode		            	GPIO_Mode_IN_FLOATING

#define               	T_ALARM5_GPIO_CLK                     	RCC_APB2Periph_GPIOE
#define              	T_ALARM5_GPIO_PORT    	            	GPIOE			   
#define               	T_ALARM5_GPIO_PIN		                GPIO_Pin_12
#define              	T_ALARM5_GPIO_Mode		            	GPIO_Mode_IN_FLOATING

#define               	T_ALARM6_GPIO_CLK                     	RCC_APB2Periph_GPIOE
#define              	T_ALARM6_GPIO_PORT    	            	GPIOE			   
#define               	T_ALARM6_GPIO_PIN		                GPIO_Pin_13
#define              	T_ALARM6_GPIO_Mode		            	GPIO_Mode_IN_FLOATING

#define               	T_ALARM7_GPIO_CLK                     	RCC_APB2Periph_GPIOE
#define              	T_ALARM7_GPIO_PORT    	            	GPIOE			   
#define               	T_ALARM7_GPIO_PIN		                GPIO_Pin_14
#define              	T_ALARM7_GPIO_Mode		            	GPIO_Mode_IN_FLOATING

#define               	T_ALARM8_GPIO_CLK                     	RCC_APB2Periph_GPIOE
#define              	T_ALARM8_GPIO_PORT    	            	GPIOE			   
#define               	T_ALARM8_GPIO_PIN		                GPIO_Pin_15
#define              	T_ALARM8_GPIO_Mode		            	GPIO_Mode_IN_FLOATING

// Relay AC input, high->close, low->open
#define               	RELAY_AC_INPUT_1_GPIO_CLK				RCC_APB2Periph_GPIOF
#define               	RELAY_AC_INPUT_1_GPIO_PORT    	        GPIOF			   
#define               	RELAY_AC_INPUT_1_GPIO_PIN		        GPIO_Pin_0
#define               	RELAY_AC_INPUT_1_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_AC_INPUT_2_GPIO_CLK				RCC_APB2Periph_GPIOF
#define               	RELAY_AC_INPUT_2_GPIO_PORT    	        GPIOF			   
#define               	RELAY_AC_INPUT_2_GPIO_PIN		        GPIO_Pin_1
#define               	RELAY_AC_INPUT_2_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_AC_INPUT_3_GPIO_CLK				RCC_APB2Periph_GPIOF
#define               	RELAY_AC_INPUT_3_GPIO_PORT    	        GPIOF			   
#define               	RELAY_AC_INPUT_3_GPIO_PIN		        GPIO_Pin_2
#define               	RELAY_AC_INPUT_3_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_AC_INPUT_4_GPIO_CLK				RCC_APB2Periph_GPIOF
#define               	RELAY_AC_INPUT_4_GPIO_PORT    	        GPIOF			   
#define               	RELAY_AC_INPUT_4_GPIO_PIN		        GPIO_Pin_3
#define               	RELAY_AC_INPUT_4_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_AC_INPUT_5_GPIO_CLK				RCC_APB2Periph_GPIOF
#define               	RELAY_AC_INPUT_5_GPIO_PORT    	        GPIOF			   
#define               	RELAY_AC_INPUT_5_GPIO_PIN		        GPIO_Pin_4
#define               	RELAY_AC_INPUT_5_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_AC_INPUT_6_GPIO_CLK				RCC_APB2Periph_GPIOF
#define               	RELAY_AC_INPUT_6_GPIO_PORT    	        GPIOF			   
#define               	RELAY_AC_INPUT_6_GPIO_PIN		        GPIO_Pin_5
#define               	RELAY_AC_INPUT_6_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_AC_INPUT_7_GPIO_CLK				RCC_APB2Periph_GPIOF
#define               	RELAY_AC_INPUT_7_GPIO_PORT    	        GPIOF			   
#define               	RELAY_AC_INPUT_7_GPIO_PIN		        GPIO_Pin_6
#define               	RELAY_AC_INPUT_7_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_AC_INPUT_8_GPIO_CLK				RCC_APB2Periph_GPIOF
#define               	RELAY_AC_INPUT_8_GPIO_PORT    	        GPIOF			   
#define               	RELAY_AC_INPUT_8_GPIO_PIN		        GPIO_Pin_7
#define               	RELAY_AC_INPUT_8_GPIO_Mode		        GPIO_Mode_Out_PP

// Relay DC output, high->close, low->open
// Relay DC output, low->close, high->open - 20190228
#define               	RELAY_DC_OUTPUT_1_GPIO_CLK				RCC_APB2Periph_GPIOG
#define               	RELAY_DC_OUTPUT_1_GPIO_PORT    	        GPIOG			   
#define               	RELAY_DC_OUTPUT_1_GPIO_PIN		        GPIO_Pin_0
#define               	RELAY_DC_OUTPUT_1_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_DC_OUTPUT_2_GPIO_CLK				RCC_APB2Periph_GPIOG
#define               	RELAY_DC_OUTPUT_2_GPIO_PORT    	        GPIOG			   
#define               	RELAY_DC_OUTPUT_2_GPIO_PIN		        GPIO_Pin_1
#define               	RELAY_DC_OUTPUT_2_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_DC_OUTPUT_3_GPIO_CLK				RCC_APB2Periph_GPIOG
#define               	RELAY_DC_OUTPUT_3_GPIO_PORT    	        GPIOG			   
#define               	RELAY_DC_OUTPUT_3_GPIO_PIN		        GPIO_Pin_2
#define               	RELAY_DC_OUTPUT_3_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_DC_OUTPUT_4_GPIO_CLK				RCC_APB2Periph_GPIOG
#define               	RELAY_DC_OUTPUT_4_GPIO_PORT    	        GPIOG			   
#define               	RELAY_DC_OUTPUT_4_GPIO_PIN		        GPIO_Pin_3
#define               	RELAY_DC_OUTPUT_4_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_DC_OUTPUT_5_GPIO_CLK				RCC_APB2Periph_GPIOG
#define               	RELAY_DC_OUTPUT_5_GPIO_PORT    	        GPIOG			   
#define               	RELAY_DC_OUTPUT_5_GPIO_PIN		        GPIO_Pin_4
#define               	RELAY_DC_OUTPUT_5_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_DC_OUTPUT_6_GPIO_CLK				RCC_APB2Periph_GPIOG
#define               	RELAY_DC_OUTPUT_6_GPIO_PORT    	        GPIOG			   
#define               	RELAY_DC_OUTPUT_6_GPIO_PIN		        GPIO_Pin_5
#define               	RELAY_DC_OUTPUT_6_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_DC_OUTPUT_7_GPIO_CLK				RCC_APB2Periph_GPIOG
#define               	RELAY_DC_OUTPUT_7_GPIO_PORT    	        GPIOG			   
#define               	RELAY_DC_OUTPUT_7_GPIO_PIN		        GPIO_Pin_6
#define               	RELAY_DC_OUTPUT_7_GPIO_Mode		        GPIO_Mode_Out_PP

#define               	RELAY_DC_OUTPUT_8_GPIO_CLK				RCC_APB2Periph_GPIOG
#define               	RELAY_DC_OUTPUT_8_GPIO_PORT    	        GPIOG			   
#define               	RELAY_DC_OUTPUT_8_GPIO_PIN		        GPIO_Pin_7
#define               	RELAY_DC_OUTPUT_8_GPIO_Mode		        GPIO_Mode_Out_PP

// Remote ON/OFF control pin
#define               	ON_OFF_1_GPIO_CLK						RCC_APB2Periph_GPIOF
#define               	ON_OFF_1_GPIO_PORT    	                GPIOF			   
#define               	ON_OFF_1_GPIO_PIN		                GPIO_Pin_8
#define               	ON_OFF_1_GPIO_Mode		                GPIO_Mode_Out_PP

#define               	ON_OFF_2_GPIO_CLK						RCC_APB2Periph_GPIOF
#define               	ON_OFF_2_GPIO_PORT    	                GPIOF			   
#define               	ON_OFF_2_GPIO_PIN		                GPIO_Pin_9
#define               	ON_OFF_2_GPIO_Mode		                GPIO_Mode_Out_PP

#define               	ON_OFF_3_GPIO_CLK						RCC_APB2Periph_GPIOF
#define               	ON_OFF_3_GPIO_PORT    	                GPIOF			   
#define               	ON_OFF_3_GPIO_PIN		                GPIO_Pin_10
#define               	ON_OFF_3_GPIO_Mode		                GPIO_Mode_Out_PP

#define               	ON_OFF_4_GPIO_CLK						RCC_APB2Periph_GPIOF
#define               	ON_OFF_4_GPIO_PORT    	                GPIOF			   
#define               	ON_OFF_4_GPIO_PIN		                GPIO_Pin_11
#define               	ON_OFF_4_GPIO_Mode		                GPIO_Mode_Out_PP

#define               	ON_OFF_5_GPIO_CLK						RCC_APB2Periph_GPIOF
#define               	ON_OFF_5_GPIO_PORT    	                GPIOF			   
#define               	ON_OFF_5_GPIO_PIN		                GPIO_Pin_12
#define               	ON_OFF_5_GPIO_Mode		                GPIO_Mode_Out_PP

#define               	ON_OFF_6_GPIO_CLK						RCC_APB2Periph_GPIOF
#define               	ON_OFF_6_GPIO_PORT    	                GPIOF			   
#define               	ON_OFF_6_GPIO_PIN		                GPIO_Pin_13
#define               	ON_OFF_6_GPIO_Mode		                GPIO_Mode_Out_PP

#define               	ON_OFF_7_GPIO_CLK						RCC_APB2Periph_GPIOF
#define               	ON_OFF_7_GPIO_PORT    	                GPIOF			   
#define               	ON_OFF_7_GPIO_PIN		                GPIO_Pin_14
#define               	ON_OFF_7_GPIO_Mode		                GPIO_Mode_Out_PP

#define               	ON_OFF_8_GPIO_CLK						RCC_APB2Periph_GPIOF
#define               	ON_OFF_8_GPIO_PORT    	                GPIOF			   
#define               	ON_OFF_8_GPIO_PIN		                GPIO_Pin_15
#define               	ON_OFF_8_GPIO_Mode		                GPIO_Mode_Out_PP

// ChgrMod select, high->curve, low->pmbus control
#define               	RELAY_CHMOD_D1_GPIO_CLK			    	RCC_APB2Periph_GPIOG
#define               	RELAY_CHMOD_D1_GPIO_PORT    	        GPIOG			    
#define               	RELAY_CHMOD_D1_GPIO_PIN		            GPIO_Pin_8
#define               	RELAY_CHMOD_D1_GPIO_Mode		        GPIO_Mode_Out_PP
                                                                                 
#define               	RELAY_CHMOD_D2_GPIO_CLK			    	RCC_APB2Periph_GPIOG
#define               	RELAY_CHMOD_D2_GPIO_PORT    	        GPIOG			    
#define               	RELAY_CHMOD_D2_GPIO_PIN		            GPIO_Pin_9
#define               	RELAY_CHMOD_D2_GPIO_Mode		        GPIO_Mode_Out_PP
                                                                                 
#define               	RELAY_CHMOD_D3_GPIO_CLK			    	RCC_APB2Periph_GPIOG
#define               	RELAY_CHMOD_D3_GPIO_PORT    	        GPIOG			    
#define               	RELAY_CHMOD_D3_GPIO_PIN		            GPIO_Pin_10
#define               	RELAY_CHMOD_D3_GPIO_Mode		        GPIO_Mode_Out_PP
                                                                                 
#define               	RELAY_CHMOD_D4_GPIO_CLK			    	RCC_APB2Periph_GPIOG
#define               	RELAY_CHMOD_D4_GPIO_PORT    	        GPIOG			    
#define               	RELAY_CHMOD_D4_GPIO_PIN		            GPIO_Pin_11
#define               	RELAY_CHMOD_D4_GPIO_Mode		        GPIO_Mode_Out_PP
                                                                                 
#define               	RELAY_CHMOD_D5_GPIO_CLK			    	RCC_APB2Periph_GPIOG
#define               	RELAY_CHMOD_D5_GPIO_PORT    	        GPIOG			    
#define               	RELAY_CHMOD_D5_GPIO_PIN		            GPIO_Pin_12
#define               	RELAY_CHMOD_D5_GPIO_Mode		        GPIO_Mode_Out_PP
                                                                                 
#define               	RELAY_CHMOD_D6_GPIO_CLK			    	RCC_APB2Periph_GPIOG
#define               	RELAY_CHMOD_D6_GPIO_PORT    	        GPIOG			    
#define               	RELAY_CHMOD_D6_GPIO_PIN		            GPIO_Pin_13
#define               	RELAY_CHMOD_D6_GPIO_Mode		        GPIO_Mode_Out_PP
                                                                                 
#define               	RELAY_CHMOD_D7_GPIO_CLK			    	RCC_APB2Periph_GPIOG
#define               	RELAY_CHMOD_D7_GPIO_PORT    	        GPIOG			    
#define               	RELAY_CHMOD_D7_GPIO_PIN		            GPIO_Pin_14
#define               	RELAY_CHMOD_D7_GPIO_Mode		        GPIO_Mode_Out_PP
                                                                                 
#define               	RELAY_CHMOD_D8_GPIO_CLK			    	RCC_APB2Periph_GPIOG
#define               	RELAY_CHMOD_D8_GPIO_PORT    	        GPIOG			    
#define               	RELAY_CHMOD_D8_GPIO_PIN		            GPIO_Pin_15
#define               	RELAY_CHMOD_D8_GPIO_Mode		        GPIO_Mode_Out_PP

// Check Bat Conn, 0 = bat connected, 1 = bat disconnected
#define               	BAT_CONN_GPIO_CLK						RCC_APB2Periph_GPIOA
#define               	BAT_CONN_GPIO_PORT    	                GPIOA			   
#define               	BAT_CONN_GPIO_PIN		                GPIO_Pin_1
#define               	BAT_CONN_GPIO_Mode		                GPIO_Mode_IN_FLOATING

// Relay
#define               	RELAY_GPIO_CLK							RCC_APB2Periph_GPIOB
#define               	RELAY_GPIO_PORT    	                	GPIOB			   
#define               	RELAY_GPIO_PIN		                	GPIO_Pin_8
#define               	RELAY_GPIO_Mode		               		GPIO_Mode_Out_PP



// emergency stop, press = stop = 0, up = start = 1
#define               	STOP_GPIO_CLK							(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO)
#define               	STOP_GPIO_PORT    	                	GPIOD			   
#define               	STOP_GPIO_PIN		                	GPIO_Pin_12
#define               	STOP_GPIO_Mode		                	GPIO_Mode_IN_FLOATING

#define                 STOP_EXTI_PORTSOURCE                    GPIO_PortSourceGPIOD
#define                 STOP_EXTI_PINSOURCE                     GPIO_PinSource12
#define                 STOP_EXTI_LINE                          EXTI_Line12
#define                 STOP_EXTI_IRQ                           EXTI15_10_IRQn


// sys indicate led
#define               	LED_GREEN_GPIO_CLK							RCC_APB2Periph_GPIOC
#define               	LED_GREEN_GPIO_PORT    	                	GPIOC			   
#define               	LED_GREEN_GPIO_PIN		                	GPIO_Pin_6
#define               	LED_GREEN_GPIO_Mode		                	GPIO_Mode_Out_PP

#define               	LED_RED_GPIO_CLK							RCC_APB2Periph_GPIOC
#define               	LED_RED_GPIO_PORT    	                	GPIOC
#define               	LED_RED_GPIO_PIN		                	GPIO_Pin_7
#define               	LED_RED_GPIO_Mode		                	GPIO_Mode_Out_PP

#define               	LED_WIFI_CONN_GPIO_CLK						RCC_APB2Periph_GPIOC
#define               	LED_WIFI_CONN_GPIO_PORT    	               	GPIOC
#define               	LED_WIFI_CONN_GPIO_PIN		               	GPIO_Pin_8
#define               	LED_WIFI_CONN_GPIO_Mode		               	GPIO_Mode_Out_PP


// 485 comm ext , spi-1
#define                 SPI_EXT_1_NSS_GPIO_CLK                      (RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO)
#define                 SPI_EXT_1_NSS_GPIO_PORT                     GPIOA
#define                 SPI_EXT_1_NSS_GPIO_PIN                      GPIO_Pin_4
#define                 SPI_EXT_1_NSS_GPIO_MODE                     GPIO_Mode_Out_PP

#define                 SPI_EXT_1_SCK_GPIO_CLK                      (RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO)
#define                 SPI_EXT_1_SCK_GPIO_PORT                     GPIOA
#define                 SPI_EXT_1_SCK_GPIO_PIN                      GPIO_Pin_5
#define                 SPI_EXT_1_SCK_GPIO_MODE                     GPIO_Mode_AF_PP 

#define                 SPI_EXT_1_MISO_GPIO_CLK                     RCC_APB2Periph_GPIOA
#define                 SPI_EXT_1_MISO_GPIO_PORT                    GPIOA
#define                 SPI_EXT_1_MISO_GPIO_PIN                     GPIO_Pin_6
#define                 SPI_EXT_1_MISO_GPIO_MODE                    GPIO_Mode_AF_PP

#define                 SPI_EXT_1_MOSI_GPIO_CLK                     RCC_APB2Periph_GPIOA
#define                 SPI_EXT_1_MOSI_GPIO_PORT                    GPIOA
#define                 SPI_EXT_1_MOSI_GPIO_PIN                     GPIO_Pin_7
#define                 SPI_EXT_1_MOSI_GPIO_MODE                    GPIO_Mode_AF_PP

#define                 SPI_EXT_1_IRQ_GPIO_CLK                      RCC_APB2Periph_GPIOA
#define                 SPI_EXT_1_IRQ_GPIO_PORT                     GPIOA
#define                 SPI_EXT_1_IRQ_GPIO_PIN                      GPIO_Pin_8
#define                 SPI_EXT_1_IRQ_GPIO_MODE                     GPIO_Mode_IN_FLOATING

#define                 SPI_EXT_1_RST_GPIO_CLK                      RCC_APB2Periph_GPIOA
#define                 SPI_EXT_1_RST_GPIO_PORT                     GPIOA
#define                 SPI_EXT_1_RST_GPIO_PIN                      GPIO_Pin_1
#define                 SPI_EXT_1_RST_GPIO_MODE                     GPIO_Mode_Out_PP

// 485 comm ext , spi-2
#define                 SPI_EXT_2_NSS_GPIO_CLK                      (RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO)
#define                 SPI_EXT_2_NSS_GPIO_PORT                     GPIOB
#define                 SPI_EXT_2_NSS_GPIO_PIN                      GPIO_Pin_12
#define                 SPI_EXT_2_NSS_GPIO_MODE                     GPIO_Mode_Out_PP

#define                 SPI_EXT_2_SCK_GPIO_CLK                      RCC_APB2Periph_GPIOB
#define                 SPI_EXT_2_SCK_GPIO_PORT                     GPIOB
#define                 SPI_EXT_2_SCK_GPIO_PIN                      GPIO_Pin_13
#define                 SPI_EXT_2_SCK_GPIO_MODE                     GPIO_Mode_AF_PP

#define                 SPI_EXT_2_MISO_GPIO_CLK                     RCC_APB2Periph_GPIOB
#define                 SPI_EXT_2_MISO_GPIO_PORT                    GPIOB
#define                 SPI_EXT_2_MISO_GPIO_PIN                     GPIO_Pin_14
#define                 SPI_EXT_2_MISO_GPIO_MODE                    GPIO_Mode_AF_PP

#define                 SPI_EXT_2_MOSI_GPIO_CLK                     RCC_APB2Periph_GPIOB
#define                 SPI_EXT_2_MOSI_GPIO_PORT                    GPIOB
#define                 SPI_EXT_2_MOSI_GPIO_PIN                     GPIO_Pin_15
#define                 SPI_EXT_2_MOSI_GPIO_MODE                    GPIO_Mode_AF_PP

#define                 SPI_EXT_2_IRQ_GPIO_CLK                      RCC_APB2Periph_GPIOD
#define                 SPI_EXT_2_IRQ_GPIO_PORT                     GPIOD
#define                 SPI_EXT_2_IRQ_GPIO_PIN                      GPIO_Pin_10
#define                 SPI_EXT_2_IRQ_GPIO_MODE                     GPIO_Mode_IN_FLOATING

#define                 SPI_EXT_2_RST_GPIO_CLK                      RCC_APB2Periph_GPIOD
#define                 SPI_EXT_2_RST_GPIO_PORT                     GPIOD
#define                 SPI_EXT_2_RST_GPIO_PIN                      GPIO_Pin_7
#define                 SPI_EXT_2_RST_GPIO_MODE                     GPIO_Mode_Out_PP

// ext wifi
#define                 EXT_WIFI_RST_GPIO_CLK                       RCC_APB2Periph_GPIOD
#define                 EXT_WIFI_RST_GPIO_PORT                      GPIOD
#define                 EXT_WIFI_RST_GPIO_PIN                       GPIO_Pin_3
#define                 EXT_WIFI_RST_GPIO_MODE                      GPIO_Mode_Out_PP

#define                 EXT_WIFI_RELOAD_GPIO_CLK                    RCC_APB2Periph_GPIOD
#define                 EXT_WIFI_RELOAD_GPIO_PORT                   GPIOD
#define                 EXT_WIFI_RELOAD_GPIO_PIN                    GPIO_Pin_4
#define                 EXT_WIFI_RELOAD_GPIO_MODE                   GPIO_Mode_Out_PP

#define                 EXT_WIFI_RDY_GPIO_CLK                       RCC_APB2Periph_GPIOD
#define                 EXT_WIFI_RDY_GPIO_PORT                      GPIOD
#define                 EXT_WIFI_RDY_GPIO_PIN                       GPIO_Pin_5
#define                 EXT_WIFI_RDY_GPIO_MODE                      GPIO_Mode_IN_FLOATING

#define                 EXT_WIFI_LINK_GPIO_CLK                      RCC_APB2Periph_GPIOD
#define                 EXT_WIFI_LINK_GPIO_PORT                     GPIOD
#define                 EXT_WIFI_LINK_GPIO_PIN                      GPIO_Pin_6
#define                 EXT_WIFI_LINK_GPIO_MODE                     GPIO_Mode_IN_FLOATING

// fan
#define               	FAN_1_SPEED_CTL_GPIO_CLK					RCC_APB2Periph_GPIOB
#define               	FAN_1_SPEED_CTL_GPIO_PORT    	            GPIOB
#define               	FAN_1_SPEED_CTL_GPIO_PIN		            GPIO_Pin_9
#define               	FAN_1_SPEED_CTL_GPIO_Mode		            GPIO_Mode_Out_PP

#define               	FAN_1_ERR_CHK_GPIO_CLK					    RCC_APB2Periph_GPIOD
#define               	FAN_1_ERR_CHK_GPIO_PORT    	                GPIOD
#define               	FAN_1_ERR_CHK_GPIO_PIN		                GPIO_Pin_11
#define               	FAN_1_ERR_CHK_GPIO_Mode		                GPIO_Mode_IN_FLOATING

#define               	FAN_2_SPEED_CTL_GPIO_CLK					RCC_APB2Periph_GPIOB
#define               	FAN_2_SPEED_CTL_GPIO_PORT    	            GPIOB
#define               	FAN_2_SPEED_CTL_GPIO_PIN		            GPIO_Pin_5
#define               	FAN_2_SPEED_CTL_GPIO_Mode		            GPIO_Mode_Out_PP

#define               	FAN_2_ERR_CHK_GPIO_CLK					    RCC_APB2Periph_GPIOD
#define               	FAN_2_ERR_CHK_GPIO_PORT    	                GPIOD
#define               	FAN_2_ERR_CHK_GPIO_PIN		                GPIO_Pin_9
#define               	FAN_2_ERR_CHK_GPIO_Mode		                GPIO_Mode_IN_FLOATING



#ifndef HIGH
#define					HIGH									1
#endif

#ifndef LOW
#define					LOW										0
#endif



#define 				GPIO_READ(port, pin)  					((GPIO_ReadInputDataBit(port, pin) != Bit_RESET)?HIGH:LOW)
#define					GPIO_SET(port, pin)						(GPIO_SetBits(port, pin))
#define					GPIO_RESET(port, pin)					(GPIO_ResetBits(port, pin))
#define					GPIO_TOGGLE(port, pin) 					( GPIO_ReadOutputDataBit ( port, pin ) ? \
																GPIO_ResetBits ( port, pin ) : \
																GPIO_SetBits ( port, pin ))


#define    LED_RED_ON()    (GPIO_SetBits(LED_RED_GPIO_PORT, LED_RED_GPIO_PIN))
#define    LED_RED_OFF()   (GPIO_ResetBits(LED_RED_GPIO_PORT, LED_RED_GPIO_PIN))

#define    LED_GREEN_ON()  (GPIO_SetBits(LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PIN))
#define    LED_GREEN_OFF() (GPIO_ResetBits(LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PIN))

#define    LED_RED_TOGGLE() (GPIO_TOGGLE(LED_RED_GPIO_PORT, LED_RED_GPIO_PIN))
#define    LED_GREEN_TOGGLE() (GPIO_TOGGLE(LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PIN))


void sInitGpio(void);

void gpio_close_ac_input( uint8_t num );
void gpio_open_ac_input( uint8_t num );

void gpio_open_dc_relay( uint8_t num );
void gpio_close_dc_relay( uint8_t num );

void gpio_light_on_led_wifi_conn(void);
void gpio_light_off_led_wifi_conn(void);

void gpio_set( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin );
void gpio_reset( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin );
void gpio_toggle( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin );

#endif // bsp_gpio.h
