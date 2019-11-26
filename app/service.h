#ifndef __SERVICE_H
#define __SERVICE_H

#include "stm32f10x.h"
#include "uart.h"

#include "timers.h"


#ifdef __DEBUG
/* Define the queue parameters. */
#define    PRINT_QUEUE_LENGTH    5
#define    PRINT_QUEUE_ITEM_SIZE 200
	
/* Define the data type that will be queued. */
typedef uint8_t PrintMSG[PRINT_QUEUE_ITEM_SIZE] ;
#endif

void kprintf(const char *fmt, ...);
void print_tsk_status(void);


#ifndef __LOG

#define    __LOG_INFO_APP    0
#define    __LOG_ERROR_APP   0
#define    __LOG_WARNING_APP 0
#define    __LOG_DEBUG_APP   0
#define    __LOG_TRACE       0

#endif


#define LOG_INFO_APP(fmt,arg...)      do{\
                                          if(__LOG_INFO_APP)\
											   kprintf("\r\n<<-INFO_APP->> "fmt" ",##arg);\
                                          }while(0)

#define LOG_ERROR_APP(fmt,arg...)      do{\
                                          if(__LOG_ERROR_APP)\
											  kprintf("\r\n<<-ERROR_APP->> "fmt"\n",##arg);\
                                          }while(0)

#define LOG_WARNING_APP(fmt,arg...)      do{\
                                          if(__LOG_ERROR_APP)\
											  kprintf("\r\n<<-WARNING_APP->> "fmt" ",##arg);\
                                          }while(0)

#define LOG_WARNING_APP_1(fmt,arg...)      do{\
                                          if(__LOG_DEBUG_APP)\
											  kprintf(fmt, ##arg);\
                                          }while(0)

// 直接通过串口发送，不通过调试任务，无系统api调用
#define LOG_DEBUG_FILE_APP(fmt,arg...)      do{\
                                          if(__LOG_DEBUG_APP) { printf("\r\n<<-DEBUG_FILE_APP->> [file:%s, line:%d] "fmt" \n", __FILE__, __LINE__, ##arg); fflush(stdout); } \
                                          }while(0)

#define LOG_DEBUG_APP(fmt,arg...)      do{\
                                          if(__LOG_DEBUG_APP)\
											  kprintf("\r\n<<-DEBUG_APP->> "fmt" \n",##arg);\
                                          }while(0)

#define LOG_DEBUG_APP_1(fmt,arg...)      do{\
                                          if(__LOG_DEBUG_APP)\
											  kprintf(fmt, ##arg);\
                                          }while(0)

// 无系统api										  
#define LOG_TRACE(fmt,arg...)          do{\
                                          if(__LOG_TRACE) { printf("\r\n\r\n<<trace->> " fmt, ##arg); fflush(stdout); } \
                                          }while(0)

#define LOG_TRACE_1(fmt,arg...)          do{\
                                          if(__LOG_TRACE) { printf(fmt, ##arg); fflush(stdout); } \
                                          }while(0)

#define  NUM_TEST_TIMERS           3
#define  TEST_TIMER_ID_CAN         0
#define  TEST_TIMER_ID_485         1
#define  TEST_TIMER_ID_ETHERNET    2

										  

void delay_ms( __IO uint32_t _T );
void delay_us( __IO uint32_t _T );
void kprintf(const char *fmt, ...);
void myprintf(const char *fmt, ...);

extern TaskHandle_t h_printf_entry;
extern TaskHandle_t h_cmd_entry;
void service_init(void);
										  
extern TimerHandle_t xTimersBox_Test[];

#endif // service.h

