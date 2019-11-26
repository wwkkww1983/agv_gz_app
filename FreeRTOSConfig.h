/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
 *=================================== 中文注解 =======================================
 * FreeRTOSConfig.h这个头文件用来配置FreeRTOS的功能，但是并不全面，剩下的其他功能可以
 * 在FreeRTOS.h这个头文件配置，比如使能互斥信号量，使能递归信号量，使能事件标志组这些
 * 内核对象的功能时，是在FreeRTOS.h这个头文件里面配置的。即要想完全使用FreeRTOS的功能
 * 需要这两个头文件一起联合使用。
 *====================================================================================
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

//#define vAssertCalled(char,int) printf("Sys Error:%s,%d\r\n",char,int)
//#define configASSERT(x) if((x)==0) vAssertCalled(__FILE__,__LINE__)

#define configUSE_PREEMPTION		1                               /* 使能抢占式调度，否则用合作式调度，默认我们都是用抢占式 */
#define configUSE_IDLE_HOOK			0                               /* 空闲任务钩子函数 */
#define configUSE_TICK_HOOK			0                               /* 时基任务钩子函数 */
#define configCPU_CLOCK_HZ			( ( unsigned long ) 72000000 )	/* 系统时钟，单位为HZ */
#define configTICK_RATE_HZ			( ( TickType_t ) 1000 )         /* SysTick中断周期，单位为HZ，1000HZ即1ms中断一次 */
#define configMAX_PRIORITIES		( 10 )                           /* 任务能使用最大优先级个数，数字越大优先级越高，范围为：0~configMAX_PRIORITIES-1
                                                                   最低的0由系统分配给空闲任务，每个任务的优先级可以相同 */
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 128 )      /* 单位为：Word */
#define configTOTAL_HEAP_SIZE		( ( size_t ) ( 50 * 1024 ) )    /* 堆空间大小，内核在创建各种对象时需要用到，单位为Byte */
#define configMAX_TASK_NAME_LEN		( 20 )                        /* 任务名称的长度，即字符串的长度 */
#define configUSE_TRACE_FACILITY	 1              // use vTaskList, =1
#define configUSE_STATS_FORMATTING_FUNCTIONS	0   // use vTaskList, =1
#define configUSE_16_BIT_TICKS		0                             /* SysTick Counter的宽度，0表示32bit，1表示16bit，STM32用的是32bit */
#define configIDLE_SHOULD_YIELD		1                             /* 上下文切换强制使能，即当前任务执行完毕之后还有剩余的时间片，可以强制自己放弃
                                                                   剩余的时间片，然后执行上下文切换去执行其他的任务*/
#define	configCHECK_FOR_STACK_OVERFLOW	2
//#define	configCHECK_FOR_STACK_OVERFLOW	1


#define configSUPPORT_DYNAMIC_ALLOCATION 1  // for xSemaphoreCreateMutex
#define configUSE_MUTEXES 1 // for xQueueCreateMutex

#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	#define xSemaphoreCreateMutex() xQueueCreateMutex( queueQUEUE_TYPE_MUTEX ) // 其中，queueQUEUE_TYPE_MUTEX = 1 
#endif

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		0                             /* 合作式调度配置 */
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* 软件定时器定义 */
#define configUSE_TIMERS				    1
#define configTIMER_TASK_PRIORITY		  	( 2 )
#define configTIMER_QUEUE_LENGTH		  	10
#define configTIMER_TASK_STACK_DEPTH		( configMINIMAL_STACK_SIZE * 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		0
#define INCLUDE_uxTaskPriorityGet		0
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	1
//#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1

/*=========================================== SysTick 和 PendSV 的中断优先级配置 ======================================*/
/* This is the raw value as per the Cortex-M3 NVIC.  Values can be 255
(lowest) to 0 (1?) (highest). */
/*
 * 当把配置好的优先级写到寄存器的时候是按照这个公式来写的：((priority << (8 - __NVIC_PRIO_BITS)) & 0xff) = 255 => priority = 15
 * 因为在Cortex-M 系列中是用8位来表示优先级，操作的时候以8bit来写的，但是ST的处理器只用了其中的高四位，即__NVIC_PRIO_BITS=4。
 *
 * configKERNEL_INTERRUPT_PRIORITY 宏用来配置 SysTick 和 PendSV 的中断优先级，这里配置为 15，即无论中断优先级如何分钟，都是最低。
 * 
 * 当 SysTick 和 PendSV 的中断优先级 配置为最低的时候，可以提供系统的实时性，即外部中断可以随时响应。
 */
#define configKERNEL_INTERRUPT_PRIORITY 		255          
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */

/*===========================================可屏蔽的中断优先级配置====================================================*/
/*
 * 用于配置STM32的特殊寄存器basepri寄存器的值，用于屏蔽中断，当大于basepri值的
 * 优先级的中断将被全部屏蔽。basepri只有4bit有效，默认只为0，即全部中断都没有被屏蔽。
 * 这里191对应的二进制为(1011 1111)b只有高四位有效，即(1011)b=11，意思就是中断优先级大于11的中断都被屏蔽
 *
 * 在FreeRTOS中，关中断是通过配置basepri寄存器来实现的，关掉的中断由配置的basepri的值决定，小于basepri值的
 * 中断FreeRTOS是关不掉的，这样做的好处是可以系统设计者可以人为的控制哪些非常重要的中断不能被关闭，在紧要的关头必须被响应。
 * 而在UCOS中，关中断是通过控制PRIMASK来实现的，PRIMASK是一个单1的二进制位，写1则除能除了NMI和硬 fault的所有中断。当os关闭
 * 中断之后，即使是你在系统中设计的非常紧急的中断来了都不能马上响应，这加大了中断延迟的时间，如果是性命攸关的场合，那后果估计挺严重。
 *
 * 当把配置好的优先级写到寄存器的时候是按照这个公式来写的：((priority << (8 - __NVIC_PRIO_BITS)) & 0xff) = 191 => priority = 11
 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	191 /* equivalent to 0xb0, or priority 11. */


/* This is the value being used as per the ST library which permits 16
priority values, 0 to 15.  This must correspond to the
configKERNEL_INTERRUPT_PRIORITY setting.  Here 15 corresponds to the lowest
NVIC value of 255. */
/*
 * configLIBRARY_KERNEL_INTERRUPT_PRIORITY 宏就是 这个公式：((priority << (8 - __NVIC_PRIO_BITS)) & 0xff)里面的priority
 * 即实际用来配置STM32的中断优先级，但是要写到8位的中断优先级的寄存器里面，还是需要用上面的公式先转化下。通过计算之后就
 * 是上面configKERNEL_INTERRUPT_PRIORITY这个宏的值，用于配置SysTick和PendSV 的中断优先级。
 */
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY	15       // 0x0f


/*=================================== SVC，PendSV 和 SysTick 中断服务函数的配置 ========================================*/
/*
 * 在移植FreeRTOS的时候，需要用到STM32的三个中断，分别是SVC,PendSV和SysTick，这三个中断在向量表
 * 里面的名字分别是：SVC_Handler，PendSV_Handler和SysTick_Handler（在启动文件:startup_stm32f10x_hd.s中）
 * 
 * 而在port.c里面写的这三个中断的服务函数的名称跟向量表里面的名称不一样，为了中断响应之后能正确的执行port.c
 * 里面写好的中断服务函数，我们需要统一向量表和中断服务函数的名字。
 *
 * 为了实现这个目的，可以有两种方法：
 * 1：把启动文件里面的向量表里面的名字改成跟port.c里面的中断服务函数名一样。
 * 2：把port.c里面的中断服务函数名改成跟启动文件里面的中断向量表里面的名字一样。
 *
 * 这里为了保持启动文件的完整性，我们统一修改port.c里面的中断函数名，即添加下面三个宏定义即可。
 * 如果你在stm32f10x_it.c这里面实现了这三个中断服务函数的定义的话，那么为了避免跟port.c里面的重复定义，应该
 * 把stm32f10x_it.c里面的注释掉。
 */

#define xPortPendSVHandler   PendSV_Handler
#define xPortSysTickHandler  SysTick_Handler
#define vPortSVCHandler      SVC_Handler

#endif /* FREERTOS_CONFIG_H */

