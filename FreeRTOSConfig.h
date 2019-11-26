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
 *=================================== ����ע�� =======================================
 * FreeRTOSConfig.h���ͷ�ļ���������FreeRTOS�Ĺ��ܣ����ǲ���ȫ�棬ʣ�µ��������ܿ���
 * ��FreeRTOS.h���ͷ�ļ����ã�����ʹ�ܻ����ź�����ʹ�ܵݹ��ź�����ʹ���¼���־����Щ
 * �ں˶���Ĺ���ʱ������FreeRTOS.h���ͷ�ļ��������õġ���Ҫ����ȫʹ��FreeRTOS�Ĺ���
 * ��Ҫ������ͷ�ļ�һ������ʹ�á�
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

#define configUSE_PREEMPTION		1                               /* ʹ����ռʽ���ȣ������ú���ʽ���ȣ�Ĭ�����Ƕ�������ռʽ */
#define configUSE_IDLE_HOOK			0                               /* ���������Ӻ��� */
#define configUSE_TICK_HOOK			0                               /* ʱ�������Ӻ��� */
#define configCPU_CLOCK_HZ			( ( unsigned long ) 72000000 )	/* ϵͳʱ�ӣ���λΪHZ */
#define configTICK_RATE_HZ			( ( TickType_t ) 1000 )         /* SysTick�ж����ڣ���λΪHZ��1000HZ��1ms�ж�һ�� */
#define configMAX_PRIORITIES		( 10 )                           /* ������ʹ��������ȼ�����������Խ�����ȼ�Խ�ߣ���ΧΪ��0~configMAX_PRIORITIES-1
                                                                   ��͵�0��ϵͳ�������������ÿ����������ȼ�������ͬ */
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 128 )      /* ��λΪ��Word */
#define configTOTAL_HEAP_SIZE		( ( size_t ) ( 50 * 1024 ) )    /* �ѿռ��С���ں��ڴ������ֶ���ʱ��Ҫ�õ�����λΪByte */
#define configMAX_TASK_NAME_LEN		( 20 )                        /* �������Ƶĳ��ȣ����ַ����ĳ��� */
#define configUSE_TRACE_FACILITY	 1              // use vTaskList, =1
#define configUSE_STATS_FORMATTING_FUNCTIONS	0   // use vTaskList, =1
#define configUSE_16_BIT_TICKS		0                             /* SysTick Counter�Ŀ�ȣ�0��ʾ32bit��1��ʾ16bit��STM32�õ���32bit */
#define configIDLE_SHOULD_YIELD		1                             /* �������л�ǿ��ʹ�ܣ�����ǰ����ִ�����֮����ʣ���ʱ��Ƭ������ǿ���Լ�����
                                                                   ʣ���ʱ��Ƭ��Ȼ��ִ���������л�ȥִ������������*/
#define	configCHECK_FOR_STACK_OVERFLOW	2
//#define	configCHECK_FOR_STACK_OVERFLOW	1


#define configSUPPORT_DYNAMIC_ALLOCATION 1  // for xSemaphoreCreateMutex
#define configUSE_MUTEXES 1 // for xQueueCreateMutex

#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
	#define xSemaphoreCreateMutex() xQueueCreateMutex( queueQUEUE_TYPE_MUTEX ) // ���У�queueQUEUE_TYPE_MUTEX = 1 
#endif

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		0                             /* ����ʽ�������� */
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* �����ʱ������ */
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

/*=========================================== SysTick �� PendSV ���ж����ȼ����� ======================================*/
/* This is the raw value as per the Cortex-M3 NVIC.  Values can be 255
(lowest) to 0 (1?) (highest). */
/*
 * �������úõ����ȼ�д���Ĵ�����ʱ���ǰ��������ʽ��д�ģ�((priority << (8 - __NVIC_PRIO_BITS)) & 0xff) = 255 => priority = 15
 * ��Ϊ��Cortex-M ϵ��������8λ����ʾ���ȼ���������ʱ����8bit��д�ģ�����ST�Ĵ�����ֻ�������еĸ���λ����__NVIC_PRIO_BITS=4��
 *
 * configKERNEL_INTERRUPT_PRIORITY ���������� SysTick �� PendSV ���ж����ȼ�����������Ϊ 15���������ж����ȼ���η��ӣ�������͡�
 * 
 * �� SysTick �� PendSV ���ж����ȼ� ����Ϊ��͵�ʱ�򣬿����ṩϵͳ��ʵʱ�ԣ����ⲿ�жϿ�����ʱ��Ӧ��
 */
#define configKERNEL_INTERRUPT_PRIORITY 		255          
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */

/*===========================================�����ε��ж����ȼ�����====================================================*/
/*
 * ��������STM32������Ĵ���basepri�Ĵ�����ֵ�����������жϣ�������basepriֵ��
 * ���ȼ����жϽ���ȫ�����Ρ�basepriֻ��4bit��Ч��Ĭ��ֻΪ0����ȫ���ж϶�û�б����Ρ�
 * ����191��Ӧ�Ķ�����Ϊ(1011 1111)bֻ�и���λ��Ч����(1011)b=11����˼�����ж����ȼ�����11���ж϶�������
 *
 * ��FreeRTOS�У����ж���ͨ������basepri�Ĵ�����ʵ�ֵģ��ص����ж������õ�basepri��ֵ������С��basepriֵ��
 * �ж�FreeRTOS�ǹز����ģ��������ĺô��ǿ���ϵͳ����߿�����Ϊ�Ŀ�����Щ�ǳ���Ҫ���жϲ��ܱ��رգ��ڽ�Ҫ�Ĺ�ͷ���뱻��Ӧ��
 * ����UCOS�У����ж���ͨ������PRIMASK��ʵ�ֵģ�PRIMASK��һ����1�Ķ�����λ��д1����ܳ���NMI��Ӳ fault�������жϡ���os�ر�
 * �ж�֮�󣬼�ʹ������ϵͳ����Ƶķǳ��������ж����˶�����������Ӧ����Ӵ����ж��ӳٵ�ʱ�䣬������������صĳ��ϣ��Ǻ������ͦ���ء�
 *
 * �������úõ����ȼ�д���Ĵ�����ʱ���ǰ��������ʽ��д�ģ�((priority << (8 - __NVIC_PRIO_BITS)) & 0xff) = 191 => priority = 11
 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	191 /* equivalent to 0xb0, or priority 11. */


/* This is the value being used as per the ST library which permits 16
priority values, 0 to 15.  This must correspond to the
configKERNEL_INTERRUPT_PRIORITY setting.  Here 15 corresponds to the lowest
NVIC value of 255. */
/*
 * configLIBRARY_KERNEL_INTERRUPT_PRIORITY ����� �����ʽ��((priority << (8 - __NVIC_PRIO_BITS)) & 0xff)�����priority
 * ��ʵ����������STM32���ж����ȼ�������Ҫд��8λ���ж����ȼ��ļĴ������棬������Ҫ������Ĺ�ʽ��ת���¡�ͨ������֮���
 * ������configKERNEL_INTERRUPT_PRIORITY������ֵ����������SysTick��PendSV ���ж����ȼ���
 */
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY	15       // 0x0f


/*=================================== SVC��PendSV �� SysTick �жϷ����������� ========================================*/
/*
 * ����ֲFreeRTOS��ʱ����Ҫ�õ�STM32�������жϣ��ֱ���SVC,PendSV��SysTick���������ж���������
 * ��������ֱַ��ǣ�SVC_Handler��PendSV_Handler��SysTick_Handler���������ļ�:startup_stm32f10x_hd.s�У�
 * 
 * ����port.c����д���������жϵķ����������Ƹ���������������Ʋ�һ����Ϊ���ж���Ӧ֮������ȷ��ִ��port.c
 * ����д�õ��жϷ�������������Ҫͳһ��������жϷ����������֡�
 *
 * Ϊ��ʵ�����Ŀ�ģ����������ַ�����
 * 1���������ļ��������������������ָĳɸ�port.c������жϷ�������һ����
 * 2����port.c������жϷ��������ĳɸ������ļ�������ж����������������һ����
 *
 * ����Ϊ�˱��������ļ��������ԣ�����ͳһ�޸�port.c������жϺ���������������������궨�弴�ɡ�
 * �������stm32f10x_it.c������ʵ�����������жϷ������Ķ���Ļ�����ôΪ�˱����port.c������ظ����壬Ӧ��
 * ��stm32f10x_it.c�����ע�͵���
 */

#define xPortPendSVHandler   PendSV_Handler
#define xPortSysTickHandler  SysTick_Handler
#define vPortSVCHandler      SVC_Handler

#endif /* FREERTOS_CONFIG_H */

