/* USER CODE BEGIN Header */
/*
 * FreeRTOS Kernel V10.3.1
 * Portion Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Portion Copyright (C) 2019 StMicroelectronics, Inc.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */
/* USER CODE END Header */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * These parameters and more are described within the 'configuration' section of the
 * FreeRTOS API documentation available on the FreeRTOS.org web site.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/* Section where include file can be added */
/* USER CODE END Includes */

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
  void xPortSysTickHandler(void);
#endif
// Enable FPU support
#define configENABLE_FPU                         0
// Enable MPU support
#define configENABLE_MPU                         0

// Use preemption
#define configUSE_PREEMPTION                     1
// Support static allocation
#define configSUPPORT_STATIC_ALLOCATION          1
// Support dynamic allocation
#define configSUPPORT_DYNAMIC_ALLOCATION         1
// Use idle hook
#define configUSE_IDLE_HOOK                      0
// Use tick hook
#define configUSE_TICK_HOOK                      0
// CPU clock frequency
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
// Tick rate in Hz
#define configTICK_RATE_HZ                       ((TickType_t)1000)
// Maximum priorities
#define configMAX_PRIORITIES                     ( 7 )
// Minimal stack size
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
// Total heap size
#define configTOTAL_HEAP_SIZE                    ((size_t)15360)
// Maximum task name length
#define configMAX_TASK_NAME_LEN                  ( 16 )
// Use 16-bit tick type
#define configUSE_16_BIT_TICKS                   0
// Use mutexes
#define configUSE_MUTEXES                        1
// Use recursive mutexes
#define configUSE_RECURSIVE_MUTEXES              1
// Use counting semaphores
#define configUSE_COUNTING_SEMAPHORES            1
// Queue registry size
#define configQUEUE_REGISTRY_SIZE                8
// Use trace facility
#define configUSE_TRACE_FACILITY                 1
// Use newlib reentrant
#define configUSE_NEWLIB_REENTRANT               1
// Use application task tag
#define configUSE_APPLICATION_TASK_TAG           0
// Use stats formatting functions
#define configUSE_STATS_FORMATTING_FUNCTIONS     1
// Generate run time stats
#define configGENERATE_RUN_TIME_STATS            1

// Use port optimized task selection
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
// Task notifications array size
#define configTASK_NOTIFICATION_ARRAY_ENTRIES    1
// Use task notifications
#define configUSE_TASK_NOTIFICATIONS             1
// Stack overflow checking method
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP 0
// Check for stack overflow
#define configCHECK_FOR_STACK_OVERFLOW           0
// Number of legacy task reset priority deprecated
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS  0
// Use daemon task startup hook
#define configUSE_DAEMON_TASK_STARTUP_HOOK       0
// Enable backward compatibility
#define configENABLE_BACKWARD_COMPATIBILITY      0
// Use tickless idle
#define configUSE_TICKLESS_IDLE                  0

// Use timers
#define configUSE_TIMERS                         1
// Timer task priority
#define configTIMER_TASK_PRIORITY                ( 2 )
// Timer task stack size
#define configTIMER_TASK_STACK_DEPTH             128
// Timer queue registry size
#define configTIMER_QUEUE_REGISTRY_SIZE          0

// Set to 1 to include the vTaskCleanUpResources() function
#define configUSE_MALLOC_FAILED_HOOK             0

// Set to desired log number (0==no log)
#define configSTATS_BUFFER_MAX_LENGTH            0xFFFFFFFF

// Software timer definitions
#define configTIMER_TASK_PRIORITY                ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             ( configMINIMAL_STACK_SIZE * 2 )

// Set the following definitions to 1 to include the API function, or zero to exclude
#define INCLUDE_vTaskPrioritySet             1
#define INCLUDE_uxTaskPriorityGet            1
#define INCLUDE_vTaskDelete                  1
#define INCLUDE_vTaskCleanUpResources        0
#define INCLUDE_vTaskSuspend                 1
#define INCLUDE_vTaskDelayUntil              0
#define INCLUDE_vTaskDelay                   1
#define INCLUDE_xTaskGetSchedulerState       1

// Cortex-M specific definitions
#ifdef __NVIC_PRIO_BITS
 // __BVIC_PRIO_BITS will be specified when CMSIS is being used
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS         4
#endif

// The lowest interrupt priority that can be used in a call to a "set priority" function
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

// The highest interrupt priority that can be used by any interrupt service routine that makes calls to interrupt safe FreeRTOS API functions
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

// Interrupt priorities used by the kernel port layer itself
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
// configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

// Normal assert() semantics without relying on the provision of an assert.h header file
/* USER CODE BEGIN 1 */
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
/* USER CODE END 1 */

// Definitions that map the FreeRTOS port interrupt handlers to their CMSIS standard names
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

// IMPORTANT: This define is commented when used with STM32Cube firmware, when the timebase source is SysTick, to prevent overwriting SysTick_Handler defined within STM32Cube HAL
/* #define xPortSysTickHandler SysTick_Handler */

/* USER CODE BEGIN Defines */
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
/* USER CODE END Defines */

#endif /* FREERTOS_CONFIG_H */