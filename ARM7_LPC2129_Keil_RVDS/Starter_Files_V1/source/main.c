/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"
#include "semphr.h"

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

xSemaphoreHandle LED_semaphore ;  /** define handler for the semaphore **/

TaskHandle_t LEDtask_Handler    = NULL ;  /** definition of task handler for led task    **/
TaskHandle_t Buttontask_Handler = NULL ;  /** definition of task handler for button task **/

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

/**
 ** @brief     : task to check the button status 
 ** @arguments : void pointer(to be casted latter when we need to pass arguments)
 ** @return    : void 
 **/
void Button_task( void *TaskParameters)
{
	uint32_t button_State = 0 ; /** local variable to read button **/
	
	for( ; ; )
	{
		button_State = GPIO_read(PORT_0 , PIN0); /** read  button connected to PIN0 , PORT_0 **/
		
		if( button_State == PIN_IS_HIGH ) /** if button is pressed **/
		{
			while( button_State == PIN_IS_HIGH ) /** while it's pressed **/
			{
				vTaskDelay(10); /** polling every 10 ms **/
				
				button_State = GPIO_read(PORT_0 , PIN0); /** read button every 10 ms **/
			}
			xSemaphoreGive(LED_semaphore); /** give the semaphore **/

		}
		
		vTaskDelay(10); /** task periodicity every 10 ms **/
	}
	vTaskDelete(Buttontask_Handler);  /** delete task in case of failure **/
}

/**
 ** @brief     : task to toggle LED 
 ** @arguments : void pointer(to be casted latter when we need to pass arguments)
 ** @return    : void 
 **/
void TogLED_task( void *TaskParameters)
{
	for( ; ; )
	{
		/** try to take the semaphore **/
		if ( xSemaphoreTake(LED_semaphore , (TickType_t) 0 ) == pdTRUE )
		{
			if( GPIO_read(PORT_0 , PIN1) == PIN_IS_HIGH ) /** toggle LED **/
			{
				GPIO_write( PORT_0 , PIN1 , PIN_IS_LOW) ;
			}
			else
			{
				GPIO_write( PORT_0 , PIN1 , PIN_IS_HIGH) ;
			}
		}
		vTaskDelay(10); /** task periodicity 10 ms **/
	
	}
	vTaskDelete(LEDtask_Handler);  /** delete task in case of failure **/
}

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
  
	/** create binary semaphore with initial value 0 (not available) **/
	LED_semaphore = xSemaphoreCreateBinary(); 

	/** Create Button Task here **/
  xTaskCreate(
               Button_task ,             /** task name that's already implemented     **/
               "Button_task",            /** text name for the task                   **/
			   configMINIMAL_STACK_SIZE, /** stack size in words needed for this task **/
	           ( void *)NULL,            /** parameters that's passed to the task     **/
			   2 ,                       /** task priority                            **/
			   &Buttontask_Handler );       /** task handler which is already initialized to NULL **/

	
  /** Create LED Task here **/
  xTaskCreate(
               TogLED_task ,             /** task name that's already implemented     **/
               "TogLED_task",            /** text name for the task                   **/
			   configMINIMAL_STACK_SIZE, /** stack size in words needed for this task **/
	           ( void *)NULL,            /** parameters that's passed to the task     **/
			   1 ,                       /** task priority                            **/
			   &LEDtask_Handler );       /** task handler which is already initialized to NULL **/

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


