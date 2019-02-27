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
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

uint32_t RED = 0b001;
uint32_t YELLOW = 0b010;
uint32_t GREEN = 0b100;

uint16_t YELLOW_SLEEP_TIME = 2000;
uint16_t RED_SLEEP_TIME = 2000;
uint16_t GREEN_SLEEP_TIME = 3000;

uint32_t max_pot_val = 4095;
TimerHandle_t timer;
uint32_t traffic_light = 0b100;

uint32_t lsb_temp = 0b11111111000;
uint32_t msb_temp = 0b1111111111100000000000;

uint32_t NEW_CAR = 0b1000;


/*-----------------------------------------------------------*/
#define QUEUE_LENGTH 1

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */

xQueueHandle TrafficAdjustmentQueue;
xQueueHandle LightAdjustmentQueue;
xQueueHandle DisplayTrafficQueue;
xQueueHandle DisplayLightQueue;

/*-----------------------------------------------------------*/

static void Init_Shift_Registers(){
	SPI_InitTypeDef SPI_1;
	GPIO_InitTypeDef GPIO_A;

	//Init Clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	//Set GPIO pin configs
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	//Init GPIO
	GPIO_A.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_A.GPIO_Mode = GPIO_Mode_AF;
	GPIO_A.GPIO_OType = GPIO_OType_PP;
	GPIO_A.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_A.GPIO_Speed = GPIO_Speed_50MHz,
	GPIO_Init(GPIOA, &GPIO_A);

	//Init SPI
	SPI_I2S_DeInit(SPI1);
	SPI_1.SPI_Mode = SPI_Mode_Master;
	SPI_1.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_1.SPI_DataSize = SPI_DataSize_8b;
	SPI_1.SPI_CPHA = SPI_CPOL_Low;
	SPI_1.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_1.SPI_NSS = SPI_NSS_Soft;
	SPI_1.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_1.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_1.SPI_CRCPolynomial = 7; //might not need
	SPI_Init(SPI1,&SPI_1);
	SPI_Cmd(SPI1, ENABLE);
}

static void Send_Data_To_Shift_Registers(uint32_t light_setup) {
	SPI_I2S_SendData(SPI1, light_setup >> 14);
	while ((SPI1->SR & SPI_SR_TXE) == 0);// might be able to remove
	while (SPI1->SR & SPI_SR_BSY);
	SPI_I2S_SendData(SPI1, light_setup >> 7);
	while ((SPI1->SR & SPI_SR_TXE) == 0);
	while (SPI1->SR & SPI_SR_BSY);
	SPI_I2S_SendData(SPI1, light_setup);
	while ((SPI1->SR & SPI_SR_TXE) == 0);
	while (SPI1->SR & SPI_SR_BSY);
}

void Init_Potentiometer(){
	ADC_InitTypeDef ADC_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	//Init clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	//Init GPIO for ADC
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	//Init ADC to read potentiometer with regular channel
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_Cmd(ADC1, ENABLE);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_84Cycles);
}


void TrafficAdjustmentTask(){
	uint16_t pot_val = 0;
	for(;;){
		ADC_SoftwareStartConv(ADC1);
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
		//read pot val
		pot_val = ADC_GetConversionValue(ADC1);
		printf("VAL: %d\n",pot_val);
		//send pot val to both queues
		if(xQueueSend(TrafficAdjustmentQueue,&pot_val,500)){
			xQueueReset(LightAdjustmentQueue);
			xQueueSend(LightAdjustmentQueue,&pot_val,50);
		}
		vTaskDelay(250);
	}
}

int newCarArrival(uint16_t pot_val, uint16_t counter){
	if (counter*250 >= pot_val){
		return 1;
	}
	return 0;
}

void TrafficCreatorTask(){
	uint16_t pot_val;
	int counter=1;
	uint16_t val = 1;
	//xQueueSend(DisplayTrafficQueue,&val,2000);
	for(;;){
		if(xQueueReceive(TrafficAdjustmentQueue, &pot_val, 2000)){
			if(newCarArrival(pot_val, counter)){
				uint16_t arrived = 1;
				xQueueSend(DisplayTrafficQueue,&arrived,2000);
				counter=1;
			}
			counter++;
		}
	}
}

void vTimerCallback( TimerHandle_t xTimer ){
	uint16_t pot_val;
	xQueueReceive(LightAdjustmentQueue, &pot_val, 2000);
	float normalized_pot_val = max_pot_val-(max_pot_val-pot_val);
	uint32_t val = 0;
	if(traffic_light==GREEN){
		val=YELLOW_SLEEP_TIME+normalized_pot_val;
		traffic_light=YELLOW;
	}else if(traffic_light==RED){
		val=GREEN_SLEEP_TIME+normalized_pot_val;
		traffic_light=GREEN;
	}else{
		val=RED_SLEEP_TIME+normalized_pot_val;
		traffic_light=RED;
	}
	xTimerChangePeriod(timer, val, 0);
}

//taken from https://www.geeksforgeeks.org/extract-k-bits-given-position-number/
uint32_t get_bits(uint32_t number, uint32_t k, uint32_t p) {
	 return (((1 << k) - 1) & (number >> (p - 1)));
}

uint32_t shift_traffic(uint32_t setup, uint16_t new_car){
	uint32_t pointer = 0b10000000;
	uint32_t counter=0;
	uint32_t offset=8;
	uint32_t movable_traffic;
	while(pointer>0){
		if(((setup&pointer)>>(offset-counter-1))==0){
			movable_traffic = get_bits(setup,offset-counter, 1);
			movable_traffic = movable_traffic<<1;
			if(new_car){
				movable_traffic = 1|movable_traffic;
			}
			uint32_t left_bits = get_bits(setup, counter, offset-counter+1)<<(offset-counter);
			return (movable_traffic|left_bits)<<3;
		}
		pointer=pointer>>1;
		counter++;
	}
	if(new_car){
		setup = NEW_CAR|setup;
	}
	return setup<<3;
}


void TrafficDisplayTask(){
	uint32_t traffic_setup=0;
	uint32_t final_bits=0;
	uint16_t new_car;
	uint32_t lsb;
	uint32_t msb;
	uint32_t shifted_traffic;
	for(;;){
		new_car=0;
		xQueueReceive(DisplayTrafficQueue, &new_car, 2000);
		if(traffic_light == RED || traffic_light == YELLOW){
			lsb = traffic_setup&lsb_temp;
			msb = traffic_setup&msb_temp;
			shifted_traffic = shift_traffic(lsb>>3, new_car);
			msb=msb<<1; // keep the intersection and onwards moving
			traffic_setup = shifted_traffic|msb;
		}else{
			//green light
			traffic_setup = traffic_setup<<1;
			if(new_car){
				//add 1 to left of traffic_setup
				traffic_setup = traffic_setup|NEW_CAR;
			}
		}

		final_bits = traffic_setup|traffic_light;
		printf("TRAFFIC_LIGHT: %d\n",traffic_light);
		printf("TRAFFIC_SETUP: %d\n",traffic_setup);
		Send_Data_To_Shift_Registers(final_bits);
	}
}


int main(void)
{
	//initialize potentiometer and shift register
	Init_Potentiometer();
	Init_Shift_Registers();
	Send_Data_To_Shift_Registers(0);
	timer = xTimerCreate("sleep_light",GREEN_SLEEP_TIME, pdTRUE,( void * ) 0, vTimerCallback);
	xTimerStart(timer,0);

	prvSetupHardware();

	TrafficAdjustmentQueue = xQueueCreate(QUEUE_LENGTH, sizeof( uint16_t ));
	LightAdjustmentQueue = xQueueCreate(QUEUE_LENGTH, sizeof( uint16_t ));
	DisplayTrafficQueue = xQueueCreate(QUEUE_LENGTH, sizeof( uint16_t ));
	DisplayLightQueue = xQueueCreate(QUEUE_LENGTH, sizeof( uint32_t ));


	vQueueAddToRegistry( TrafficAdjustmentQueue, "TrafficQueue" );
	vQueueAddToRegistry( LightAdjustmentQueue, "LightQueue" );
	vQueueAddToRegistry( DisplayTrafficQueue, "DisplayTrafficQueue" );
	vQueueAddToRegistry( DisplayLightQueue, "DisplayLightQueue" );

	xTaskCreate( TrafficAdjustmentTask, "Adjustment", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( TrafficCreatorTask, "Creator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( TrafficDisplayTask, "Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	vTaskStartScheduler();
	return 0;
}


void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}

