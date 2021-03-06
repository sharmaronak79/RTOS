/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#include<stdio.h>
#include<stdlib.h>
#include "FreeRTOS.h"
#include "stm32f4xx.h"

#include "task.h"
			
TaskHandle_t xTaskHandle1=NULL;
TaskHandle_t xTaskHandle2=NULL;

//prototypes of functions
void vTask1_handler ( void *params );
void vTask2_handler ( void *params );

//used for semihosting
extern void initialise_monitor_handles();


int main(void)
{
	//for printing  on console related to semihosting
	initialise_monitor_handles();


	//1. Resets the RCC clock configuration to the default reset state.
	//HSI ON, HSE OFF , PLL OFF SYSTEM CLOCK = 16MHZ, cpu_clock=16MHZ
	RCC_DeInit();

	//2.Update the systemCoreColck variable
	SystemCoreClockUpdate();

	//3. lets create two tasks, for that we have to incude tasks.h header file
	xTaskCreate( vTask1_handler,"Task-1",configMINIMAL_STACK_SIZE,NULL,2,&xTaskHandle1 );

	xTaskCreate( vTask2_handler,"Task-2",configMINIMAL_STACK_SIZE,NULL,2,&xTaskHandle2 );

	printf("This is helloworld practise \n");

	vTaskStartScheduler();

	for(;;);
}

void vTask1_handler(void *params)
{

while(1){
	printf("message from Task-1\n");
}

}

void vTask2_handler(void *params)
{

	while(1){
		printf("message from Task-2\n");
	}

}




