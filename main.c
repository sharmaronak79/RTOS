/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#include "FreeRTOS.h"
#include "stm32f4xx.h"

#include "task.h"
			
TaskHandle_t xTaskHandle1=NULL;
TaskHandle_t xTaskHandle2=NULL;

//prototypes of functions
void vTask1_handler ( void *params );
void vTask2_handler ( void *params );

int main(void)
{
	//1. Resets the RCC clock configuration to the default reset state.
	//HSI ON, HSE OFF , PLL OFF SYSTEM CLOCK = 16MHZ, cpu_clock=16MHZ
	RCC_DeInit();

	//2.Update the systemCoreColck variable
	SystemCoreClockUpdate();

	//3. lets create two tasks, for that we have to incude tasks.h header file
	xTaskCreate( vTask1_handler,"Task-1",configMINIMAL_STACK_SIZE,NULL,2,&xTaskHandle1 );

	xTaskCreate( vTask2_handler,"Task-2",configMINIMAL_STACK_SIZE,NULL,2,&xTaskHandle2 );

	vTaskStartScheduler();

	for(;;);
}

void vTask1_handler(void *params)
{

while(1);

}

void vTask2_handler(void *params)
{

	while(1);

}




