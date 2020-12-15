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
#include<stdint.h>
#include<string.h>
#include "FreeRTOS.h"
#include "stm32f4xx.h"

#include "task.h"

TaskHandle_t xTaskHandle1=NULL;
TaskHandle_t xTaskHandle2=NULL;

//prototypes of functions
void vTask1_handler ( void *params );
void vTask2_handler ( void *params );

//used for semihosting
#ifdef USE_SEMIOSTING
extern void initialise_monitor_handles();
#endif
static void prvSetupHardware(void);



int main(void)
{
	//preprocessor macro
#ifdef USE_SEMIOSTING //for printing  on console related to semihosting
	initialise_monitor_handles();
	printf("This is helloworld practise \n");
#endif

	//1. Resets the RCC clock configuration to the default reset state.
	//HSI ON, HSE OFF , PLL OFF SYSTEM CLOCK = 16MHZ, cpu_clock=16MHZ
	RCC_DeInit();

	//2.Update the systemCoreColck variable
	SystemCoreClockUpdate();

	prvSetupHardware();

	//3. lets create two tasks, for that we have to incude tasks.h header file
	xTaskCreate( vTask1_handler,"Task-1",configMINIMAL_STACK_SIZE,NULL,2,&xTaskHandle1 );

	xTaskCreate( vTask2_handler,"Task-2",configMINIMAL_STACK_SIZE,NULL,2,&xTaskHandle2 );



	vTaskStartScheduler();

	for(;;);
}

void vTask1_handler(void *params)
{

while(1){

}

}

void vTask2_handler(void *params)
{

	while(1){

	}

}

static void prvSetupHardware(void)
{
	GPIO_InitTypeDef gpio_uart_pins;
	USART_InitTypeDef uart2_Init;
	//1.Enable UART2 and GPIOA peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	//PA2 IS TX OF MCU AND PA3 IS RX OF MCU
	//2.Alternate function configuration of MCU pins to behave as UART2 Tx and Rx

	// Zeroing each and every member element of of he structure
	memset(&gpio_uart_pins,0,sizeof(gpio_uart_pins));

	gpio_uart_pins.GPIO_Pin=GPIO_Pin_2| GPIO_Pin_3 ;
	gpio_uart_pins.GPIO_Mode=GPIO_Mode_AF;
	gpio_uart_pins.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&gpio_uart_pins);

	//3.AF mode setting for the pins
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //PA2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //PA3

	//4.UART parameter initialization

	// Zeroing each and every member element of of he structure
		memset(&uart2_Init,0,sizeof(uart2_Init));

	uart2_Init.USART_BaudRate=115200;
	uart2_Init.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	uart2_Init.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
	uart2_Init.USART_Parity=USART_Parity_No;
	uart2_Init.USART_StopBits=USART_Parity_No;
	uart2_Init.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART2, &uart2_Init);


}

