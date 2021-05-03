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
#include "task.h"
#include "stm32f4xx.h"
#include <string.h>
#include<stdio.h>

TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;

void vTask1_handler(void *params);
void vTask2_handler(void *params);
#ifdef USE_SEMIHOSTING
extern void initialise_monitor_handles();
#endif
static void prvSetupHardware(void);
static void prvSetupUart(void);

void printmsg(char *msg);

//SOME MACROS
#define TRUE 1
#define FALSE 0
#define AVAILABLE TRUE
#define NOT_AVAILABLE FALSE

//GLOBAL VARIABLES
char usr_msg[250];
uint8_t UART_ACCESS_KEY = AVAILABLE;

int main(void)
{
#ifdef USE_SEMIHOSTING
	initialise_monitor_handles();
	printf("this is hello application \n");
#endif

	DWT->CTRL |= (1<<0); //ENABLE CYCCNT IN DWT_CTRL.

	RCC_DeInit();

	SystemCoreClockUpdate();

	prvSetupHardware();

	sprintf(usr_msg,"THIS IS TIME TO TEST UART OF STM32\r\n");

	printmsg(usr_msg);

	//TO START SEGGER RECORDING
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	xTaskCreate(vTask1_handler, "Task-1", configMINIMAL_STACK_SIZE, NULL, 2,
				&xTaskHandle1);

	xTaskCreate(vTask2_handler, "Task-2", configMINIMAL_STACK_SIZE, NULL, 2,
					&xTaskHandle2);

	vTaskStartScheduler();
	for(;;);
}

void vTask1_handler(void *params)
{

	while(1)
	{
		if(UART_ACCESS_KEY == AVAILABLE)
		{
			UART_ACCESS_KEY = NOT_AVAILABLE;
			printmsg("hello from TASK-1\r\n");
			UART_ACCESS_KEY = AVAILABLE;
			taskYIELD();
		}
	}
}

void vTask2_handler(void *params)
{

	while(1)
	{
		if(UART_ACCESS_KEY == AVAILABLE)
				{
					UART_ACCESS_KEY = NOT_AVAILABLE;
					printmsg("hello from TASK-2 \r\n");
					UART_ACCESS_KEY = AVAILABLE;
					taskYIELD();
				}
	}

}

static void prvSetupUart(void)
{
	GPIO_InitTypeDef gpio_uart_pins;
		USART_InitTypeDef uart2_init;

		//1.enable UART peripheral and GPIOA peripheral clock
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

		//2. PA2 IS UART2_TX AND PA3 is UART2_RX
		// Alternate function configuration of MCU pin to behave as UART2 TX AND RX
		//void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
		// zeroing each and every member element of the structure
		memset(&gpio_uart_pins,0,sizeof(gpio_uart_pins));


		gpio_uart_pins.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
		gpio_uart_pins.GPIO_Mode = GPIO_Mode_AF;
		gpio_uart_pins.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA,&gpio_uart_pins);

		//3. AF mode setting for the pins
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,GPIO_AF_USART2); //PA2 is as Tx
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,GPIO_AF_USART2); //PA3 is as Rx

		//4. UART Parameter Initializations
		memset(&uart2_init,0,sizeof(uart2_init));
		uart2_init.USART_BaudRate = 115200;
		uart2_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		uart2_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		uart2_init.USART_Parity = USART_Parity_No;
		uart2_init.USART_StopBits = USART_StopBits_1;
		uart2_init.USART_WordLength = USART_WordLength_8b;
		USART_Init(USART2,&uart2_init);

		//5.Enable the USART peripheral
		USART_Cmd(USART2,ENABLE);

}

static void prvSetupHardware(void)
{
	//setup UART
	prvSetupUart();
}

void printmsg(char *msg)
{
	for(uint32_t i=0;i<strlen(msg);i++)
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) != SET); //to check that buffer is empty or not
		USART_SendData(USART2,msg[i]);
	}
}
