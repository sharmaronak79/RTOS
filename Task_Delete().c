/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#include<stdlib.h>
#include<stdio.h>
#include<string.h>
#include<stdint.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;

#define TRUE 1
#define FALSE 0
#define NOT_PRESSED FALSE
#define PRESSED TRUE
//#define delay vTaskDelay(100)

char usr_msg[250];


//Functions prototype

void printmsg(char *msg);
void prvSetupGPIO(void);
static void prvSetupUart(void);
static void prvSetupHardware(void);
void vTask1_handler(void *params);
void vTask2_handler(void *params);

//Global spaces

uint8_t button_status_flag = NOT_PRESSED;

void SEGGER_SYSVIEW_Conf();
void SEGGER_SYSVIEW_Start();


int main(void)
{

	DWT->CTRL |= (1<<0); //ENABLE CYCCNT IN DWT_CTRL.

	RCC_DeInit();

	SystemCoreClockUpdate();

	prvSetupHardware();

	sprintf(usr_msg,"This is demo of delete_TASK APIs\r\n");
	printmsg(usr_msg);

	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();


	//Lets create led task
	xTaskCreate(vTask1_handler,"TASK-1",500,NULL,1,&xTaskHandle1);

	//Lets create button task
	xTaskCreate(vTask2_handler,"TASK-2",500,NULL,2,&xTaskHandle2);

	//Lets start tghe scheduler
	vTaskStartScheduler();

	for(;;);
}

void vTask1_handler(void *params)
{
	TickType_t current_tick = 0;
	sprintf(usr_msg,"Task-1 is running \r\n"); //ORANGE LED
	printmsg(usr_msg);
	while(1)
	{
		vTaskDelay(200);
		GPIO_ToggleBits(GPIOD,GPIO_Pin_13);

	}
}

void vTask2_handler(void *params)
{
	TickType_t current_tick =0;
	sprintf(usr_msg,"Task-2 is running\r\n"); //GREEN LED
	printmsg(usr_msg);

	while(1)
	{
		if( !GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0))
				{
					vTaskDelay(200);
					GPIO_ToggleBits(GPIOD,GPIO_Pin_12);


				}
		else
			{
			sprintf(usr_msg,"Task-2 is grtting deleted\r\n");
			printmsg(usr_msg);
			vTaskDelete(NULL);
			}
	}
}

/*void led_task_handler(void *params)
{
	while(1)
	{
     //wait until any notification received from button task
		if(xTaskNotifyWait(0,0,NULL,0xffffffff) == pdTRUE)
		{
			GPIO_ToggleBits(GPIOD,GPIO_Pin_12);
			sprintf(usr_msg,"Notification is received");
		}
	}
}*/

/*void button_task_handler(void *params)
{
	while(1)
	{
		if(! GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0))
		{
			//we will come here if button is pressed

			// wait here for 100ms to compenaste for the button debouncing
			delay;

			//lets send the notification to led task
			xTaskNotify(xTaskHandle1,0x0,eNoAction);

		}

	}
}
*/

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
	//setup LED and GPIO
	prvSetupGPIO();
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

void prvSetupGPIO(void)
{
	// first we have to enable the bus on which this are connected
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);


	//this function is board specific
	GPIO_InitTypeDef led_init, button_init;
	led_init.GPIO_Mode = GPIO_Mode_OUT;
	led_init.GPIO_OType = GPIO_OType_PP;
	led_init.GPIO_Pin = GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	led_init.GPIO_Speed = GPIO_Low_Speed;
	led_init.GPIO_PuPd = GPIO_PuPd_NOPULL ;

	GPIO_Init(GPIOD,&led_init);

	button_init.GPIO_Mode = GPIO_Mode_IN;
	button_init.GPIO_OType = GPIO_OType_PP;
	button_init.GPIO_Pin = GPIO_Pin_0;
	button_init.GPIO_Speed = GPIO_Low_Speed;
	button_init.GPIO_PuPd = GPIO_PuPd_NOPULL ;

	GPIO_Init(GPIOA,&button_init);


}


