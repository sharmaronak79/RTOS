/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include<string.h>
#include<stdint.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#define TRUE 1
#define FALSE 0
#define NOT_PRESSED FALSE
#define PRESSED TRUE
#define delay vTaskDelay(250)


//Functions prototype

void printmsg(char *msg);
void prvSetupGPIO(void);
static void prvSetupUart(void);
static void prvSetupHardware(void);
void button_handler(void *params);
void led_task_handler(void *params);

//Global spaces

uint8_t button_status_flag = NOT_PRESSED;



int main(void)
{

	RCC_DeInit();

	SystemCoreClockUpdate();

	prvSetupHardware();

	//Lets create led task
	xTaskCreate(led_task_handler,"LED-Task",configMINIMAL_STACK_SIZE,NULL,1,NULL);


	//Lets start tghe scheduler
	vTaskStartScheduler();

	for(;;);
}

void led_task_handler(void *params)
{
	while(1)
	{
		if (button_status_flag == PRESSED)
		{
			//turn on the led

			GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_SET);
			delay;
			GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_SET);
			delay;
			GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_SET);
			delay;
			GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_SET);
			delay;

		}
		else
		{
			//turn off the led
			GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_RESET);
			GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_RESET);
			GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET);
			GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_RESET);
		}

	}
}

void button_handler(void *params)
{

	button_status_flag ^= 1;

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
	//we also have to enable APB2 peripheral bus, because SYSCFG is connected to APB2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);




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

	//Interrupt configuration for the button PA0
	//1. System configuration for EXTI line (SYSCFG setting)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);


	//2. EXTI line configuration 0,interrupt mode , falling edge
	EXTI_InitTypeDef exti_init;
	exti_init.EXTI_Line=EXTI_Line0;
	exti_init.EXTI_Mode=EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger=EXTI_Trigger_Falling;
	exti_init.EXTI_LineCmd=ENABLE;
	EXTI_Init(&exti_init);

	//3.NVIC settings for (IRQ settings for the selected EXTI line(10)
	NVIC_SetPriority(EXTI0_IRQn,5);
	NVIC_EnableIRQ(EXTI0_IRQn);


}

void EXTI0_IRQHandler(void)
{
	//to trace the IRQ entry and exit in segger
	traceISR_ENTER();
	//1.clear the interrupt pending bit of the EXTI line(0)
	EXTI_ClearITPendingBit(EXTI_Line0);
	button_handler(NULL);
	traceISR_EXIT();

}






