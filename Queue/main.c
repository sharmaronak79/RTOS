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
#include "queue.h"
#include "timers.h"

#define TRUE 1
#define FALSE 0
#define NOT_PRESSED FALSE
#define PRESSED TRUE
//#define delay vTaskDelay(100)


//Functions prototype

void printmsg(char *msg);
void prvSetupGPIO(void);
static void prvSetupUart(void);
static void prvSetupHardware(void);
uint8_t getCommandCode(uint8_t *buffer);

//Tasks prototypes
void vTask1_menu_display(void *params);
void vTask2_cmd_handling(void *params);
void vTask3_cmd_processing(void *params);
void vTask4_uart_write(void *params);


//Global spaces fo some variables
char usr_msg[250]={0}; //this is also used as buffer
uint8_t command_buffer[20];
uint8_t command_len = 0;

//Task Handle
TaskHandle_t xTaskHandle1=NULL;
TaskHandle_t xTaskHandle2=NULL;
TaskHandle_t xTaskHandle3=NULL;
TaskHandle_t xTaskHandle4=NULL;

//Queue Command Handle
QueueHandle_t command_queue=NULL;
QueueHandle_t uart_write_queue=NULL;

//command structure
typedef struct APP_CMD // typedef keyword is used to give an alias name or duplicate
{
	uint8_t COMMAND_NUM;
	uint8_t COMMAND_ARGS[10];
}APP_CMD_t;

//This is the Menu
char menu[]={"\ 
		\r\n LED_ON     			----> 1 \
		\r\n LED_OFF    			----> 2 \
		\r\n LED_TOGGLE 			----> 3 \
		\r\n LED_TOGGLE_OFF 		----> 4 \
		\r\n LED_READ_STATUS		----> 5 \
		\r\n LED_PRINT_DATE_TIME    ----> 6 \
		\r\n EXIT_APP				----> 7 \
		\r\n Type Your Opinion Here : " };

#define LED_ON_COMMAND 			 			1
#define LED_OFF_COMMAND 					2
#define LED_TOGGLE_COMMAND					3
#define LED_TOGGLE_STOP_COMMAND				4
#define LED_READ_STATUS_COMMAND				5
#define RTC_READ_DATE_TIME_COMMAND			6

//Helping Functions Prototypes
void make_led_on(void);
void make_led_off(void);
void led_toggle(void);
void led_toggle_start(void);
void led_toggle_stop(void);
void read_led_status(char *task_msg);
void read_rtc_info(char *task_msg);
void print_error_message(char *task_msg);

void SEGGER_SYSVIEW_Conf();
void SEGGER_SYSVIEW_Start();



int main(void)
{

	DWT->CTRL |= (1<<0); //ENABLE CYCCNT IN DWT_CTRL.

	RCC_DeInit();

	SystemCoreClockUpdate();

	prvSetupHardware();

	sprintf(usr_msg,"\r\nThis is QUEUE processing demo\r\n");
	printmsg(usr_msg);

	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	//Lets create command queue
	command_queue = xQueueCreate(10,sizeof(APP_CMD_t*)); //we have used that structure's address to rwduce the heap memory

	//Lets create uart queue
	uart_write_queue = xQueueCreate(10,sizeof(char*)); //


	if((command_queue != NULL) && (uart_write_queue != NULL))
	{
	//Lets create led task
	xTaskCreate(vTask1_menu_display,"Task 1 - Menu",500,NULL,1,&xTaskHandle1);

	xTaskCreate(vTask2_cmd_handling,"Task 2 - Command Handling",500,NULL,1,&xTaskHandle2);

	xTaskCreate(vTask3_cmd_processing,"Task 3 - Command Processing ",500,NULL,1,&xTaskHandle3);

	xTaskCreate(vTask4_uart_write,"Task 4 - UART write",500,NULL,1,&xTaskHandle4);

	//Lets start tghe scheduler
	vTaskStartScheduler();
	}
	else
	{
		sprintf(usr_msg,"No space in the heap Memory \r\n");
		printmsg(usr_msg);
	}
	for(;;);
}


// TASKS HANDLERS IMPLEMENTATIONS
void vTask1_menu_display(void *params)
{
	char *pData = menu;

	while(1)
	{
		xQueueSend(uart_write_queue,&pData,portMAX_DELAY);

		//lets wait jere unitl someone notify
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
	}
}

void vTask2_cmd_handling(void *params)
{
	uint8_t command_code = 0;
	APP_CMD_t *new_cmd;
	while(1)
	{
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		//1.send command to queue
		command_code = getCommandCode(command_buffer);
		new_cmd = (APP_CMD_t*)pvPortMalloc(sizeof(APP_CMD_t));
		new_cmd->COMMAND_NUM = command_code;
		getArgyuments(new_cmd->COMMAND_ARGS);

		//send command to command Queue
		xQueueSend(command_queue,&new_cmd,portMAX_DELAY);


	}
}

void vTask3_cmd_processing(void *params)
{
	APP_CMD_t *new_cmd;
	char task_msg[50];

	while(1)
	{
		xQueueReceive(command_queue,(void*)new_cmd,portMAX_DELAY);

		if(new_cmd->COMMAND_NUM == LED_ON_COMMAND)
		{
			make_led_on();
		}
		else if(new_cmd->COMMAND_NUM == LED_OFF_COMMAND)
		{
			make_led_off();
		}
		else if(new_cmd->COMMAND_NUM == LED_TOGGLE_COMMAND)
		{
			led_toggle_start();
		}
		else if(new_cmd->COMMAND_NUM == LED_TOGGLE_STOP_COMMAND)
		{
			led_toggle_stop();
		}
		else if(new_cmd->COMMAND_NUM == LED_READ_STATUS_COMMAND)
		{
			read_led_status(task_msg);
		}
		else if(new_cmd->COMMAND_NUM == RTC_READ_DATE_TIME_COMMAND)
		{
			read_rtc_info(task_msg);
		}
		else
		{
			print_error_message(task_msg);
		}


	}
}

void vTask4_uart_write(void *params)
{
	char *pData = NULL;
	while(1)
	{

		xQueueReceive(uart_write_queue,&pData,portMAX_DELAY);
		printmsg(pData);
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

		//lets enable the UART byte reception interrupt in the micro controller
		USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);

		//lets set the priority interrupt
		NVIC_SetPriority(USART2_IRQn,5);

		//now, enable the UART2 IRQ in the NVIC
		NVIC_EnableIRQ(USART2_IRQn);

		//5.Enable the UART peripheral
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

void vApplicationIdleHook()
{
	//send the CPU in sleep mode
	__WFI();
}


void USART2_IRQHandler(void)
{
	uint8_t data_byte;
	BaseType_t xHigherPriorityTasWoken = pdFalse;
	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE))
	{
		//A DATA byte is received from the user
		data_byte = USART_ReceiveData(USART2);

		command_buffer[command_len++] = data_byte & 0xFF ;

		if(data_byte == '\r')
		{
			//user finished entering data

			//Reset command_len variable
			command_len = 0;

			//lets notify the command handling task

			xTaskNotifyFromISR(xTaskHandle2,0,eNoAction,&xHigherPriorityTasWoken);


			xTaskNotifyFromISR(xTaskHandle1,0,eNoAction,&xHigherPriorityTasWoken);
		}

	}

	//if the above freeRTOS API wakeup any higher priority task then yield the processor to the
	//higher priority task wich uis just woken up

	if(xHigherPriorityTasWoken)
	{
		taskYIELD();
	}

}

uint8_t getCommandCode(uint8_t *buffer)
{
	return buffer[0]-48; //convert ASCI value to number
}

void getArguments(uint8_t *buffer)
{



}
void make_led_on(void)
{
	GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_SET);
}


void make_led_off(void)
{
	GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_RESET);
}



void led_toggle(TimerHandle_t xTimer)
{
	GPIO_ToggleBits(GPIOD,GPIO_Pin_12);
}

void led_toggle_start(uint32_t duration)
{

	
}


void led_toggle_stop(void)
{
	 
}


void read_led_status(char *task_msg)
{
	sprintf(task_msg , "\r\nLED status is : %d\r\n", GPIO_ReadOutputDataBit(GPIOD,GPIO_Pin_12));
	xQueueSend(uart_write_queue,&task_msg,portMAX_DELAY);
}


void read_rtc_info(char *task_msg)
{
	

}


void print_error_message(char *task_msg)
{
	sprintf( task_msg,"\r\nInvalid command received\r\n");
	xQueueSend(uart_write_queue,&task_msg,portMAX_DELAY);
}
