//********************************************************************
//*                    EEE3017W C template                           *
//*                    LCD test                                      *
//*==================================================================*
//* WRITTEN BY:    	                 		                         *
//* DATE CREATED:                                                    *
//* MODIFIED:                                                        *
//*==================================================================*
//* PROGRAMMED IN: Eclipse Luna Service Release 1 (4.4.1)            *
//* DEV. BOARD:    UCT STM32 Development Board                       *
//*==================================================================*
//* DESCRIPTION:                                                     *
//*                                                                  *
//********************************************************************
// INCLUDE FILES
//====================================================================
#include "stdio.h"
#include "math.h"
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
//====================================================================
// GLOBAL CONSTANTS
//====================================================================

//====================================================================
// GLOBAL VARIABLES
//====================================================================
char lcdstring[16];
uint32_t temp;
RCC_ClocksTypeDef sysclocks;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_GPIO(void);
void init_TIM14(void);
void init_USART1(void);

//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	RCC_GetClocksFreq(&sysclocks);
	temp = (sysclocks.SYSCLK_Frequency)/(pow(10,6));
	init_LCD();								// Initialise lcd
	lcd_putstring("USART -> USB");		// Display string on line 1
	lcd_command(LINE_TWO);					// Move cursor to line 2
	sprintf(lcdstring,"%d MHz, SW0",(int) temp);
	lcd_putstring(lcdstring);				// Display string on line 2
	init_GPIO();
	while(GPIO_ReadInputData(GPIOA)&GPIO_IDR_0){}
	lcd_command(CLEAR);
	init_USART1();
	GPIO_Write(GPIOB,0xff);
	for(;;){
		if(USART_GetFlagStatus(USART1,USART_FLAG_TXE)){
			GPIO_Write(GPIOB,0x00);
			USART_SendData(USART1,0b1010101010);
		}
		else{
			GPIO_Write(GPIOB,0xff);
		}
	}
}											// End of main

//********************************************************************
// END OF PROGRAM
//********************************************************************
void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBENR_GPIOBEN),ENABLE);
	GPIO_InitTypeDef GPIOA_struct,GPIOAUSART,GPIOB_struct;
	//PA0-PA3 Inputs
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOA,&GPIOA_struct);
	//PA9-PA10
	GPIOAUSART.GPIO_Mode=GPIO_Mode_AF;
	GPIOAUSART.GPIO_OType=GPIO_OType_PP;
	GPIOAUSART.GPIO_Pin=(GPIO_Pin_9|GPIO_Pin_10);
	GPIOAUSART.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOAUSART.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIOAUSART);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);
	//PB0-PB7
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOB,&GPIOB_struct);
}
void init_USART1(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	USART_InitTypeDef USART1_struct;
	USART1_struct.USART_BaudRate=115200;
	USART1_struct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART1_struct.USART_Mode=(USART_Mode_Rx|USART_Mode_Tx);
	USART1_struct.USART_Parity=USART_Parity_No;
	USART1_struct.USART_StopBits=USART_StopBits_2;
	USART1_struct.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART1,&USART1_struct);
	USART_Cmd(USART1,ENABLE);
}
