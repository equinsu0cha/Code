//********************************************************************
//*                    USART BASIC		                             *
//*==================================================================*
//USART testing for RX, interrupts from RXNE flag
//====================================================================
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
#include <stm32f0xx_usart.h>
#include <stm32f0xx_gpio.h>
#include <stdio.h>
//====================================================================
// GLOBAL CONSTANTS
//====================================================================

//====================================================================
// GLOBAL VARIABLES
//====================================================================
char lcdstring[16];
uint16_t temp = 0;
char lcdstring[16];
//================GPIO_ODR_0====================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_ports(void);
void Usart_config(void);
void init_NVIC(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_ports();
	init_LCD();								// Initialise lcd
	init_NVIC();
	Usart_config();
	lcd_putstring("USART TEST2");		// Display string on line 1
	for(;;){
		lcd_command(LINE_TWO);
		//temp = (USART1->RDR);
		sprintf(lcdstring,"GPIOB: %d",temp);
		lcd_putstring(lcdstring);
	}
}										// End of main
void Usart_config(void){
	USART_InitTypeDef Usart1_init;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,1);
	Usart1_init.USART_BaudRate=100000;
	Usart1_init.USART_WordLength=USART_WordLength_8b;
	Usart1_init.USART_StopBits=USART_StopBits_2;
	Usart1_init.USART_Parity=USART_Parity_Even;
	Usart1_init.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	Usart1_init.USART_Mode=USART_Mode_Rx;
	USART_Init(USART1,&Usart1_init);
	USART1->CR1|=USART_CR1_PCE;
	USART1->CR2|=USART_CR2_MSBFIRST|USART_CR2_DATAINV;
	USART_ITConfig(USART1,USART_IT_RXNE,1);
	USART_Cmd(USART1,1);
}
void init_ports(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,1);
	GPIO_InitTypeDef GPIOB_init;
	GPIOB_init.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
	GPIOB_init.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_init.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIOB_init);
	GPIOB->MODER|=(GPIO_MODER_MODER6_1|GPIO_MODER_MODER7_1);
	GPIOB->PUPDR|=(GPIO_PUPDR_PUPDR6_0|GPIO_PUPDR_PUPDR7_0);
	GPIOB->OSPEEDR|=(GPIO_OSPEEDR_OSPEEDR6|GPIO_OSPEEDER_OSPEEDR7);
	GPIO_PinAFConfig(GPIOB,(GPIO_PinSource6|GPIO_PinSource7),GPIO_AF_0);
}

void init_NVIC(void){
	NVIC_InitTypeDef NVIC_init_struct;
	NVIC_init_struct.NVIC_IRQChannel=USART1_IRQn;
	NVIC_init_struct.NVIC_IRQChannelPriority=0;
	NVIC_init_struct.NVIC_IRQChannelCmd = 1;
	NVIC_Init(&NVIC_init_struct);
}

void USART1_IRQHandler(void){
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=0){
		lcd_putstring("USART IN");
		lcd_command(LINE_TWO);
		temp = (USART1->RDR);
		sprintf(lcdstring,"%d",temp);
		lcd_putstring(lcdstring);
		USART_ClearFlag(USART1,USART_FLAG_RXNE);
	}
}
//********************************************************************
// END OF PROGRAM
//********************************************************************
