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
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
#include <stm32f0xx_usart.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_dma.h>
#include <stm32f0xx_syscfg.h>
#include <stdio.h>
//====================================================================
// GLOBAL CONSTANTS
//====================================================================
#define USART1_RDR_Address 0x40013824
#define USART1_TDR_Address 0x40013828
//====================================================================
// GLOBAL VARIABLES
//====================================================================
uint8_t RxBuffer [26] = {0};
char lcdstring[16];
uint16_t temp = 0;
char lcdstring[16];
//================GPIO_ODR_0====================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_clocks(void);
void init_ports(void);
void Usart_config(void);
void dma_config(void);
void init_NVIC(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_LCD();
	init_ports();
	//GPIOS Enabled HERE
	lcd_putstring("SW0 to Start");
	while((GPIOA->IDR)&(GPIO_IDR_0)); //wait for DMA config
	init_clocks();
	Usart_config();
	dma_config();
	init_NVIC();
	lcd_command(CLEAR);
	lcd_putstring("USART TEST");		// Display string on line 1

	for(;;){
		int temp=0x0;
		lcd_command(CURSOR_HOME);
		USART1->RDR;
		temp = DMA_GetCurrDataCounter(DMA1_Channel3);
		sprintf(lcdstring,"GPIOB: %d  ",temp);
		lcd_putstring(lcdstring);
	}
}										// End of main

void init_clocks(void){
	RCC_APB2PeriphClockCmd((RCC_APB2Periph_USART1|RCC_APB2Periph_SYSCFG),1);	//clock USART1
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,1);		//clock DMA
}

void init_ports(void){
	//Init GPIOA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,1);
	GPIO_InitTypeDef GPIOA_init;
	GPIOA_init.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	GPIOA_init.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_init.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIOA_init);
	//Init GPIOB
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,1);
	GPIO_InitTypeDef GPIOB_init;
	GPIOB_init.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5); //enable PB0-5
	GPIOB_init.GPIO_Mode=GPIO_Mode_OUT;			//as outputs
	GPIOB_init.GPIO_PuPd=GPIO_PuPd_NOPULL;		//no pullup resistors
	GPIO_Init(GPIOB,&GPIOB_init);
	//GPIOB Alternate Function init
	GPIOB->MODER|=(GPIO_MODER_MODER6_1|GPIO_MODER_MODER7_1); //set PB6-7 to AF for USART1 RX/TX
	GPIOB->PUPDR|=(GPIO_PUPDR_PUPDR6_0|GPIO_PUPDR_PUPDR7_0); //Set Pullup resistor
	GPIOB->OSPEEDR|=(GPIO_OSPEEDR_OSPEEDR6|GPIO_OSPEEDER_OSPEEDR7);	//highspeed outputs
	GPIO_PinAFConfig(GPIOB,(GPIO_PinSource6|GPIO_PinSource7),GPIO_AF_0); //PB6-7 AF0
}

void Usart_config(void){
	USART_InitTypeDef Usart1_init;
	Usart1_init.USART_BaudRate=100000;
	Usart1_init.USART_WordLength=USART_WordLength_8b;
	Usart1_init.USART_StopBits=USART_StopBits_2;
	Usart1_init.USART_Parity=USART_Parity_Even;
	Usart1_init.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	Usart1_init.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_DataInvCmd(USART1,1);
	USART_MSBFirstCmd(USART1,1);
	USART_DirectionModeCmd(USART1,USART_Mode_Rx,1);
	USART_DMACmd(USART1,USART_DMAReq_Rx,1);
	USART_Init(USART1,&Usart1_init);
	USART_Cmd(USART1,1);
	//USART_ITConfig(USART1,USART_IT_RXNE,1);
}
void dma_config(void){
	DMA_InitTypeDef DMA_struct;
	DMA_struct.DMA_BufferSize=sizeof(RxBuffer); //26 byte buffer
	DMA_struct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte; //8 Bit peripheral data size
	DMA_struct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	DMA_struct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_struct.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_struct.DMA_Mode=DMA_Mode_Normal;
	//DMA_struct.DMA_M2M=DMA_M2M_Disable;
	//
	DMA_struct.DMA_MemoryBaseAddr=(uint32_t)&RxBuffer[0];
	DMA_struct.DMA_DIR=DMA_DIR_PeripheralSRC;
	DMA_struct.DMA_Priority=DMA_Priority_High;
	DMA_struct.DMA_PeripheralBaseAddr=(uint32_t)&USART1->RDR;
	DMA_Init(DMA1_Channel3,&DMA_struct);
	SYSCFG_DMAChannelRemapConfig(SYSCFG_DMARemap_USART1Rx,1);
	DMA_Cmd(DMA1_Channel3,1);
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
