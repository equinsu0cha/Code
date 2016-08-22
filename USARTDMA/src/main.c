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
//====================================================================
// GLOBAL VARIABLES
//====================================================================
uint8_t RxBuffer[24];
uint16_t temp = 0,ch1,ch2,ch3,ch4,ch5,ch6;
int y=0;
char lcdstring[16];
//================GPIO_ODR_0====================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_ports(void);
void DMAUSART_config(void);
void init_NVIC(void);
void init_EXTI(void);
void EXTI0_1_IRQHandler(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_LCD();
	init_ports();
	lcd_putstring("Press SW1");
	while((GPIOA->IDR)&(GPIO_IDR_1)); //wait for DMA config
	//Config for USART/DMA
	DMAUSART_config();
	//NVIC Interrupts
	init_EXTI();
	init_NVIC();
	lcd_command(CLEAR);
	lcd_putstring("USART");		// Display string on line 1
	for(;;){
		//USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		for(int x = 0; x <25; x++){
			lcd_command(CURSOR_HOME);
			sprintf(lcdstring,"BRR:%d",(USART1->BRR));
			lcd_putstring(lcdstring);
			if (RxBuffer[x]==241){
				GPIO_Write(GPIOB,0xff);
			}
			lcd_command(LINE_TWO);
			sprintf(lcdstring,"Buffer[%d]: %d SPACE",x,RxBuffer[x]);
			lcd_putstring(lcdstring);
			for(int y = 0; y <6553; y++){
				for(int z = 0; z <2; z ++){}
			}
		}
	}
}										// End of main

void init_ports(void){
	//Init GPIOA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIOA_struct;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIOA_struct);
	//Init GPIOB
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIOB_struct;
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6); //enable PB0-5
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;			//as outputs
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;		//no pullup resistors
	GPIO_Init(GPIOB,&GPIOB_struct);
	//GPIOB Alternate Function init
	GPIO_InitTypeDef GPIOB_USART_struct;
	GPIOB_USART_struct.GPIO_Pin=(GPIO_Pin_7);
	GPIOB_USART_struct.GPIO_Mode=GPIO_Mode_AF;
	GPIOB_USART_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_USART_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_USART_struct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIOB_USART_struct);
	GPIO_PinAFConfig(GPIOB,(GPIO_PinSource7),GPIO_AF_0); //PB6-7 AF0
}

void DMAUSART_config(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,1);
	//USART Config
	USART_DeInit(USART1);
	USART1->CR1=(USART_CR1_OVER8|USART_CR1_PCE|USART_CR1_RE);//
	USART1->BRR=0x1E0;
	USART1->CR2=(USART_CR2_ABRMODE_1|USART_CR2_ABREN|USART_CR2_MSBFIRST|USART_CR2_DATAINV|USART_StopBits_2);//
	//DMA Config
	DMA1_Channel3->CNDTR=0x18;
	DMA1_Channel3->CPAR=(uint32_t) &(USART1->RDR);
	DMA1_Channel3->CMAR=(uint32_t) &(RxBuffer[0]);
	DMA1_Channel3->CCR=(DMA_M2M_Disable|DMA_Priority_VeryHigh|DMA_MemoryDataSize_Byte|DMA_PeripheralDataSize_Byte
			|DMA_MemoryInc_Enable|DMA_PeripheralInc_Disable|DMA_Mode_Circular|DMA_DIR_PeripheralSRC);
	USART1->CR3=(USART_CR3_DMAR);
	DMA_Cmd(DMA1_Channel3,ENABLE);
	USART1->CR1|=(USART_CR1_UE);
}
void init_EXTI(void){
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
	EXTI_InitTypeDef EXTI1_PA0;
	EXTI1_PA0.EXTI_Line=EXTI_Line0;
	EXTI1_PA0.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI1_PA0.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI1_PA0.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI1_PA0);
}
void init_NVIC(void){
	NVIC_InitTypeDef NVIC_init_struct;
	NVIC_init_struct.NVIC_IRQChannel=EXTI0_1_IRQn;
	NVIC_init_struct.NVIC_IRQChannelPriority=0;
	NVIC_init_struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_init_struct);
}
void EXTI0_1_IRQHandler(void){
	while((GPIOA->IDR&GPIO_IDR_0)){}
	if(y<=24){
		y++;
	}
	else{
		y=0;
	}
	EXTI_ClearITPendingBit(EXTI_Line0);

}
//********************************************************************
// END OF PROGRAM
//********************************************************************
