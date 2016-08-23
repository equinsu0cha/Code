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
#include <stdio.h>
//====================================================================
// GLOBAL CONSTANTS
//====================================================================

//====================================================================
// GLOBAL VARIABLES
//====================================================================
char lcdstring[16];
uint8_t RxBuffer[25];
int count;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_GPIO(void);
void init_DMAUSART1(void);
void init_USART2(void);
void init_TIM14(void);
void init_NVIC(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_LCD();								// Initialise lcd
	lcd_putstring("USART->USART");		// Display string on line 1
	init_GPIO();
	init_DMAUSART1();
	init_USART2();
	while((GPIO_ReadInputData(GPIOA))&GPIO_IDR_0){}
	lcd_command(CLEAR);
	lcd_putstring("Pass");
	init_TIM14();
	init_NVIC();
	for(;;){
		lcd_command(CURSOR_HOME);
		sprintf(lcdstring,"time:%d",count);
		lcd_putstring(lcdstring);
		lcd_command(LINE_TWO);
		int temp = RxBuffer[0];
		sprintf(lcdstring,"USART:%d",temp);
		lcd_putstring(lcdstring);
	}
}

void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBENR_GPIOBEN),ENABLE);
	GPIO_InitTypeDef GPIOA_struct,GPIOA_USART_struct,GPIOB_struct,GPIOB_USART_struct;
	//GPIOB PB0-PB6 Outputs
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|
			GPIO_Pin_5|GPIO_Pin_6);
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOB,&GPIOB_struct);
	//GPIOB PB7 USART1 RX AF0
	GPIOB_USART_struct.GPIO_Pin=GPIO_Pin_7;
	GPIOB_USART_struct.GPIO_Mode=GPIO_Mode_AF;
	GPIOB_USART_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_USART_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_USART_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOB,&GPIOB_USART_struct);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_0);
	//GPIOA PA0-PA2 Inputs
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3);
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIOA_struct);
	//GPIOA PA2 USART2 TX AF1
	GPIOA_USART_struct.GPIO_Pin=GPIO_Pin_2;
	GPIOA_USART_struct.GPIO_Mode=GPIO_Mode_AF;
	GPIOA_USART_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_USART_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOA_USART_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOA_USART_struct);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_1);
}
void init_DMAUSART1(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	//USART1 RX Config
	USART_InitTypeDef USART1RX_struct;
	USART1RX_struct.USART_BaudRate=100000;
	USART1RX_struct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART1RX_struct.USART_Mode=USART_Mode_Rx;
	USART1RX_struct.USART_Parity=USART_Parity_Even;
	USART1RX_struct.USART_StopBits=USART_StopBits_2;
	USART1RX_struct.USART_WordLength=USART_WordLength_9b;
	USART_Init(USART1,&USART1RX_struct);
	USART_InvPinCmd(USART1,USART_InvPin_Rx,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	USART_MSBFirstCmd(USART1,ENABLE);
	//DMA1
	DMA1_Channel3->CNDTR=sizeof(RxBuffer);			//25 bytes to recieve
	DMA1_Channel3->CPAR=(uint32_t) &(USART1->RDR);
	DMA1_Channel3->CMAR=(uint32_t) &(RxBuffer[0]);
	DMA1_Channel3->CCR=(DMA_M2M_Disable|DMA_Priority_VeryHigh|DMA_MemoryDataSize_Byte|DMA_PeripheralDataSize_Byte
			|DMA_MemoryInc_Enable|DMA_PeripheralInc_Disable|DMA_Mode_Circular|DMA_DIR_PeripheralSRC);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	DMA_Cmd(DMA1_Channel3,ENABLE);
	USART_Cmd(USART1,ENABLE);
}
void init_USART2(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	USART_InitTypeDef USART2TX_struct;
	USART2TX_struct.USART_BaudRate=100000;
	USART2TX_struct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART2TX_struct.USART_Mode=USART_Mode_Tx;
	USART2TX_struct.USART_Parity=USART_Parity_Even;
	USART2TX_struct.USART_StopBits=USART_StopBits_2;
	USART2TX_struct.USART_WordLength=USART_WordLength_9b;
	USART_Init(USART2,&USART2TX_struct);
	USART_InvPinCmd(USART2,USART_InvPin_Tx,ENABLE);
	//USART_DataInvCmd(USART2,ENABLE);
	USART_MSBFirstCmd(USART2,ENABLE);
	USART_Cmd(USART2,ENABLE);
}
void init_TIM14(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	TIM14->PSC=10;
	TIM14->ARR=61091;
	TIM14->DIER|=TIM_DIER_UIE;
	TIM_Cmd(TIM14,ENABLE);
}
void init_NVIC(void){
	NVIC_InitTypeDef NVIC_TIM14_init;
	NVIC_TIM14_init.NVIC_IRQChannel=TIM14_IRQn;
	NVIC_TIM14_init.NVIC_IRQChannelPriority=0;
	NVIC_TIM14_init.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM14_init);
}
void TIM14_IRQHandler(void){
	count++;
	USART2->TDR=240;
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TC)){}
	USART2->TDR=191;
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TC)){}
	USART2->TDR=197;
	//SART_SendData(USART2,0x240);
	TIM_ClearITPendingBit(TIM14,TIM_IT_Update);

}
//********************************************************************
// END OF PROGRAM
//********************************************************************
