//********************************************************************
//*                    S.BUS USART RX 		                         *
//*                    LCD test                                      *
//*==================================================================*
// Rx USART from 16 CH encoder. 6 CH pwm from RX module, encoded to
// S.BUS w/ PWM2SBUS encoder. S.BUS is 8 25 bytes of data sent at
// 100 000 baud (bits/s). 1st byte 0xf0 start byte.
// 25 bytes of data structured:
// MSB first
// inverted
// 1 start bit
// 8 bits of data
// even parity bit
// 2 stop bits
// buffers 25 bytes into DMA from USART then triggers interrupts,
// subroutine checks for start byte at DMA[0] then bitshits channel
// data into variables.
// Channel data is little endian first
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
uint16_t temp = 0,CH[16],FLAGS;
char lcdstring[16];
//================GPIO_ODR_0====================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_GPIO(void);
void DMAUSART1_config(void);
void init_TIM2(void);
void init_NVIC(void);
void USART1_IRQHandler(void);
uint16_t endian(uint16_t little);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_LCD();
	init_GPIO();
	lcd_putstring("USART SERVO TEST");
	while((GPIO_ReadInputData(GPIOA))&(GPIO_IDR_0)){}
	//Config for USART/DMA
	DMAUSART1_config();
	init_TIM2();
	//NVIC Interrupts
	init_NVIC();
	lcd_command(CLEAR);
	lcd_putstring("USART");		// Display string on line 1
	int channel4,channel5;
	for(;;){
		lcd_command(CURSOR_HOME);
		channel4 = endian(CH[4]);
		channel5 = endian(CH[5]);
		sprintf(lcdstring,"ch4:%d                ",channel4);
		lcd_putstring(lcdstring);
		lcd_command(LINE_TWO);
		sprintf(lcdstring,"ch5:%d                ",channel5);
		lcd_putstring(lcdstring);
		TIM_SetCompare3(TIM2,((1400+channel4)/1400.0)*(1/3.03)*48485);
		TIM_SetCompare4(TIM2,((1400+channel5)/1400.0)*(1/3.03)*48485);
	}
}										// End of main

void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB),ENABLE);
	//GPIOA PA0-PA3 Inputs
	GPIO_InitTypeDef GPIOA_struct;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIOA_struct);
	//GPIOB PB0-PB6 Outputs
	GPIO_InitTypeDef GPIOB_struct;
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6);
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIOB_struct);
	//GPIOB PB7 AF0 USART1 RX
	GPIO_InitTypeDef GPIOB_USART_struct;
	GPIOB_USART_struct.GPIO_Pin=(GPIO_Pin_7);
	GPIOB_USART_struct.GPIO_Mode=GPIO_Mode_AF;
	GPIOB_USART_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_USART_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_USART_struct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIOB_USART_struct);
	GPIO_PinAFConfig(GPIOB,(GPIO_PinSource7),GPIO_AF_0);
	//GPIOB PB10 & PB11 TIM2 CH3 & CH4 OUTPUT
	GPIO_InitTypeDef GPIOB_TIM2_struct;
	GPIOB_TIM2_struct.GPIO_Pin=(GPIO_Pin_10|GPIO_Pin_11);
	GPIOB_TIM2_struct.GPIO_Mode=GPIO_Mode_AF;
	GPIOB_TIM2_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_TIM2_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIOB_TIM2_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIOB_TIM2_struct);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_2);
}
void init_TIM2(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_DeInit(TIM2);
	TIM_TimeBaseInitTypeDef TIM2_basestruct;
	TIM2_basestruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM2_basestruct.TIM_ClockDivision= 0;
	TIM2_basestruct.TIM_Period=48485;
	TIM2_basestruct.TIM_Prescaler=3;
	TIM2_basestruct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM2_basestruct);
	//OC PWM MODE
	TIM_OCInitTypeDef TIM2_OCstruct={0,};
	TIM2_OCstruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM2_OCstruct.TIM_Pulse=(uint32_t) 24242;
	TIM2_OCstruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM2_OCstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OC3Init(TIM2,&TIM2_OCstruct);
	TIM_OC4Init(TIM2,&TIM2_OCstruct);
	//Start TIMER2
	TIM_Cmd(TIM2,ENABLE);
}
void DMAUSART1_config(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,1);
	//USART Config
	USART_DeInit(USART1);
	USART_InitTypeDef USART1_struct;
	USART1_struct.USART_BaudRate=100000;
	USART1_struct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART1_struct.USART_Mode=USART_Mode_Rx;
	USART1_struct.USART_Parity=USART_Parity_Even;
	USART1_struct.USART_StopBits=USART_StopBits_2;
	USART1_struct.USART_WordLength=USART_WordLength_9b;
	USART_Init(USART1,&USART1_struct);
	USART_InvPinCmd(USART1,USART_InvPin_Rx,ENABLE);
	USART_MSBFirstCmd(USART1,ENABLE);
	DMA_DeInit(DMA1_Channel3);
	DMA1_Channel3->CNDTR=sizeof(RxBuffer);			//25 bytes to recieve
	DMA1_Channel3->CPAR=(uint32_t) &(USART1->RDR);
	DMA1_Channel3->CMAR=(uint32_t) &(RxBuffer[0]);
	DMA1_Channel3->CCR=(DMA_M2M_Disable|DMA_Priority_VeryHigh|DMA_MemoryDataSize_Byte|DMA_PeripheralDataSize_Byte
			|DMA_MemoryInc_Enable|DMA_PeripheralInc_Disable|DMA_Mode_Normal|DMA_DIR_PeripheralSRC);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	DMA_Cmd(DMA1_Channel3,ENABLE);
	USART_Cmd(USART1,ENABLE);
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
	DMA_Cmd(DMA1_Channel3,ENABLE);
	USART_Cmd(USART1,ENABLE);
}
void init_NVIC(void){
	NVIC_InitTypeDef NVIC_init_struct;
	NVIC_init_struct.NVIC_IRQChannel=USART1_IRQn;
	NVIC_init_struct.NVIC_IRQChannelPriority=0;
	NVIC_init_struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_init_struct);
	NVIC_init_struct.NVIC_IRQChannel=DMA1_Channel2_3_IRQn;
	NVIC_Init(&NVIC_init_struct);
}
void USART1_IRQHandler(void){
	DMA_Cmd(DMA1_Channel3,DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel3,25);
	DMA_Cmd(DMA1_Channel3,ENABLE);
	USART_ClearITPendingBit(USART1,USART_IT_IDLE);
	if(RxBuffer[0]!=240){
		GPIO_Write(GPIOB,0xff);
	}
	else{
		GPIO_Write(GPIOB,0x0);
		CH[1]=((RxBuffer[1]&0b11111111)<<3)+((RxBuffer[2]&0b11100000)>>5);
		CH[2]=((RxBuffer[2]&0b00011111)<<6)+((RxBuffer[3]&0b11111100)>>2);
		CH[3]=((RxBuffer[3]&0b00000011)<<9)+((RxBuffer[4]&0b11111111)<<1)+((RxBuffer[5]&0b10000000)>>7);
		CH[4]=((RxBuffer[5]&0b01111111)<<4)+((RxBuffer[6]&0b11110000)>>4);
		CH[5]=((RxBuffer[6]&0b00001111)<<7)+((RxBuffer[7]&0b11111110)>>1);
		CH[6]=((RxBuffer[7]&0b00000001)<<10)+((RxBuffer[8]&0b11111111)<<2)+((RxBuffer[9]&0b11000000)>>6);
		CH[7]=((RxBuffer[9]&0b00111111)<<5)+((RxBuffer[10]&0b11111000)>>3);
		CH[8]=((RxBuffer[10]&0b00000111)<<8)+((RxBuffer[11]&0b11111111)>>0);
		CH[9]=((RxBuffer[12]&0b11111111)<<3)+((RxBuffer[13]&0b11100000)>>5);
		CH[10]=((RxBuffer[13]&0b00011111)<<6)+((RxBuffer[14]&0b11111100)>>2);
		CH[11]=((RxBuffer[14]&0b00000011)<<9)+((RxBuffer[15]&0b11111111)<<1)+((RxBuffer[16]&0b10000000)>>7);
		CH[12]=((RxBuffer[16]&0b01111111)<<4)+((RxBuffer[17]&0b11110000)>>4);
		CH[13]=((RxBuffer[17]&0b00001111)<<7)+((RxBuffer[18]&0b11111110)>>1);
		CH[14]=((RxBuffer[18]&0b00000001)<<10)+((RxBuffer[19]&0b11111111)<<2)+((RxBuffer[20]&0b11000000)>>6);
		CH[15]=((RxBuffer[20]&0b00111111)<<5)+((RxBuffer[21]&0b11111000)>>3);
		CH[16]=((RxBuffer[21]&0b00000111)<<8)+((RxBuffer[22]&0b11111111)>>0);
		FLAGS=((RxBuffer[23]&0b11111111)<<3)+((RxBuffer[24]&0b11100000)>>5);
	}
}

uint16_t endian(uint16_t little){
	int temp=0;
	temp +=(little&0b10000000000)>>10;
	temp +=(little&0b01000000000)>>8;
	temp +=(little&0b00100000000)>>6;
	temp +=(little&0b00010000000)>>4;
	temp +=(little&0b00001000000)>>2;
	temp +=(little&0b00000100000)>>0;
	temp +=(little&0b00000010000)<<2;
	temp +=(little&0b00000001000)<<4;
	temp +=(little&0b00000000100)<<6;
	temp +=(little&0b00000000010)<<8;
	temp +=(little&0b00000000001)<<10;
	return temp;
}

//********************************************************************
// END OF PROGRAM
//********************************************************************
