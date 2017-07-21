//********************************************************************
//*							Servo Step test
//*==================================================================*
// Steps Servo 1 (alpha) from TIM2Ch3
// Steps Servo 2 (lambda) from TIM2CH4
// ADC1CH5 and ADC1CH6 are input for strain guage and potentiometer respectively
// USART1 sends TIM2CH3 OC compare value and ADC1CH5 input
// TIM14 schedules steps in servo range and USART send packets
//
// Servo1 = servo_alph
// Servo1 range = [11800:39000]
// Servo1 0th offset = 25500
//
// Servo2 = servo_lam
// Servo2 range = [:]
// Servo2 0th offset =
//====================================================================
#include <stdio.h>
#include <stdlib.h>
#include "stdio.h"
#include "math.h"
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_adc.h"
//Variables
char lcdstring[16],usart_char[16];
//uninitialized variables
int temp,sysclock,TIM2CH3=10000;
//initialized variables
int tmr=0;
uint16_t ADC_Buffer[1];
//Function defs
void init_GPIO(void);
void init_ADC(void);
void init_DMA(void);
void init_TIM2(void);
void init_TIM14(void);
void init_USART1(void);
void set_servo_alph(float);
void send_packet(const char *str);
//Main
void main(void){
	init_LCD();
	init_GPIO();
	//Get operating CLK freq
	RCC_ClocksTypeDef CLK;
	RCC_GetClocksFreq(&CLK);
	sysclock= (CLK.SYSCLK_Frequency)/((float)(pow(10,6)));
	//Sys clocks in MHz
	sprintf(lcdstring,"CLK:%d MHz",sysclock);
	lcd_putstring(lcdstring);
	lcd_command(LINE_TWO);
	lcd_putstring("ADCServo 1");
	GPIO_Write(GPIOB,0x00);
	//Waits for SW0
	while(GPIO_ReadInputData(GPIOA)&GPIO_IDR_0){}
	//Confirm Start
	lcd_command(CLEAR);
	GPIO_Write(GPIOB,0xff);
	//Init secondary peripherals
	init_TIM2();
	init_ADC();
	init_DMA();
	init_USART1();
	init_TIM14();
	//Start ADC Conversion
	ADC_StartOfConversion(ADC1);
	lcd_command(CLEAR);
	//For loop
	for(;;){
		lcd_command(CURSOR_HOME);
		sprintf(lcdstring,"%d",temp);
		lcd_putstring(lcdstring);
	}
}
void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBENR_GPIOBEN),ENABLE);
	GPIO_InitTypeDef GPIOA_struct,GPIOAADC_struct,GPIOATIM3_struct,GPIOAUSART_struct,GPIOB_struct,GPIOBTIM2_struct;
	//PA0-PA1 inputs
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2);
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOA_struct);
	//PA5 ADC Input
	GPIOAADC_struct.GPIO_Mode=GPIO_Mode_AN;
	GPIOAADC_struct.GPIO_OType=GPIO_OType_PP;
	GPIOAADC_struct.GPIO_Pin=GPIO_Pin_5;
	GPIOAADC_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOAADC_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOAADC_struct);
	//AF1 for TIM3CH2 output
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_1);
	//PA9-PA10 USART1 RX/TX
	GPIOAUSART_struct.GPIO_Mode=GPIO_Mode_AF;
	GPIOAUSART_struct.GPIO_OType=GPIO_OType_PP;
	GPIOAUSART_struct.GPIO_Pin=(GPIO_Pin_9|GPIO_Pin_10);
	GPIOAUSART_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOAUSART_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOAUSART_struct);
	//AF1 for PA9-PA10 USART RX/TX pins
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);
	//PB0-PB7 Outputs
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOB,&GPIOB_struct);
	GPIO_Write(GPIOB,0x00);
	//PB10 TIM2CH3 PWM output
	GPIOBTIM2_struct.GPIO_Mode=GPIO_Mode_AF;
	GPIOBTIM2_struct.GPIO_OType=GPIO_OType_PP;
	GPIOBTIM2_struct.GPIO_Pin=GPIO_Pin_10;
	GPIOBTIM2_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOBTIM2_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOB,&GPIOBTIM2_struct);
	//PB10 AF2 for TIM2CH3 output
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_2);
}
void init_ADC(void){
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN,ENABLE);
	ADC_InitTypeDef ADC1_struct;
	ADC1_struct.ADC_ContinuousConvMode=DISABLE;
	ADC1_struct.ADC_DataAlign=ADC_DataAlign_Right;
	ADC1_struct.ADC_ExternalTrigConv=ADC_ExternalTrigConvEdge_None;
	ADC1_struct.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
	ADC1_struct.ADC_Resolution=ADC_Resolution_12b;
	ADC1_struct.ADC_ScanDirection=ADC_ScanDirection_Upward;
	ADC_Init(ADC1,&ADC1_struct);
	NVIC_InitTypeDef ADC1_NVIC;
	ADC1_NVIC.NVIC_IRQChannel=ADC1_COMP_IRQn;
	ADC1_NVIC.NVIC_IRQChannelPriority=0;
	ADC1_NVIC.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&ADC1_NVIC);
	ADC_ChannelConfig(ADC1,ADC_Channel_5,ADC_SampleTime_55_5Cycles);
	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
	ADC_DMARequestModeConfig(ADC1,ADC_DMAMode_Circular);
	ADC_DMACmd(ADC1,ENABLE);
	// Enable ADC
	ADC_Cmd(ADC1,ENABLE);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_ADRDY)){}
}
void ADC1_COMP_IRQHandler(void){
	ADC_StartOfConversion(ADC1);
	ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
}
void init_DMA(void){
	RCC_AHBPeriphClockCmd(RCC_AHBENR_DMA1EN,ENABLE);
	DMA_InitTypeDef DMA_struct;
	DMA_struct.DMA_BufferSize=0x1;
	DMA_struct.DMA_DIR=DMA_DIR_PeripheralSRC;
	DMA_struct.DMA_M2M=DMA_M2M_Disable;
	DMA_struct.DMA_MemoryBaseAddr=(uint32_t) &(ADC_Buffer[0]);
	DMA_struct.DMA_MemoryDataSize=DMA_MemoryDataSize_Word;
	DMA_struct.DMA_MemoryInc=DMA_MemoryInc_Enable;
	DMA_struct.DMA_Mode=DMA_Mode_Circular;
	DMA_struct.DMA_PeripheralBaseAddr=(uint32_t) &(ADC1->DR);
	DMA_struct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Word;
	DMA_struct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_struct.DMA_Priority=DMA_Priority_High;
	DMA_Init(DMA1_Channel1,&DMA_struct);
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
	NVIC_InitTypeDef DMA_NVIC;
	DMA_NVIC.NVIC_IRQChannel=DMA1_Channel1_IRQn;
	DMA_NVIC.NVIC_IRQChannelPriority=0;
	DMA_NVIC.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&DMA_NVIC);
	DMA_Cmd(DMA1_Channel1,ENABLE);
}
void DMA1_Channel1_IRQHandler(void){
	temp = ADC_Buffer[0];
	DMA_ClearITPendingBit(DMA1_IT_TC1);

}
void init_TIM2(void){
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2EN,ENABLE);
	TIM_TimeBaseInitTypeDef TIM2_struct;
	TIM2_struct.TIM_ClockDivision=0;
	TIM2_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM2_struct.TIM_Period=48485;
	TIM2_struct.TIM_Prescaler=2;
	TIM2_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM2_struct);
	TIM_OC3FastConfig(TIM2,TIM_OCFast_Enable);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	NVIC_InitTypeDef TIM2_NVIC;
	TIM2_NVIC.NVIC_IRQChannel=TIM2_IRQn;
	TIM2_NVIC.NVIC_IRQChannelPriority=0;
	TIM2_NVIC.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&TIM2_NVIC);
	TIM_OCInitTypeDef TIM2_OCstruct={0,};
	TIM2_OCstruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM2_OCstruct.TIM_Pulse=(int)(TIM2CH3);
	TIM2_OCstruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM2_OCstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	//Init OC3 output at 50% Duty Cycle for testing
	TIM_OC3Init(TIM2,&TIM2_OCstruct);
	TIM_Cmd(TIM2,ENABLE);
}
void TIM2_IRQHandler(void){
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
}
void init_TIM14(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	TIM_TimeBaseInitTypeDef TIM14_struct;
	TIM14_struct.TIM_ClockDivision=0x0;
	TIM14_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM14_struct.TIM_Period=60000;
	TIM14_struct.TIM_Prescaler=11;
	TIM14_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM14,&TIM14_struct);
	TIM_ITConfig(TIM14,TIM_IT_Update,ENABLE);
	NVIC_InitTypeDef TIM14_NVIC;
	TIM14_NVIC.NVIC_IRQChannel=TIM14_IRQn;
	TIM14_NVIC.NVIC_IRQChannelPriority=0;
	TIM14_NVIC.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&TIM14_NVIC);
	TIM_Cmd(TIM14,ENABLE);
}
void TIM14_IRQHandler(void){
	if(TIM2CH3<=42000){
		TIM2CH3+=10;
	}
	else{
		TIM2CH3=10000;
	}
	TIM_SetCompare3(TIM2,TIM2CH3);
	sprintf(usart_char,"%d\n",(int)(240));
	send_packet(usart_char);
	//sprintf(usart_char,"%d\n",tmr);
	//send_packet(usart_char);
	sprintf(usart_char,"%d\n",(int) (ADC_Buffer[0]));
	send_packet(usart_char);
	//sprintf(usart_char,"%d\n",(int) (ADC_Buffer[1]));
	//send_packet(usart_char);
	sprintf(usart_char,"%d\n",(int) (TIM2->CCR3));
	send_packet(usart_char);
	TIM_ClearITPendingBit(TIM14,TIM_IT_Update);
}
void init_USART1(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	USART_InitTypeDef USART1_struct;
	USART1_struct.USART_BaudRate=345600;
	USART1_struct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART1_struct.USART_Mode=(USART_Mode_Rx|USART_Mode_Tx);
	USART1_struct.USART_Parity=USART_Parity_No;
	USART1_struct.USART_StopBits=USART_StopBits_2;
	USART1_struct.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART1,&USART1_struct);
	USART_Cmd(USART1,ENABLE);
}
void set_servo_alph(float degree){
	if(degree<0){
		TIM_SetCompare3(TIM2,(int) (10600*(degree+216.5)/90));
	}
	if(degree>=0){
		TIM_SetCompare3(TIM2,(int) (13000*(degree+176.53)/90));
	}
}
void send_packet(const char *str){
	while(*str){
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0);
		USART_SendData(USART1,*str++);
	}
}
