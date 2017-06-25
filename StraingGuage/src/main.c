//********************************************************************
//*							Strain Test
//*==================================================================*
// Measures strain gauge analogue signal for torque test on ADC1CH5 input
// from PA5.
// Includes servo position output on timer channel
//
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
int temp,sysclock,stp=-15;
//initialized variables
int tmr=0,TIM3OC2=2800;;
uint16_t ADC_Buffer[1];
//Function defs
void init_GPIO(void);
void init_EXTI(void);
void init_ADC(void);
void init_DMA(void);
void init_TIM2(void);
void init_TIM3(void);
void init_TIM14(void);
void init_USART1(void);
void set_servo(float);
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
	lcd_putstring("Inner Step");
	GPIO_Write(GPIOB,0x00);
	//Waits for SW0
	while(GPIO_ReadInputData(GPIOA)&GPIO_IDR_0){}
	//Confirm Start
	lcd_command(CLEAR);
	GPIO_Write(GPIOB,0xff);
	//Init secondary peripherals
	init_TIM2();
	init_TIM3();
	//Spin up BLDC Motor
	lcd_putstring("Spin UP");
	while(TIM3OC2<=3600){
		int GPIO_LED_temp;
		TIM3OC2++;
		TIM_SetCompare2(TIM3,TIM3OC2);
		GPIO_LED_temp = (int)(255.0*(TIM3OC2-2800)/800);
		GPIO_Write(GPIOB,GPIO_LED_temp);
		for(int x=0;x<=255;x++){
			for(int y=0;y<=255;y++){}
		}
	}
	//Init EXTI for TIM2CH3 output steps
	init_EXTI();
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
	//PA7 TIM3CH2 PWM output
	GPIOATIM3_struct.GPIO_Mode=GPIO_Mode_AF;
	GPIOATIM3_struct.GPIO_OType=GPIO_OType_PP;
	GPIOATIM3_struct.GPIO_Pin=GPIO_Pin_7;
	GPIOATIM3_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOATIM3_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOATIM3_struct);
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
void init_EXTI(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
	EXTI_InitTypeDef EXTI_struct;
	EXTI_struct.EXTI_Line=(EXTI_Line0);
	EXTI_struct.EXTI_LineCmd=ENABLE;
	EXTI_struct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_struct.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_struct);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource1);
	EXTI_struct.EXTI_Line=(EXTI_Line1);
	EXTI_Init(&EXTI_struct);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource2);
	EXTI_struct.EXTI_Line=(EXTI_Line2);
	EXTI_Init(&EXTI_struct);
	NVIC_InitTypeDef EXTI_NVIC;
	EXTI_NVIC.NVIC_IRQChannel=EXTI0_1_IRQn;
	EXTI_NVIC.NVIC_IRQChannelPriority=0;
	EXTI_NVIC.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&EXTI_NVIC);
	EXTI_NVIC.NVIC_IRQChannel=EXTI2_3_IRQn;
	NVIC_Init(&EXTI_NVIC);
}
void EXTI0_1_IRQHandler(void){
	if(EXTI_GetFlagStatus(EXTI_Line1)){
		set_servo(0);
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
	if(EXTI_GetFlagStatus(EXTI_Line0)){
		set_servo(-stp+5);
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}
void EXTI2_3_IRQHandler(void){
	if(EXTI_GetFlagStatus(EXTI_Line2)){
		set_servo(-15);
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
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
	//TIM_CtrlPWMOutputs(TIM2,ENABLE);
	TIM_OC3FastConfig(TIM2,TIM_OCFast_Enable);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	NVIC_InitTypeDef TIM2_NVIC;
	TIM2_NVIC.NVIC_IRQChannel=TIM2_IRQn;
	TIM2_NVIC.NVIC_IRQChannelPriority=0;
	TIM2_NVIC.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&TIM2_NVIC);
	TIM_OCInitTypeDef TIM2_OCstruct={0,};
	TIM2_OCstruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM2_OCstruct.TIM_Pulse=(int)(25500);
	TIM2_OCstruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM2_OCstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	//Init OC3 output at 50% Duty Cycle for testing
	TIM_OC3Init(TIM2,&TIM2_OCstruct);
	TIM_Cmd(TIM2,ENABLE);
}
void TIM2_IRQHandler(void){
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
}
void init_TIM3(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_TimeBaseInitTypeDef TIM3_struct;
	TIM3_struct.TIM_ClockDivision=0;
	TIM3_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM3_struct.TIM_Period=32000;
	TIM3_struct.TIM_Prescaler=14;
	TIM3_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM3,&TIM3_struct);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	NVIC_InitTypeDef TIM3_NVIC;
	TIM3_NVIC.NVIC_IRQChannel=TIM3_IRQn;
	TIM3_NVIC.NVIC_IRQChannelPriority=0;
	TIM3_NVIC.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&TIM3_NVIC);
	TIM_OCInitTypeDef TIM3_OCstruct={0,};
	TIM3_OCstruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM3_OCstruct.TIM_Pulse=(int)(TIM3OC2);
	TIM3_OCstruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM3_OCstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	//Init OC3 output at 50% Duty Cycle for testing
	TIM_OC2Init(TIM3,&TIM3_OCstruct);
	TIM_Cmd(TIM3,ENABLE);
}
void TIM3_IRQHandler(void){
	tmr++;
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}
void init_TIM14(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	TIM_TimeBaseInitTypeDef TIM14_struct;
	TIM14_struct.TIM_ClockDivision=0x0;
	TIM14_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM14_struct.TIM_Period=30000;
	TIM14_struct.TIM_Prescaler=0;
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
	sprintf(usart_char,"%d\n",(int)(240));
	send_packet(usart_char);
	//sprintf(usart_char,"%d\n",tmr);
	//send_packet(usart_char);
	sprintf(usart_char,"%d\n",(int) (ADC_Buffer[0]));
	send_packet(usart_char);
	//sprintf(usart_char,"%d\n",(int) (ADC_Buffer[1]));
	//send_packet(usart_char);
	//sprintf(usart_char,"%d\n",(int) (TIM2->CCR3));
	//send_packet(usart_char);
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
void set_servo(float degree){
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
