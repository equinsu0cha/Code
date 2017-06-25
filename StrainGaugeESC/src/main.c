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
//Global Variables
char lcdstring[16],usart_char[16];;
int sysclock,temp=14900,tmr=0,dir=1,TIM3OC2=2500;
uint16_t ADC_Buffer[1];
//Function Declarations
void init_GPIO(void);
void init_ADC(void);
void ADC_NVIC(void);
void init_DMA(void);
void init_TIM2(void);
void init_TIM3(void);
void init_TIM14(void);
void TIM14_NVIC(void);
void init_TIM17(void);
void TIM17_NVIC(void);
void init_EXTI(void);
void EXTI_NVIC(void);
void init_USART1(void);
void set_servo(float degree);
void send_packet(const char *str);
//Main Loops
void main(void){
	//Init primary functions
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
	lcd_putstring("Strain Gauge");
	GPIO_Write(GPIOB,0x00);
	while((GPIO_ReadInputData(GPIOA)&GPIO_IDR_0)){}
	//Run loop check
	lcd_command(CLEAR);
	lcd_putstring("Init");
	GPIO_Write(GPIOB,0xff);
	//Init secondary functions
	init_ADC();
	init_DMA();
	init_TIM2();
	init_TIM3();
	init_TIM14();
	init_TIM17();
	init_EXTI();
	init_USART1();
	// Start ADC
	ADC_StartOfConversion(ADC1);
	//For Loop
	set_servo(0);
	while((GPIO_ReadInputData(GPIOA)&GPIO_IDR_0)){}
	lcd_command(CLEAR);
	lcd_putstring("Spin up");
	while(TIM3OC2<=3800){
		TIM3OC2++;
		TIM_SetCompare2(TIM3,TIM3OC2);
		GPIO_Write(GPIOB,(uint16_t)((255*TIM3OC2/3800)));
		for(int x=0;x<=255;x++){
			for(int y=0;y<=255;y++){}
		}
	}
	for(;;){
		lcd_command(CURSOR_HOME);
		sprintf(lcdstring,"DEG:%d       ",(int)(ADC_Buffer[0]));
		lcd_putstring(lcdstring);
		//sprintf(lcdstring,"CMD:%d       ",(int)(ADC_Buffer[1]));
		//lcd_command(LINE_TWO);
		//lcd_putstring(lcdstring);
	}
}

void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN),ENABLE);
	GPIO_InitTypeDef GPIOA_struct,GPIOB_struct,GPIOAADC1_struct,GPIOAUSART,GPIOATIM3_struct,GPIOBTIM2_struct;
	// GPIOA PA0-PA2 Inputs
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2);
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOA,&GPIOA_struct);
	// GPIOA PA5-PA6 ADC1CH5-ADC1CH6 input
	GPIOAADC1_struct.GPIO_Mode=GPIO_Mode_AN;
	GPIOAADC1_struct.GPIO_OType=GPIO_OType_PP;
	GPIOAADC1_struct.GPIO_Pin=(GPIO_Pin_5|GPIO_Pin_6);
	GPIOAADC1_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOAADC1_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOAADC1_struct);
	// GPIOA PA7 TIM3CH2 output
	GPIOATIM3_struct.GPIO_Mode=GPIO_Mode_AF;
	GPIOATIM3_struct.GPIO_OType=GPIO_OType_PP;
	GPIOATIM3_struct.GPIO_Pin=GPIO_Pin_7;
	GPIOATIM3_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOATIM3_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOATIM3_struct);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_1);
	//PA9-PA10 USART1 RX/TX
	GPIOAUSART.GPIO_Mode=GPIO_Mode_AF;
	GPIOAUSART.GPIO_OType=GPIO_OType_PP;
	GPIOAUSART.GPIO_Pin=(GPIO_Pin_9|GPIO_Pin_10);
	GPIOAUSART.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOAUSART.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOAUSART);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);
	// GPIOB PB0-PB7 Outputs Init
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOB,&GPIOB_struct);
	// GPIOB PB10 TIM2CH3 PWM output
	GPIOBTIM2_struct.GPIO_Mode=GPIO_Mode_AF;
	GPIOBTIM2_struct.GPIO_OType=GPIO_OType_PP;
	GPIOBTIM2_struct.GPIO_Pin=(GPIO_Pin_10);
	GPIOBTIM2_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOBTIM2_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOB,&GPIOBTIM2_struct);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_2);
}

void init_ADC(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	// Enable ADC1 in 10 bit mode, CH5 and CH6 on PA5 and PA6 respectively
	ADC_InitTypeDef ADC1_struct;
	ADC1_struct.ADC_ContinuousConvMode=DISABLE;
	ADC1_struct.ADC_DataAlign=ADC_DataAlign_Right;
	ADC1_struct.ADC_ExternalTrigConv=ADC_ExternalTrigConvEdge_None;
	ADC1_struct.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
	ADC1_struct.ADC_Resolution=ADC_Resolution_12b;
	ADC1_struct.ADC_ScanDirection=ADC_ScanDirection_Upward;
	ADC_Init(ADC1,&ADC1_struct);
	ADC_ChannelConfig(ADC1,ADC_Channel_5,ADC_SampleTime_55_5Cycles);
	//ADC_ChannelConfig(ADC1,ADC_Channel_6,ADC_SampleTime_55_5Cycles);
	// Enable end of conversion interrupt
	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
	ADC_NVIC();
	// Use DMA for both channels, oneshot mode, reset in ADC EOC interrupt
	ADC_DMARequestModeConfig(ADC1,ADC_DMAMode_Circular);
	ADC_DMACmd(ADC1,ENABLE);
	// Enable ADC
	ADC_Cmd(ADC1,ENABLE);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_ADRDY)){}
}
void ADC_NVIC(void){
	NVIC_InitTypeDef ADCNVIC;
	ADCNVIC.NVIC_IRQChannel=ADC1_COMP_IRQn;
	ADCNVIC.NVIC_IRQChannelPriority=0x01;
	ADCNVIC.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&ADCNVIC);
}
void ADC1_COMP_IRQHandler(void){
	ADC_StartOfConversion(ADC1);
	ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
}
void init_DMA(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,1);
	//Enable DMA for 2 channel ADC input
	DMA1_Channel1->CNDTR=0x1;
	DMA1_Channel1->CPAR=(uint32_t) &(ADC1->DR);
	DMA1_Channel1->CMAR=(uint32_t) &(ADC_Buffer[0]);
	DMA1_Channel1->CCR=(DMA_M2M_Disable|DMA_Priority_VeryHigh|DMA_MemoryDataSize_HalfWord|DMA_PeripheralDataSize_HalfWord
			|DMA_MemoryInc_Enable|DMA_PeripheralInc_Disable|DMA_Mode_Circular|DMA_DIR_PeripheralSRC|DMA_CCR_EN);
}
void init_TIM2(void){
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2EN,ENABLE);
	TIM_TimeBaseInitTypeDef TIM2_struct;
	// Timer base struct at 330Hz intervals
	TIM2_struct.TIM_ClockDivision=0;
	TIM2_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM2_struct.TIM_Period=48485;
	TIM2_struct.TIM_Prescaler=2;
	TIM2_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM2_struct);
	// OCcompare PWM mode for CH3
	TIM_OCInitTypeDef TIM2_OCstruct={0,};
	TIM2_OCstruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM2_OCstruct.TIM_Pulse=(int)(0);
	TIM2_OCstruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM2_OCstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	//Init OC3 output at 50% Duty Cycle for testing
	TIM_OC3Init(TIM2,&TIM2_OCstruct);
	TIM_Cmd(TIM2,ENABLE);
}
void init_TIM3(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_TimeBaseInitTypeDef TIM3_struct;
	TIM3_struct.TIM_ClockDivision=0;
	TIM3_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM3_struct.TIM_Period=64000;
	TIM3_struct.TIM_Prescaler=14;
	TIM3_struct.TIM_RepetitionCounter=0;
	// TIM3 OC base clock at 50 HZ
	TIM_TimeBaseInit(TIM3,&TIM3_struct);
	TIM_OCInitTypeDef TIM3_OCstruct={0,};
	TIM3_OCstruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM3_OCstruct.TIM_Pulse=(int)(0);
	TIM3_OCstruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM3_OCstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	//Init OC2 output at 50% Duty Cycle for testing
	TIM_OC2Init(TIM3,&TIM3_OCstruct);
	TIM_Cmd(TIM3,ENABLE);
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
	TIM14_NVIC();
	TIM_Cmd(TIM14,ENABLE);
}
void TIM14_NVIC(void){
	NVIC_InitTypeDef NVIC_TIM14;
	NVIC_TIM14.NVIC_IRQChannel=TIM14_IRQn;
	NVIC_TIM14.NVIC_IRQChannelPriority=0;
	NVIC_TIM14.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM14);
}
void TIM14_IRQHandler(void){

	sprintf(usart_char,"%d\n", 240);
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
void init_TIM17(void){
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM17EN,ENABLE);
	TIM_TimeBaseInitTypeDef TIM17_struct;
	TIM17_struct.TIM_ClockDivision=0;
	TIM17_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM17_struct.TIM_Period=60000;
	TIM17_struct.TIM_Prescaler=255;
	TIM17_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM3,&TIM17_struct);
	TIM_ITConfig(TIM17,TIM_IT_Update,ENABLE);
	TIM17_NVIC();
	//TIM_Cmd(TIM3,ENABLE);
}
void TIM17_NVIC(void){
	NVIC_InitTypeDef NVIC_TIM17;
	NVIC_TIM17.NVIC_IRQChannel=TIM17_IRQn;
	NVIC_TIM17.NVIC_IRQChannelPriority=0;
	NVIC_TIM17.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM17);
}
void TIM17_IRQHandler(void){
	TIM_ClearITPendingBit(TIM17,TIM_IT_Update);
}
void init_EXTI(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource2);
	EXTI_InitTypeDef EXTI_struct;
	EXTI_struct.EXTI_Line=(EXTI_Line1|EXTI_Line2);
	EXTI_struct.EXTI_LineCmd=ENABLE;
	EXTI_struct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_struct.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_struct);
	EXTI_NVIC();
}
void EXTI_NVIC(void){
	NVIC_InitTypeDef NVIC_EXTI;
	NVIC_EXTI.NVIC_IRQChannel=EXTI0_1_IRQn;
	NVIC_EXTI.NVIC_IRQChannelPriority=1;
	NVIC_EXTI.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_EXTI);
	NVIC_EXTI.NVIC_IRQChannel=EXTI2_3_IRQn;
	NVIC_Init(&NVIC_EXTI);
}
void EXTI0_1_IRQHandler(void){
	if(EXTI_GetFlagStatus(EXTI_Line1)){
		set_servo(0);
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}
void EXTI2_3_IRQHandler(void){
	if(EXTI_GetFlagStatus(EXTI_Line2)){
		set_servo(-90);
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
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
// Set servo position, connected to TIM2CH3 output compare
void set_servo(float degree){
	if(degree<0){
		TIM_SetCompare3(TIM2,(int) (10600*(degree+216.5)/90));
	}
	if(degree>=0){
		TIM_SetCompare3(TIM2,(int) (13000*(degree+176.53)/90));
	}
}
//Send USART packet
void send_packet(const char *str){
	while(*str){
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0);
		USART_SendData(USART1,*str++);
	}
}
// ----------------------------------------------------------------------------
