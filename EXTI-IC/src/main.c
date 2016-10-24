//********************************************************************
//*							EXTI Input Capture
//*==================================================================*
// Trigger PWM input capture using GPIO EXTI subroutines
//====================================================================
#include "stdio.h"
#include "math.h"
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_adc.h"
//====================================================================
// GLOBAL CONSTANTS
//====================================================================

//====================================================================
// GLOBAL VARIABLES
//====================================================================
char lcdstring[16];
int temp,sysclock,count;
uint32_t TIM3_tick1=0,TIM3_tick2=0,Capture1,Capture2,IC1=0,IC2=0,IC1_temp=0,IC2_temp,cmp=0;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_GPIO(void);
void init_TIM2(void);
void init_TIM3(void);
void init_TIM14(void);
void init_EXTI(void);
void init_ADC(void);
void init_USART1(void);
void send_packet(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void){
	init_LCD();
	init_GPIO();
	//Get operating CLK freq
	RCC_ClocksTypeDef CLK;
	RCC_GetClocksFreq(&CLK);
	sysclock= (CLK.SYSCLK_Frequency)/((float)(pow(10,6)));
	//Sys clocks in MHz
	sprintf(lcdstring,"%d MHz",sysclock);
	lcd_putstring(lcdstring);
	lcd_command(LINE_TWO);
	lcd_putstring("EXTI IC, SW0");
	//Init procedure waits for SW0
	while(GPIO_ReadInputData(GPIOA)&GPIO_IDR_0){}
	lcd_command(CLEAR);
	init_TIM2();		//TIM2 for 50Hz PWM output
	init_TIM3();		//TIM3 for IC Capture
	init_TIM14();		//TIM14 for timed USART comms
	init_ADC();			//ADC In
	init_EXTI();		//EXTI interrupts for IC
	init_USART1();
	uint32_t ADCval;
	for(;;){
		lcd_command(CURSOR_HOME);
		ADCval = ADC_GetConversionValue(ADC1);
		sprintf(lcdstring,"Freq1:%d    ",(int) IC1);
		lcd_putstring(lcdstring);
		lcd_command(LINE_TWO);
		sprintf(lcdstring,"Freq2:%d    ",(int) IC2);
		lcd_putstring(lcdstring);
	}
}											// End of main

void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB),ENABLE);
	GPIO_InitTypeDef GPIOA_struct, GPIOB_struct,GPIOA_ADC,GPIOA_IC,GPIOA_USART,GPIOB_TIM2;
	//GPIOA Init PA0-PA3 Input
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOA,&GPIOA_struct);
	//GPIOA_ADC PA5 ADCIN5 Analog input
	GPIOA_ADC.GPIO_Mode=GPIO_Mode_AN;
	GPIOA_ADC.GPIO_OType=GPIO_OType_PP;
	GPIOA_ADC.GPIO_Pin=GPIO_Pin_5;
	GPIOA_ADC.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOA_ADC.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOA_ADC);
	//IC Capture
	GPIOA_IC.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_IC.GPIO_OType=GPIO_OType_PP;
	GPIOA_IC.GPIO_Pin=GPIO_Pin_7|GPIO_Pin_8;
	GPIOA_IC.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOA_IC.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOA,&GPIOA_IC);
	//USART1 TX and RX PA9-PA10
	GPIOA_USART.GPIO_Mode=GPIO_Mode_AF;
	GPIOA_USART.GPIO_OType=GPIO_OType_PP;
	GPIOA_USART.GPIO_Pin=(GPIO_Pin_9|GPIO_Pin_10);
	GPIOA_USART.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOA_USART.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOA_USART);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);
	//GPIOB PB0-PB7 Outputs Init
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|
			GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOB,&GPIOB_struct);
	GPIO_Write(GPIOB,0x00);
	//GPIOB PB10-PB11 TIM2CH3&TIM2CH4 PWM Outputs AF2 for both
	GPIOB_TIM2.GPIO_Mode=GPIO_Mode_AF;
	GPIOB_TIM2.GPIO_OType=GPIO_OType_PP;
	GPIOB_TIM2.GPIO_Pin=(GPIO_Pin_10|GPIO_Pin_11);
	GPIOB_TIM2.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_TIM2.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOB,&GPIOB_TIM2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_2);
}

void init_TIM2(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_DeInit(TIM2);
	TIM_TimeBaseInitTypeDef TIM2_struct;
	TIM2_struct.TIM_ClockDivision=0;
	TIM2_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM2_struct.TIM_Period=64000;
	TIM2_struct.TIM_Prescaler=14;
	TIM2_struct.TIM_RepetitionCounter=0;
	// TIM2 OC base clock at 50 HZ
	TIM_TimeBaseInit(TIM2,&TIM2_struct);
	TIM_OCInitTypeDef TIM2_OCstruct={0,};
	TIM2_OCstruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM2_OCstruct.TIM_Pulse=(uint32_t) (0.5*(TIM2->ARR));
	TIM2_OCstruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM2_OCstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	//Init OC3 output at 50% Duty Cycle for testing
	TIM_OC3Init(TIM2,&TIM2_OCstruct);
	//TIM_OC4Init(TIM2,&TIM2_OCstruct);
	//TIM_ARRPreloadConfig(TIM2,ENABLE);	//Enable autoreload for variable period
	TIM_Cmd(TIM2,ENABLE);
}
void init_TIM3(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_TimeBaseInitTypeDef TIM3_struct;
	TIM3_struct.TIM_ClockDivision=0x0;
	TIM3_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM3_struct.TIM_Period=64000;
	TIM3_struct.TIM_Prescaler=0x0;
	TIM3_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM3,&TIM3_struct);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
	NVIC_InitTypeDef NVIC_TIM3;
	NVIC_TIM3.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_TIM3.NVIC_IRQChannelPriority=0x0;
	NVIC_TIM3.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM3);
}
void init_TIM14(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	TIM_TimeBaseInitTypeDef TIM14_struct;
	TIM14_struct.TIM_ClockDivision=0x0;
	TIM14_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM14_struct.TIM_Period=64000;
	TIM14_struct.TIM_Prescaler=48;
	TIM14_struct.TIM_RepetitionCounter=0x0;
	TIM_TimeBaseInit(TIM14,&TIM14_struct);
	TIM_ITConfig(TIM14,TIM_IT_Update,ENABLE);
	NVIC_InitTypeDef NVIC_TIM14;
	NVIC_TIM14.NVIC_IRQChannel=TIM14_IRQn;
	NVIC_TIM14.NVIC_IRQChannelPriority=0x1;
	NVIC_TIM14.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM14);
	//TIM_Cmd(TIM14,ENABLE);
}
void init_ADC(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	ADC_InitTypeDef ADC_struct;
	ADC_struct.ADC_ContinuousConvMode=ENABLE;
	ADC_struct.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_struct.ADC_ExternalTrigConv=ADC_ExternalTrigConvEdge_None;
	ADC_struct.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
	ADC_struct.ADC_Resolution=ADC_Resolution_12b;
	ADC_struct.ADC_ScanDirection=ADC_ScanDirection_Upward;
	ADC_Init(ADC1,&ADC_struct);
	ADC_ChannelConfig(ADC1,ADC_Channel_5,ADC_SampleTime_239_5Cycles);
	ADC_Cmd(ADC1,ENABLE);
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_ADRDY)){}
	ADC_StartOfConversion(ADC1);
}
void init_EXTI(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,(EXTI_PinSource7|EXTI_PinSource8));
	EXTI_InitTypeDef EXTI_GPIOA;
	EXTI_GPIOA.EXTI_Line=(EXTI_Line7|EXTI_Line8);
	EXTI_GPIOA.EXTI_LineCmd=ENABLE;
	EXTI_GPIOA.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_GPIOA.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_GPIOA);
	NVIC_InitTypeDef NVIC_EXTI;
	NVIC_EXTI.NVIC_IRQChannel=EXTI4_15_IRQn;
	NVIC_EXTI.NVIC_IRQChannelPriority=0x0;
	NVIC_EXTI.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_EXTI);
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
void EXTI4_15_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line7)==1){
		int temp;
		Capture1=(TIM3->CNT);
		temp=Capture1+TIM3_tick1*63999-IC1_temp;
		IC1=(48.0*pow(10,6)/temp);
		IC1_temp=Capture1;
		TIM3_tick1=0;
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
	else if(EXTI_GetITStatus(EXTI_Line8)==1){
		int temp;
		Capture2=(TIM3->CNT);
		temp=Capture2+TIM3_tick2*63999-IC2_temp;
		IC2=(48.0*pow(10,6)/temp);
		IC2_temp=Capture2;
		TIM3_tick2=0;
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
}
void TIM3_IRQHandler(void){
	TIM3_tick1++;
	TIM3_tick2++;
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}
void TIM14_IRQHandler(void){
	if(cmp<64000){
		cmp+=50;
	}
	else{
		cmp = 0;
	}
	TIM_SetCompare3(TIM2,cmp);
	//send_packet();
	TIM_ClearITPendingBit(TIM14,TIM_IT_Update);
}
void send_packet(void){
	uint8_t ic1u1,ic1u2,ic2u1,ic2u2,oc1,oc2,oc3,oc4;
	ic1u1 = ((IC1>>8)&0b11111111);
	ic1u2 = ((IC1>>0)&0b11111111);
	ic2u1 = ((IC2>>8)&0b11111111);
	ic2u2 = ((IC2>>0)&0b11111111);
	oc1 = (cmp >> 24)&0b11111111;
	oc2 = (cmp >> 16)&0b11111111;
	oc3 = (cmp >> 8)&0b11111111;
	oc4 = (cmp >> 0)&0b11111111;
	USART_SendData(USART1,ic1u1);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,ic1u2);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,255);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,ic2u1);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,ic2u2);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,255);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,oc1);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,oc2);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,oc3);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,oc4);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
}
//********************************************************************
// END OF PROGRAM
//********************************************************************
