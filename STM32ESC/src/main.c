//********************************************************************
//*                    STM32 ESC Test Code
//*==================================================================*
// Uses ADC1CH5 on PA5 in to control pulse width on output compare
// to drive ESC speed comming out from PB10 (TIM2CH3) and reads PWM
// input capture on PA7 (TIM3CH2)
//
// MISSING UPDATED INPUT CAPTURE
//====================================================================
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_tim.h"
//====================================================================
// GLOBAL CONSTANTS
//====================================================================
char lcdstring[16];
int temp,count=0;
float width;
uint32_t Frequency,DutyCycle;
//====================================================================
// GLOBAL VARIABLES
//====================================================================

//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_Ports(void);
void init_TIM2(void);
void init_TIM3(void);
void init_ADC(void);

//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_LCD();								// Initialise lcd
	init_Ports();
	lcd_putstring("ESC PWM Test");
	lcd_command(LINE_TWO);
	lcd_putstring("Input Capture");
	while(GPIO_ReadInputData(GPIOA)&GPIO_IDR_0){}
	GPIO_Write(GPIOB,0xff);
	lcd_command(CLEAR);
	//Secondary init functions
	init_ADC();
	init_TIM2();
	init_TIM3();
	for(;;){
		lcd_command(CURSOR_HOME);
		width=(((ADC_GetConversionValue(ADC1))/255.0)*1.1+1);
		sprintf(lcdstring,"Pulse:%d [u.s]",(int)(100*width));
		lcd_putstring(lcdstring);
		TIM_SetCompare3(TIM2,(uint32_t)((width/10.0)*32000));
		sprintf(lcdstring,"Freq:%d  ",(int)Frequency);
		lcd_command(LINE_TWO);
		lcd_putstring(lcdstring);
	}
}											// End of main

void init_Ports(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB),ENABLE);
	GPIO_InitTypeDef GPIOA_struct,GPIOA_TIM3,GPIOA_ADC,GPIOB_struct,GPIOB_TIM2;
	//Init PA0-PA3 as inputs
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_1;
	GPIO_Init(GPIOA,&GPIOA_struct);
	//Init PA5 as ADC1CH5 Analogue Input
	GPIOA_ADC.GPIO_Mode=GPIO_Mode_AN;
	GPIOA_ADC.GPIO_OType=GPIO_OType_PP;
	GPIOA_ADC.GPIO_Pin=GPIO_Pin_5;
	GPIOA_ADC.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOA_ADC.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOA,&GPIOA_ADC);
	//Init PA7 as TIM3CH2 Input, AF1
	GPIOA_TIM3.GPIO_Mode=GPIO_Mode_AF;
	GPIOA_TIM3.GPIO_OType=GPIO_OType_PP;
	GPIOA_TIM3.GPIO_Pin=GPIO_Pin_7;
	GPIOA_TIM3.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_TIM3.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOA_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_1);
	//Init PB0-PB7 as outputs
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|
			GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_struct.GPIO_Speed=GPIO_Speed_Level_1;
	GPIO_Init(GPIOB,&GPIOB_struct);
	//Init PB10 as TIM2CH3 PWM Output AF2
	GPIOB_TIM2.GPIO_Mode=GPIO_Mode_AF;
	GPIOB_TIM2.GPIO_OType=GPIO_OType_PP;
	GPIOB_TIM2.GPIO_Pin=GPIO_Pin_10;
	GPIOB_TIM2.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_TIM2.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOB,&GPIOB_TIM2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_2);
}
void init_TIM2(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_TimeBaseInitTypeDef TIM2BASE_struct;
	TIM2BASE_struct.TIM_ClockDivision=0;
	TIM2BASE_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM2BASE_struct.TIM_Period=32000;
	TIM2BASE_struct.TIM_Prescaler=14;
	TIM2BASE_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM2BASE_struct);
	TIM_OCInitTypeDef TIM2OC_struct={0,};
	TIM_OCStructInit(&TIM2OC_struct);
	TIM2OC_struct.TIM_OCMode=TIM_OCMode_PWM1;
	//TIM2OC_struct.TIM_Pulse=(uint32_t)((0.9/20.0)*64000);
	TIM2OC_struct.TIM_OutputState=TIM_OutputState_Enable;
	TIM2OC_struct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OC3Init(TIM2,&TIM2OC_struct);
	TIM_Cmd(TIM2,ENABLE);
}
void init_ADC(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	ADC_InitTypeDef ADC_struct;
	ADC_StructInit(&ADC_struct);
	ADC_struct.ADC_ContinuousConvMode=ENABLE;
	ADC_struct.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_struct.ADC_Resolution=ADC_Resolution_8b;
	ADC_struct.ADC_ScanDirection=ADC_ScanDirection_Upward;
	ADC_Init(ADC1,&ADC_struct);
	ADC_ChannelConfig(ADC1,ADC_Channel_5,ADC_SampleTime_55_5Cycles);
	ADC_Cmd(ADC1,ENABLE);
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_ADRDY)){}
	ADC_StartOfConversion(ADC1);
}
void init_TIM3(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_ICInitTypeDef TIM3CH2_ICStruct;
	TIM3CH2_ICStruct.TIM_Channel=TIM_Channel_2;
	TIM3CH2_ICStruct.TIM_ICFilter=0x0;
	TIM3CH2_ICStruct.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM3CH2_ICStruct.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM3CH2_ICStruct.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_PWMIConfig(TIM3,&TIM3CH2_ICStruct);
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);
	TIM_ITConfig(TIM3,TIM_IT_CC2,ENABLE);
	NVIC_InitTypeDef NVIC_TIM3;
	NVIC_TIM3.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_TIM3.NVIC_IRQChannelPriority=0;
	NVIC_TIM3.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM3);
	TIM_Cmd(TIM3,ENABLE);
}
void TIM3_IRQHandler(void){
	uint32_t IC2Value;
	IC2Value = TIM_GetCapture2(TIM3);
	if (IC2Value != 0){
		Frequency = (48*pow(10,6)) / IC2Value;
	}
	else{
		Frequency = 0;
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
}

//********************************************************************
// END OF PROGRAM
//********************************************************************
