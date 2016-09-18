//********************************************************************
//*							Input Capture
//*==================================================================*
// Produces frequency varying PWM with 50% duty cycle and captures
// wave frequency, used for BLDC RPM sensor
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
int temp,count=0,tim3_count=0,sysclock;
uint32_t DutyCycle,Frequency,TIM3_tick=0,CCR2_comp=0;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_GPIO(void);
void init_TIM2(void);
void init_TIM3(void);
void init_ADC(void);

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
	lcd_putstring("TIM3 IC, SW0");
	//Init procedure waits for SW0
	while(GPIO_ReadInputData(GPIOA)&GPIO_IDR_0){}
	lcd_command(CLEAR);
	init_TIM2();
	init_TIM3();
	init_ADC();
	uint32_t ADCval;
	for(;;){
		lcd_command(CURSOR_HOME);
		ADCval = ADC_GetConversionValue(ADC1);
		sprintf(lcdstring,"Duty:%d     ",(int) ADCval);
		lcd_putstring(lcdstring);
		lcd_command(LINE_TWO);
		sprintf(lcdstring,"Freq:%d     ",(int)  Frequency);
		lcd_putstring(lcdstring);

	}
}											// End of main

void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB),ENABLE);
	GPIO_InitTypeDef GPIOA_struct, GPIOB_struct,GPIOA_ADC,GPIOA_TIM3,GPIOB_TIM2;
	//GPIOA Init
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOA,&GPIOA_struct);
	//GPIOA_ADC PA5 ADCIN5 Analogue input
	GPIOA_ADC.GPIO_Mode=GPIO_Mode_AN;
	GPIOA_ADC.GPIO_OType=GPIO_OType_PP;
	GPIOA_ADC.GPIO_Pin=GPIO_Pin_5;
	GPIOA_ADC.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOA_ADC.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOA_ADC);
	//GPIOA_TIM3 PA7 TIM3CH2 Input AF1
	GPIOA_TIM3.GPIO_Mode=GPIO_Mode_AF;
	GPIOA_TIM3.GPIO_OType=GPIO_OType_PP;
	GPIOA_TIM3.GPIO_Pin=GPIO_Pin_7;
	GPIOA_TIM3.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_TIM3.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOA_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_1);
	//GPIOB Init
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
	TIM_TimeBaseInit(TIM2,&TIM2_struct);
	TIM_OCInitTypeDef TIM2_OCstruct={0,};
	TIM2_OCstruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM2_OCstruct.TIM_Pulse=(uint32_t) (0.5*(TIM2->ARR));
	TIM2_OCstruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM2_OCstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OC3Init(TIM2,&TIM2_OCstruct);
	//TIM_OC4Init(TIM2,&TIM2_OCstruct);
	TIM_ARRPreloadConfig(TIM2,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
}
void init_TIM3(void){
	//Local Variables
	int prescaler;
	TIM_TimeBaseInitTypeDef TIM3_struct;
	TIM_ICInitTypeDef TIM3IC2_struct;
	NVIC_InitTypeDef NVIC_TIM3;
	//
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	//Tim base structure
	TIM3_struct.TIM_ClockDivision=0x0;
	TIM3_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM3_struct.TIM_Prescaler=0;
	TIM3_struct.TIM_Period=65535;
	TIM_TimeBaseInit(TIM3,&TIM3_struct);
	prescaler = (sysclock/1)-1;		//10 Mhz shooting
	TIM_PrescalerConfig(TIM3,prescaler,TIM_PSCReloadMode_Immediate);
	//Input capture config CH2 only
	TIM3IC2_struct.TIM_Channel=TIM_Channel_2;
	TIM3IC2_struct.TIM_ICFilter=0x00;
	TIM3IC2_struct.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM3IC2_struct.TIM_ICPrescaler=TIM_ICPSC_DIV2;
	TIM3IC2_struct.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_PWMIConfig(TIM3,&TIM3IC2_struct);
	TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);
	//Interrupts
	TIM_ITConfig(TIM3,(TIM_IT_Update|TIM_IT_CC2),ENABLE);
	//Enable TIM3
	TIM_Cmd(TIM3,ENABLE);
	//Unmask global interrupt for TM3
	NVIC_TIM3.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_TIM3.NVIC_IRQChannelPriority=1;
	NVIC_TIM3.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM3);
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
void TIM3_IRQHandler(void){
	uint32_t deltatick;
	if(TIM_GetFlagStatus(TIM3,TIM_IT_Update)){
		TIM3_tick++;
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
	else if(TIM_GetFlagStatus(TIM3,TIM_IT_CC2)){
		temp = TIM_GetCapture2(TIM3);
		deltatick = temp + TIM3_tick*(TIM3->ARR) - CCR2_comp;
		Frequency = (float) (2*(pow(10,6)*sysclock/(TIM3->PSC+1))/(deltatick));
		TIM3_tick=0;
		CCR2_comp=temp;
		TIM_ClearITPendingBit(TIM3,TIM_IT_CC2);
	}
}

//********************************************************************
// END OF PROGRAM
//********************************************************************
