//********************************************************************
//*							Servo Step Test
//*==================================================================*
// Measures unit step response for Corona DSM339 MG Digital Servo
// PB10 TIM2CH3 PWM out at 330 HZ. TIM14 Schedules USART packets
// ADC1CH5 input measures linear rotatry 10K potentiometer with 305
// degrees deflection. Sends USART packets through USART1RX
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
char lcdstring[16],usart_char[16];
int count=0,sysclock,ADCval,TIMcmp,TIM3OC2=2500;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_GPIO(void);
void init_EXTI(void);
void init_ADC(void);
void init_TIM2(void);
void init_TIM3(void);
void init_TIM14(void);
void init_TIM17(void);
void init_USART1(void);
void send_packet(const char *str);

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
	lcd_putstring("Servo Step Test, SW0");
	//Init procedure waits for SW0
	while(GPIO_ReadInputData(GPIOA)&GPIO_IDR_0){}
	lcd_command(CLEAR);
	init_ADC();
	init_EXTI();
	init_USART1();
	init_TIM2();
	init_TIM3();
	init_TIM14();
	init_TIM17();
	for(;;){
		while(TIM3OC2<=3800){
			TIM3OC2++;
			TIM_SetCompare2(TIM3,TIM3OC2);
			for(int x=0;x<=255;x++){
				for(int y=0;y<=255;y++){}
			}
		}
		lcd_command(CURSOR_HOME);
		sprintf(lcdstring,"ADC:%d     ",(int) ADCval);
		lcd_putstring(lcdstring);
		lcd_command(LINE_TWO);
		sprintf(lcdstring,"TIM:%d     ",(int) TIM2->CCR3);//TIMcmp);
		//TIM_SetCompare2(TIM3,(int)(64000*((0.9+1.1*ADCval/2047.0)/20.0)));
		lcd_putstring(lcdstring);
	}
}											// End of main

void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB),ENABLE);
	GPIO_InitTypeDef GPIOA_struct,GPIOB_struct,GPIOAUSART,GPIOAADC,GPIOA_TIM3,GPIOB_TIM2;
	//GPIOA Init PA0-PA3 Input
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1);
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOA,&GPIOA_struct);
	// PA7 TIM3CH2 AF1
	GPIOA_TIM3.GPIO_Mode=GPIO_Mode_AF;
	GPIOA_TIM3.GPIO_OType=GPIO_OType_PP;
	GPIOA_TIM3.GPIO_Pin=GPIO_Pin_7;
	GPIOA_TIM3.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOA_TIM3.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOA_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_1);
	// PA5 ADC1IN5
	GPIOAADC.GPIO_Mode=GPIO_Mode_AN;
	GPIOAADC.GPIO_OType=GPIO_OType_PP;
	GPIOAADC.GPIO_Pin=GPIO_Pin_5;
	GPIOAADC.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOAADC.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOAADC);
	//PA9-PA10 USART RX/TX
	GPIOAUSART.GPIO_Mode=GPIO_Mode_AF;
	GPIOAUSART.GPIO_OType=GPIO_OType_PP;
	GPIOAUSART.GPIO_Pin=(GPIO_Pin_9|GPIO_Pin_10);
	GPIOAUSART.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOAUSART.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOAUSART);
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
void init_EXTI(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource1);
	EXTI_InitTypeDef EXTI_struct;
	EXTI_struct.EXTI_Line=EXTI_Line1;
	EXTI_struct.EXTI_LineCmd=ENABLE;
	EXTI_struct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_struct.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_struct);
	NVIC_InitTypeDef NVIC_EXTI;
	NVIC_EXTI.NVIC_IRQChannel=EXTI0_1_IRQn;
	NVIC_EXTI.NVIC_IRQChannelPriority=1;
	NVIC_EXTI.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_EXTI);
}
void init_ADC(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	ADC_InitTypeDef ADC_struct;
	ADC_struct.ADC_ContinuousConvMode=ENABLE;
	ADC_struct.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_struct.ADC_ExternalTrigConv=ADC_ExternalTrigConvEdge_None;
	ADC_struct.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
	ADC_struct.ADC_Resolution=ADC_Resolution_10b;
	ADC_struct.ADC_ScanDirection=ADC_ScanDirection_Upward;
	ADC_Init(ADC1,&ADC_struct);
	ADC_ChannelConfig(ADC1,ADC_Channel_5,ADC_SampleTime_1_5Cycles);
	ADC_Cmd(ADC1,ENABLE);
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_ADRDY)){}
	ADC_StartOfConversion(ADC1);
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
void init_TIM2(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_TimeBaseInitTypeDef TIM2_struct;
	TIM2_struct.TIM_ClockDivision=0;
	TIM2_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM2_struct.TIM_Period=48485;
	TIM2_struct.TIM_Prescaler=2;
	TIM2_struct.TIM_RepetitionCounter=0;
	// TIM2 OC base clock at 330 HZ
	TIM_TimeBaseInit(TIM2,&TIM2_struct);
	TIM_OCInitTypeDef TIM2_OCstruct={0,};
	TIM2_OCstruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM2_OCstruct.TIM_Pulse=(int)(16000);
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
	TIM3_OCstruct.TIM_Pulse=(int)(TIM3OC2);
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
	TIM14_struct.TIM_Period=60000;
	TIM14_struct.TIM_Prescaler=7;
	TIM14_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM14,&TIM14_struct);
	TIM_ITConfig(TIM14,TIM_IT_Update,ENABLE);
	NVIC_InitTypeDef NVIC_TIM14;
	NVIC_TIM14.NVIC_IRQChannel=TIM14_IRQn;
	NVIC_TIM14.NVIC_IRQChannelPriority=0;
	NVIC_TIM14.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM14);
	TIM_Cmd(TIM14,ENABLE);
}
void init_TIM17(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17,ENABLE);
	TIM_TimeBaseInitTypeDef TIM17_struct;
	TIM17_struct.TIM_ClockDivision=0x0;
	TIM17_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM17_struct.TIM_Period=60000;
	TIM17_struct.TIM_Prescaler=7;
	TIM17_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM17,&TIM17_struct);
	TIM_ITConfig(TIM17,TIM_IT_Update,ENABLE);
	NVIC_InitTypeDef NVIC_TIM17;
	NVIC_TIM17.NVIC_IRQChannel=TIM17_IRQn;
	NVIC_TIM17.NVIC_IRQChannelPriority=0;
	NVIC_TIM17.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM17);
	TIM_Cmd(TIM17,ENABLE);
}
void EXTI0_1_IRQHandler(void){
	TIM_SetCompare3(TIM2,28800);
	EXTI_ClearITPendingBit(EXTI_Line1);
}
void TIM14_IRQHandler(void){
	sprintf(usart_char,"%d\n", 240);
	send_packet(usart_char);
	sprintf(usart_char,"%d\n",(int) count);
	send_packet(usart_char);
	sprintf(usart_char,"%d\n",(int) TIMcmp);
	send_packet(usart_char);
	sprintf(usart_char,"%d\n",(int) (ADCval));
	send_packet(usart_char);
	TIM_ClearITPendingBit(TIM14,TIM_IT_Update);
}
void TIM17_IRQHandler(void){
	count++;
	ADCval=ADC_GetConversionValue(ADC1);
	TIMcmp = TIM2->CCR3;
	TIM_ClearITPendingBit(TIM17,TIM_IT_Update);
}
void send_packet(const char *str){
	while(*str){
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0);
		USART_SendData(USART1,*str++);
	}
}
//********************************************************************
// END OF PROGRAM
//********************************************************************
