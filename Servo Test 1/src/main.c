//********************************************************************
// Tx data to PC via USART
// System ID for Servo
// 305 degree rotation
// 10 K pot
//
//====================================================================
#include "stdio.h"
#include "math.h"
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
//====================================================================
// GLOBAL CONSTANTS
//====================================================================

//====================================================================
// GLOBAL VARIABLES
//====================================================================
char lcdstring[16];
int deg;
uint32_t adc1,temp, count=0, timer=0, comp=0;
RCC_ClocksTypeDef sysclocks;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_GPIO(void);
void init_TIM2(void);
void init_TIM3(void);
void init_TIM14(void);
void init_USART1(void);
void init_ADC(void);
void send_packet(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	RCC_GetClocksFreq(&sysclocks);
	temp = (sysclocks.SYSCLK_Frequency)/(pow(10,6));
	init_LCD();								// Initialise lcd
	lcd_putstring("USART -> USB");		// Display string on line 1
	lcd_command(LINE_TWO);					// Move cursor to line 2
	sprintf(lcdstring,"%d MHz, SW0",(int) temp);
	lcd_putstring(lcdstring);				// Display string on line 2
	init_GPIO();
	while(GPIO_ReadInputData(GPIOA)&GPIO_IDR_0){}
	lcd_command(CLEAR);
	init_TIM2();
	init_TIM3();
	init_TIM14();
	init_ADC();
	init_USART1();
	GPIO_Write(GPIOB,0xff);
	for(;;){
		lcd_command(CURSOR_HOME);
		adc1 = ADC_GetConversionValue(ADC1);
		deg = (adc1/4095.0)*305;
		sprintf(lcdstring,"%d  degree",(int) adc1);
		lcd_putstring(lcdstring);
		lcd_command(LINE_TWO);
		sprintf(lcdstring,"count: %d   ",(int) (TIM3->CCR2));
		lcd_putstring(lcdstring);
	}
}											// End of main

//********************************************************************
// END OF PROGRAM
//********************************************************************
void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBENR_GPIOBEN),ENABLE);
	GPIO_InitTypeDef GPIOA_struct,GPIOAUSART,GPIOAADC,GPIOATIM3,GPIOB_struct;
	//PA0-PA3 Inputs
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOA,&GPIOA_struct);
	//PA9-PA10
	GPIOAUSART.GPIO_Mode=GPIO_Mode_AF;
	GPIOAUSART.GPIO_OType=GPIO_OType_PP;
	GPIOAUSART.GPIO_Pin=(GPIO_Pin_9|GPIO_Pin_10);
	GPIOAUSART.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOAUSART.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOAUSART);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);
	//PA5
	GPIOAADC.GPIO_Mode=GPIO_Mode_AN;
	GPIOAADC.GPIO_OType=GPIO_OType_PP;
	GPIOAADC.GPIO_Pin=GPIO_Pin_5;
	GPIOAADC.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOAADC.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOAADC);
	//PA7
	GPIOATIM3.GPIO_Mode=GPIO_Mode_AF;
	GPIOATIM3.GPIO_OType=GPIO_OType_PP;
	GPIOATIM3.GPIO_Pin=GPIO_Pin_7;
	GPIOATIM3.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOATIM3.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOATIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_1);
	//PB0-PB7
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOB,&GPIOB_struct);
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
	ADC_ChannelConfig(ADC1,ADC_Channel_5,ADC_SampleTime_55_5Cycles);
	ADC_Cmd(ADC1,ENABLE);
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_ADRDY)){}
	ADC_StartOfConversion(ADC1);
}
void init_TIM2(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_TimeBaseInitTypeDef TIM2_struct;
	TIM2_struct.TIM_ClockDivision=0x00;
	TIM2_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM2_struct.TIM_Period=48000;
	TIM2_struct.TIM_Prescaler=0x0;
	TIM2_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM2_struct);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
	NVIC_InitTypeDef NVIC_TIM2;
	NVIC_TIM2.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_TIM2.NVIC_IRQChannelCmd=ENABLE;
	NVIC_TIM2.NVIC_IRQChannelPriority=1;
	NVIC_Init(&NVIC_TIM2);
}
void init_TIM3(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_TimeBaseInitTypeDef TIM3_struct;
	TIM3_struct.TIM_ClockDivision=0x00;
	TIM3_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM3_struct.TIM_Period=48485;
	TIM3_struct.TIM_Prescaler=0x2;
	TIM3_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM3,&TIM3_struct);
	TIM_OCInitTypeDef TIM3_OCstruct={0,};
	TIM3_OCstruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM3_OCstruct.TIM_Pulse=(uint32_t) (0.396*48485);
	TIM3_OCstruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM3_OCstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OC2Init(TIM3,&TIM3_OCstruct);
	//Start TIMER3
	TIM_Cmd(TIM3,ENABLE);
}
void init_TIM14(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	TIM_TimeBaseInitTypeDef TIM14_struct;
	TIM14_struct.TIM_ClockDivision=0x0;
	TIM14_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM14_struct.TIM_Period=64865;
	TIM14_struct.TIM_Prescaler=36;
	TIM14_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM14,&TIM14_struct);
	TIM_ITConfig(TIM14,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM14,ENABLE);
	NVIC_InitTypeDef NVIC_TIM14;
	NVIC_TIM14.NVIC_IRQChannel=TIM14_IRQn;
	NVIC_TIM14.NVIC_IRQChannelCmd=ENABLE;
	NVIC_TIM14.NVIC_IRQChannelPriority=1;
	NVIC_Init(&NVIC_TIM14);
}

void TIM14_IRQHandler(void){
	count++;
	if(count == 5){
		if(comp <= 48485){
			comp+=50;
		}
		else{
			comp = 0;
		}
		count = 0;
	}
	TIM_SetCompare2(TIM3,comp);
	send_packet();
	TIM_ClearFlag(TIM14,TIM_FLAG_Update);
}
void TIM2_IRQHandler(void){
	timer++;
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
}
void send_packet(void){
	uint8_t a1, a2, i1, i2, i3, i4;
	a1 = ((adc1>>8)&0b11111111);
	a2 = ((adc1>>0)&0b11111111);
	temp = (TIM3->CCR2);
	i1 = (temp >> 24)&0b11111111;
	i2 = (temp >> 16)&0b11111111;
	i3 = (temp >> 8)&0b11111111;
	i4 = (temp >> 0)&0b11111111;
	USART_SendData(USART1,240);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,a1);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,a2);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,0);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,i1);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,i2);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,i3);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,i4);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
	USART_SendData(USART1,0);
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE)){}
}
