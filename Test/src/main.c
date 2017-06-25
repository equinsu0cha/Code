//Include
#include "math.h"
#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
#include "stdio.h"
#include "stm32f0xx_TIM.h"
#include "stm32f0xx_DMA.h"
//Variables
char lcdstring[16];
//uninitialized variables
int temp,sysclock;
//initialized variables
int tmr=0,TIM3OC2=3000;;
uint16_t ADC_Buffer[1];
//Function defs
void init_GPIO(void);
void init_ADC(void);
void init_DMA(void);
void init_TIM2(void);
void init_TIM3(void);
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
	init_ADC();
	init_DMA();
	init_TIM2();
	init_TIM3();
	//Start ADC Conversion
	ADC_StartOfConversion(ADC1);
	//Spin up BLDC Motor
	lcd_putstring("Spin UP");
	while(TIM3OC2<=3600){
		int GPIO_LED_temp;
		TIM3OC2++;
		TIM_SetCompare2(TIM3,TIM3OC2);
		GPIO_LED_temp = (int)(255.0*(TIM3OC2-3000)/600);
		GPIO_Write(GPIOB,GPIO_LED_temp);
		for(int x=0;x<=255;x++){
			for(int y=0;y<=255;y++){}
		}
	}
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
	GPIO_InitTypeDef GPIOA_struct,GPIOAADC_struct,GPIOATIM3_struct,GPIOB_struct,GPIOBTIM2_struct;
	//PA0-PA1 inputs
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1);
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
	//NVIC_Init(&DMA_NVIC);
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
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	NVIC_InitTypeDef TIM2_NVIC;
	TIM2_NVIC.NVIC_IRQChannel=TIM2_IRQn;
	TIM2_NVIC.NVIC_IRQChannelPriority=0;
	TIM2_NVIC.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&TIM2_NVIC);
	TIM_OCInitTypeDef TIM2_OCstruct={0,};
	TIM2_OCstruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM2_OCstruct.TIM_Pulse=(int)(24242);
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
