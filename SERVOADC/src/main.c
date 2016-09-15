//********************************************************************
//*            			SERVO SYSTEM ID Program						 *
//*==================================================================*
// Controls digital servo position with PWM output compare from TIMX
// and reads position info from ADCX Channel X connected to
// Pxx potentiometer on dev board. ADCY Channel Y measures servos
// rotatry position with pannel mount potentiometer connected co-axially
// to digital servo.
// Unit step tests to come later
//====================================================================
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_adc.h"
#include <stdio.h>
//====================================================================
// GLOBAL CONSTANTS
//====================================================================
uint16_t ADC_Buffer[1];
uint32_t servo1;
char lcdstring[16];
int temp=0;
//====================================================================
// GLOBAL VARIABLES
//====================================================================

//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_GPIO(void);
void init_TIM2(void);
void init_ADC(void);
void init_DMA(void);
void init_NVIC(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_LCD();								// Initialise lcd
	init_GPIO();
	lcd_putstring("SERVO ADC TEST");		// Display string on line 1
	while(GPIO_ReadInputData(GPIOA)&GPIO_IDR_0){} //Wait for input
	GPIO_Write(GPIOB,0xff);						  //input ACK
	lcd_command(CLEAR);
	//Finish initialization procedure
	init_TIM2();
	init_ADC();
	init_DMA();
	init_NVIC();
	for(;;){
		lcd_command(CURSOR_HOME);
		temp = ADC_Buffer[0];
		servo1=25800;//(12000+(temp/4095.0)*26800); //Equation for servo, 0 degree offset at 25400
		sprintf(lcdstring,"ADC1:%d    ",(int) servo1);
		lcd_putstring(lcdstring);
		TIM_SetCompare3(TIM2,servo1);
		lcd_command(LINE_TWO);
		temp = ADC_Buffer[1];
		servo1=(2.5*(temp/4095.0)+0.5)*(1/3.03)*48484;
		sprintf(lcdstring,"ADC2:%d    ",(int) servo1);
		lcd_putstring(lcdstring);
		TIM_SetCompare4(TIM2,servo1);
	}
}											// End of main

//********************************************************************
// END OF PROGRAM
//********************************************************************
void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB),ENABLE);
	GPIO_InitTypeDef GPIOA_struct,GPIOB_struct;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIOA_struct);
	//PA5 & PA6 -> ADC IN5 & ADC IN6 (ANALOGUE MODE, PA6 needs Pull Up resistor)
	GPIO_InitTypeDef GPIOA_ADC5_struct,GPIOA_ADC6_struct;
	GPIOA_ADC5_struct.GPIO_Pin=(GPIO_Pin_5);
	GPIOA_ADC5_struct.GPIO_Mode=GPIO_Mode_AN;
	GPIOA_ADC5_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIOA_ADC5_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_ADC5_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIOA_ADC5_struct);
	GPIOA_ADC6_struct.GPIO_Pin=(GPIO_Pin_6);
	GPIOA_ADC6_struct.GPIO_Mode=GPIO_Mode_AN;
	GPIOA_ADC6_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIOA_ADC6_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_ADC6_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIOA_ADC6_struct);
	//GPIOB Initialize
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3
			|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIOB_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIOB_struct);
	//PB10 & PB11 -> TIM2CH3 & TIM2CH4 (both AF2) respectively for output compare
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
	TIM2_basestruct.TIM_Period=48484;
	TIM2_basestruct.TIM_Prescaler=2;
	TIM2_basestruct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM2_basestruct);
	//OC PWM MODE
	TIM_OCInitTypeDef TIM2_OCstruct={0,};
	TIM2_OCstruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM2_OCstruct.TIM_Pulse=(uint32_t) (25100);
	TIM2_OCstruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM2_OCstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OC3Init(TIM2,&TIM2_OCstruct);
	TIM_OC4Init(TIM2,&TIM2_OCstruct);
	//Start TIMER2
	TIM_Cmd(TIM2,ENABLE);

}
void init_ADC(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	ADC_DeInit(ADC1);
	ADC_InitTypeDef ADC_struct;
	ADC_StructInit(&ADC_struct);
	ADC_struct.ADC_Resolution=ADC_Resolution_12b;
	ADC_struct.ADC_ContinuousConvMode=ENABLE;
	ADC_struct.ADC_ExternalTrigConv=ADC_ExternalTrigConvEdge_None;
	ADC_struct.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_struct.ADC_ScanDirection=ADC_ScanDirection_Upward;
	ADC_Init(ADC1,&ADC_struct);
	ADC_ChannelConfig(ADC1,ADC_Channel_5,ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1,ADC_Channel_6,ADC_SampleTime_55_5Cycles);
	ADC_DMARequestModeConfig(ADC1,ADC_DMAMode_Circular);
	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1,ENABLE);
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_ADRDY)){}
	ADC_StartOfConversion(ADC1);
}
void init_DMA(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,1);
	DMA1_Channel1->CNDTR=0x2;
	DMA1_Channel1->CPAR=(uint32_t) &(ADC1->DR);
	DMA1_Channel1->CMAR=(uint32_t) &(ADC_Buffer[0]);
	DMA1_Channel1->CCR=(DMA_M2M_Disable|DMA_Priority_VeryHigh|DMA_MemoryDataSize_HalfWord|DMA_PeripheralDataSize_HalfWord
			|DMA_MemoryInc_Enable|DMA_PeripheralInc_Disable|DMA_Mode_Circular|DMA_DIR_PeripheralSRC|DMA_CCR_EN);
}
void init_NVIC(void){
	//NVIC_InitTypeDef TIM2_NVIC;
	//TIM2_NVIC.NVIC_IRQChannel=TIM2_IRQn;
	//TIM2_NVIC.NVIC_IRQChannelPriority=1;
	//TIM2_NVIC.NVIC_IRQChannelCmd=ENABLE;
	//NVIC_Init(&TIM2_NVIC);
}
