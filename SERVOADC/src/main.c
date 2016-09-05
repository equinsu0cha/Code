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
#include <stdio.h>
//====================================================================
// GLOBAL CONSTANTS
//====================================================================
char lcdstring[16];
int count=0;
//====================================================================
// GLOBAL VARIABLES
//====================================================================

//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_GPIO(void);
void init_TIM2(void);
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
	//Finish initialization procedure
	init_TIM2();
	init_NVIC();
	for(;;){
		lcd_command(LINE_TWO);
		sprintf(lcdstring,"TIME:%d",count);
		lcd_putstring(lcdstring);
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
	GPIO_PinAFConfig(GPIOB,(GPIO_PinSource10|GPIO_PinSource11),GPIO_AF_2);
}

void init_TIM2(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_TimeBaseInitTypeDef TIM2_basestruct;
	TIM2_basestruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM2_basestruct.TIM_ClockDivision= 0;
	TIM2_basestruct.TIM_Period=48485;
	TIM2_basestruct.TIM_Prescaler=3;
	TIM2_basestruct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM2_basestruct);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
}

void init_NVIC(void){
	NVIC_InitTypeDef TIM2_NVIC;
	TIM2_NVIC.NVIC_IRQChannel=TIM2_IRQn;
	TIM2_NVIC.NVIC_IRQChannelPriority=1;
	TIM2_NVIC.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&TIM2_NVIC);
}
void TIM2_IRQHandler(void){
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	count++;
}
