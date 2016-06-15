//********************************************************************
// INCLUDE FILES
//====================================================================
#include <stdio.h>
#include <stdlib.h>
#include <stm32f0xx_adc.h>
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"

//====================================================================
// GLOBAL CONSTANTS
//====================================================================
GPIO_InitTypeDef GPIOB_init_struct, GPIOA_init_struct;
ADC_InitTypeDef ADC1_init_structure;
//====================================================================
// GLOBAL VARIABLES
//====================================================================
const uint16_t GPIOB_PINS= GPIO_Pin_0|GPIO_Pin_1;
volatile uint8_t ADC_val;
char disp[16];
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================


//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_LCD();
	lcd_putstring("Test");
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
	GPIOB_init_struct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIOB_init_struct.GPIO_Mode = GPIO_Mode_OUT;
	GPIOB_init_struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIOB_init_struct);
	GPIO_SetBits(GPIOB, GPIOB_PINS);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	GPIOA_init_struct.GPIO_PuPd = GPIO_Pin_5;
	GPIOA_init_struct.GPIO_Mode = GPIO_Mode_AN;
	GPIOA_init_struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIOA_init_struct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	ADC1_init_structure.ADC_Resolution = ADC_Resolution_8b;
	ADC1_init_structure.ADC_ContinuousConvMode=ENABLE;
	ADC1_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_Init(ADC1, &ADC1_init_structure);
	//ADC_Init(ADC1, &ADC1_init_structure);

	ADC_ChannelConfig(ADC1,ADC_Channel_5, ADC_SampleTime_239_5Cycles);
	ADC_Cmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));
	ADC_StartOfConversion(ADC1);

	for(;;);
		while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
		ADC_C
		ADC_val = ADC_GetConversionValue(ADC1);
		lcd_command(CURSOR_HOME);
		sprintf(disp,"%d",ADC_val);
		lcd_putstring(disp);
}

//********************************************************************
// END OF PROGRAM
//********************************************************************
