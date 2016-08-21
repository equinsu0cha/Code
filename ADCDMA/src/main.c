//********************************************************************
//*                    EEE3017W C template                           *
//*                    LCD test                                      *
//*==================================================================*
//* WRITTEN BY:    	                 		                         *
//* DATE CREATED:                                                    *
//* MODIFIED:                                                        *
//*==================================================================*
//* PROGRAMMED IN: Eclipse Luna Service Release 1 (4.4.1)            *
//* DEV. BOARD:    UCT STM32 Development Board                       *
//*==================================================================*
//* DESCRIPTION:                                                     *
//*                                                                  *
//********************************************************************
// INCLUDE FILES
//====================================================================
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
#include <stdio.h>
//====================================================================
// GLOBAL CONSTANTS
//====================================================================

//====================================================================
// GLOBAL VARIABLES
//====================================================================
uint8_t ADC_Buffer[4];
int temp;
char lcdstring[16];
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_ports(void);
void init_adc(void);
void init_DMA(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_LCD();								// Initialise lcd
	init_ports();
	lcd_putstring("test");		// Display string on line 1
	while(GPIOA->IDR & GPIO_IDR_0);
	lcd_command(CLEAR);
	lcd_putstring("Init ADC/DMA");
	init_adc();
	init_DMA();
	for(;;){
		lcd_command(LINE_TWO);
		temp = (DMA_GetCurrDataCounter(DMA1_Channel1));
		sprintf(lcdstring,"%d",temp);
		lcd_putstring(lcdstring);
	}
}											// End of main

void init_ports(void){
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOAEN,1);
	GPIO_InitTypeDef GPIOA_struct;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIOA_struct);
	GPIO_InitTypeDef GPIOA_adc;
	GPIOA_adc.GPIO_Pin=(GPIO_Pin_5);
	GPIOA_adc.GPIO_Mode=GPIO_Mode_AN;
	GPIOA_adc.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIOA_adc);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,1);
	GPIO_InitTypeDef GPIOB_struct;
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIOB_struct);
	GPIO_Write(GPIOB,0xff);
}

void init_adc(void){
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN,1);
	ADC_DeInit(ADC1);
	ADC_InitTypeDef ADC_struct;
	ADC_StructInit(&ADC_struct);
	ADC_struct.ADC_Resolution=ADC_Resolution_8b;
	ADC_struct.ADC_ContinuousConvMode=ENABLE;
	ADC_struct.ADC_ExternalTrigConv=ADC_ExternalTrigConvEdge_None;
	ADC_struct.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_struct.ADC_ScanDirection=ADC_ScanDirection_Backward;
	ADC_Init(ADC1,&ADC_struct);
	ADC_ChannelConfig(ADC1,ADC_Channel_5,ADC_SampleTime_55_5Cycles);
	ADC_DMARequestModeConfig(ADC1,ADC_DMAMode_Circular);
	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1,ENABLE);
	while(!(ADC_GetFlagStatus(ADC1,ADC_FLAG_ADRDY)));
	ADC_StartOfConversion(ADC1);
}
void init_DMA(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,1);
	/*DMA1_Channel1->CNDTR=0x4;
	DMA1_Channel1->CPAR=(uint32_t) &(ADC1->DR);
	DMA1_Channel1->CMAR=(uint32_t) &(ADC_Buffer[0]);
	DMA1_Channel1->CCR=(DMA_M2M_Disable|DMA_Priority_VeryHigh|DMA_MemoryDataSize_Byte|DMA_PeripheralDataSize_Byte
				|DMA_MemoryInc_Enable|DMA_PeripheralInc_Disable|DMA_Mode_Circular|DMA_DIR_PeripheralSRC|DMA_CCR_EN);
				*/
    DMA_DeInit(DMA1_Channel1);
	DMA_InitTypeDef DMA_struct;
	DMA_StructInit(&DMA_struct);
	lcd_putstring(DMA_struct.DMA_BufferSize);
	/*
	DMA_struct.DMA_PeripheralBaseAddr=(uint32_t) &ADC1->DR;
	DMA_struct.DMA_MemoryBaseAddr=(uint32_t) &ADC_Buffer[0];
	DMA_struct.DMA_DIR=DMA_DIR_PeripheralSRC;
	DMA_struct.DMA_BufferSize=0x4;
	DMA_struct.DMA_PeripheralInc=DISABLE;
	DMA_struct.DMA_MemoryInc=ENABLE;
	DMA_struct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
	DMA_struct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	DMA_struct.DMA_Mode=DMA_Mode_Circular;
	DMA_struct.DMA_Priority=DMA_Priority_High;
	DMA_struct.DMA_M2M=DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1,&DMA_struct);
	lcd_putstring("HERE");
	DMA_Cmd(DMA1_Channel1,ENABLE);*/
}
//********************************************************************
// END OF PROGRAM
//********************************************************************
