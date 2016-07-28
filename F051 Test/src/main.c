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
uint16_t adcval;
char lcdstring[16];
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_ports(void);
void init_adc(void);
void init_nvic(void);
void init_extio(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	adcval=0;
	init_LCD();								// Initialise lcd
	lcd_putstring("EEE3017W Prac 6");		// Display string on line 1
	lcd_command(LINE_TWO);					// Move cursor to line 2
	lcd_putstring("**names**");				// Display string on line 2
	init_ports();
	while(GPIOA->IDR & GPIO_IDR_0){} //waits
	init_adc();
	init_extio();
	init_nvic();
	lcd_command(CLEAR);
	for(;;){
		lcd_command(CURSOR_HOME);
		lcd_putstring("Test");
		GPIOB->ODR=adcval;
		lcd_command(LINE_TWO);
		sprintf(lcdstring,"%d",adcval);
		lcd_putstring(lcdstring);
	};								// Loop forever
}											// End of main
void init_ports(void){
	RCC->AHBENR|=RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN;
	GPIOB->MODER|=(GPIO_MODER_MODER0_0|GPIO_MODER_MODER1_0|
			GPIO_MODER_MODER2_0|GPIO_MODER_MODER3_0|
			GPIO_MODER_MODER4_0|GPIO_MODER_MODER5_0|
			GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|
			GPIO_MODER_MODER8_0|GPIO_MODER_MODER10_0|GPIO_MODER_MODER11_0);
	GPIOA->MODER &= ~(GPIO_MODER_MODER0|GPIO_MODER_MODER1|GPIO_MODER_MODER2|GPIO_MODER_MODER3);
	GPIOA->MODER|=GPIO_MODER_MODER5;
	GPIOA->PUPDR|=(GPIO_PUPDR_PUPDR0_0|GPIO_PUPDR_PUPDR1_0|GPIO_PUPDR_PUPDR2_0|GPIO_PUPDR_PUPDR3_0);
	GPIOB->ODR=(0xff|GPIO_ODR_11);
}
void init_adc(void){
	RCC->APB2ENR|=RCC_APB2ENR_ADCEN;
	ADC1->CHSELR|=ADC_CHSELR_CHSEL5;
	ADC1->CFGR1|=(ADC_CFGR1_RES_1);
	ADC1->IER|=ADC_IER_EOCIE;
	ADC1->CR|=ADC_CR_ADEN;
	while((ADC1->ISR&ADC_ISR_ADRDY)){}
	ADC1->CR|=ADC_CR_ADSTART;
}
void init_nvic(void){
	NVIC_EnableIRQ(ADC1_COMP_IRQn);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}
void ADC1_COMP_IRQHandler(void){
	ADC1->ISR&=~(ADC_ISR_EOC);
	adcval=(ADC1->DR);
}
void init_extio(void){
	RCC->APB2ENR|= RCC_APB2ENR_SYSCFGCOMPEN;
	SYSCFG->EXTICR[1]|=SYSCFG_EXTICR1_EXTI0_PA;
	EXTI->IMR|=EXTI_IMR_MR1;
	EXTI->FTSR|=EXTI_FTSR_TR1;
}
void EXTI0_1_IRQHandler(void){
	EXTI->PR |= EXTI_PR_PR1;
	ADC1->CR|=ADC_CR_ADSTART;
}
//********************************************************************
// END OF PROGRAM
//********************************************************************
