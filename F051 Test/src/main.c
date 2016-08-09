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
uint16_t adcval=0,timeval=0;
_Bool bit;
char lcdstring[16];
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_ports(void);
void init_adc(void);
void init_nvic(void);
void init_extio(void);
void init_tim14(void);
void TIM14_IRQHandler(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_LCD();								// Initialise lcd
	lcd_putstring("Testing");		// Display string on line 1
	init_ports();
	init_adc(); //Init ADC
	init_extio();//Extio init
	init_tim14();//init TIM14
	while(GPIOA->IDR & GPIO_IDR_0){} //waits
	init_nvic();
	lcd_command(CLEAR);
	for(;;){
		adcval=ADC1->DR; //read from ADC
		lcd_command(CURSOR_HOME);
		sprintf(lcdstring,"ADC: %d",adcval);
		lcd_putstring(lcdstring);
		lcd_command(LINE_TWO);
		int temp = (RCC->CFGR);
		sprintf(lcdstring,"RCC: %d",temp);
		lcd_putstring(lcdstring);
		TIM14->CCR1= 14692*(0.7+1.3*(adcval/255.0));
	}
}

void init_ports(void){
	RCC->AHBENR|=RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOBEN;			//enable GPIOA & GPIOB Clocks
	GPIOB->MODER|=(GPIO_MODER_MODER0_0|GPIO_MODER_MODER1_0|
			GPIO_MODER_MODER2_0|GPIO_MODER_MODER3_0|
			GPIO_MODER_MODER4_0|GPIO_MODER_MODER5_0|
			GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0); //set GPIOB0-7 as outputs
	GPIOA->MODER &= ~(GPIO_MODER_MODER0|GPIO_MODER_MODER1|GPIO_MODER_MODER2|GPIO_MODER_MODER3); //PA0-3 Inputs
	GPIOA->MODER|=GPIO_MODER_MODER5;//PA5 Analog mode input
	GPIOA->MODER|=(GPIO_MODER_MODER7_1); //PA7 Alternate Function
	GPIOA->AFR[0]|=(0b0100<<28); //PA7 AF4
	GPIOA->PUPDR|=(GPIO_PUPDR_PUPDR0_0|GPIO_PUPDR_PUPDR1_0|GPIO_PUPDR_PUPDR2_0|GPIO_PUPDR_PUPDR3_0);//Pull ups for PA0-PA3
}

void init_adc(void){ //Init ADC Continuous conversion, not interrupts
	RCC->APB2ENR|=RCC_APB2ENR_ADCEN; //Clock ADC
	ADC1->CHSELR|=ADC_CHSELR_CHSEL5; //Select CH5, PA5 -> ADC In5
	ADC1->CFGR1|=(ADC_CFGR1_RES_1|ADC_CFGR1_CONT); //8Bit res & Continuous
	ADC1->CR|=ADC_CR_ADEN; //Enable ADC
	while((ADC1->ISR&ADC_ISR_ADRDY)){} //RDY Flag
	ADC1->CR|=ADC_CR_ADSTART; //Starts continuous conversion
}
void init_nvic(void){
	NVIC_EnableIRQ(EXTI0_1_IRQn); //Enables IRQ for EXTI 0 & 1
}
void init_extio(void){	//Init EXTI-0
	RCC->APB2ENR|= RCC_APB2ENR_SYSCFGCOMPEN; //Clocks cfg comp
	SYSCFG->EXTICR[1]|=SYSCFG_EXTICR1_EXTI1_PA; //Sets Control Register1 for EXTI1 PA1
	EXTI->IMR|=EXTI_IMR_MR1; //Unmasks input
	EXTI->FTSR|=EXTI_FTSR_TR1; //sets for falling edge
}
void EXTI0_1_IRQHandler(void){
	EXTI->PR |= EXTI_PR_PR1;
	//SW1 IRQ Handler
}
void init_tim14(void){
	RCC->APB1ENR|= RCC_APB1ENR_TIM14EN;
	TIM14->PSC=3;
	TIM14->ARR=48485;
	TIM14->CCMR1|=(TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1);
	TIM14->CCR1=14692*(0.7+1.3*(127.5/255.0));
	TIM14->CCER|= (TIM_CCER_CC1E);
	TIM14->CR1|=TIM_CR1_CEN;
}
void TIM14_IRQHandler(void){
	TIM14->SR&=~(TIM_SR_UIF);
}
//********************************************************************
// END OF PROGRAM
//********************************************************************
