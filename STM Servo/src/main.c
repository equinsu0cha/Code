//********************************************************************
//*                    SERVO PWM TIMER OUTPUT                        *
//*                    LCD test                                      *
//*==================================================================*
// PWM output compare for 330 Hz digital servo
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
void init_tim2(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_LCD();								// Initialise lcd
	lcd_putstring("Testing");		// Display string on line 1
	init_ports();
	init_adc(); //Init ADC
	init_tim2();//init TIM14
	while(GPIOA->IDR & GPIO_IDR_0){} //waits
	lcd_command(CLEAR);
	for(;;){
		adcval= ADC1->DR;
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
	GPIOB->MODER|=(GPIO_MODER_MODER11_1|GPIO_MODER_MODER10_1);
	GPIOB->AFR[1]|=(0b0010<<12)|(0b0010<<8);
	GPIOA->MODER &= ~(GPIO_MODER_MODER0|GPIO_MODER_MODER1|GPIO_MODER_MODER2|GPIO_MODER_MODER3); //PA0-3 Inputs
	GPIOA->MODER|=GPIO_MODER_MODER5;//PA5 Analog mode input
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
void init_tim2(void){
	RCC->APB1ENR|= RCC_APB1ENR_TIM2EN;
	TIM2->PSC=3;
	TIM2->ARR=48485;
	TIM2->CCMR2|=(TIM_CCMR2_OC4M_2|TIM_CCMR2_OC4M_1|TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3M_1);
	TIM2->CCR4=14692*(0.7+1.3*(127.5/255.0));
	TIM2->CCR3=14692*(0.7+1.3*(127.5/255.0));
	TIM2->CCER|= (TIM_CCER_CC4E|TIM_CCER_CC3E);
	TIM2->CR1|=TIM_CR1_CEN;
}
//********************************************************************
// END OF PROGRAM
//********************************************************************
