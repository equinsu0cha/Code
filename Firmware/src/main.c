//********************************************************************
//*					STM32F051C Test Firmware						 *
//*==================================================================*
//* Starting of firmware.h/firmware.c lib
//
//====================================================================
#include "libsoft.h"
#include "lcd_stm32f0.h"
#include "stdio.h"
#include "math.h"
#include "stm32f0xx.h"
//====================================================================
// GLOBAL CONSTANTS
//====================================================================
char lcd_string[16];
uint32_t temp;
//====================================================================
// GLOBAL VARIABLES
//====================================================================
RCC_ClocksTypeDef sysclocks;
servo Servo1;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================


//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	RCC_GetClocksFreq(&sysclocks);
	//Init functions
	init_LCD();								// Initialise lcd
	init_GPIO();
	//
	lcd_putstring("Firmware Test");		// Display string on line 1
	lcd_command(LINE_TWO);					// Move cursor to line 2
	sprintf(lcd_string,"%d MHz, SW0",(int)(sysclocks.SYSCLK_Frequency/(pow(10,6))));
	lcd_putstring(lcd_string);				// Display string on line 2
	while(GPIO_ReadInputData(GPIOA)&GPIO_IDR_0){}
	lcd_command(CLEAR);
	Servo1.SERVO_num=PA0;
	for(;;);								// Loop forever
}											// End of main

//********************************************************************
// END OF PROGRAM
//********************************************************************
