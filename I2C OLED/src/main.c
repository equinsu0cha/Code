//Adapted from https://github.com/thaletterb/STM32-SSD1306-128x64-I2C/blob/master/SSD1306/main.c
//Lib structure for SSD1306 OLED Display with I2C
//I2C1 -> SCL = PB6 AF1
//I2C1 -> SDA = PB7 AF1
//RST -> PB5 GPIO
//********************************************************************
// INCLUDE FILES
//====================================================================
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
#include "stm32f0xx_i2c.h"
//====================================================================
// GLOBAL CONSTANTS
//====================================================================
#define OLED_ADDR					0x3D
#define I2C_TIMEOUT                 100000
//====================================================================
// GLOBAL VARIABLES
//====================================================================

//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_GPIO(void);
void init_I2C(void);

//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_LCD();								// Initialise lcd
	init_GPIO();
	lcd_putstring("SW0");
	init_I2C();
	while(GPIO_ReadInputData(GPIOA)&GPIO_IDR_0);
	lcd_command(CLEAR);					// Move cursor to line 2
	lcd_putstring("**names**");				// Display string on line 2
	for(;;){
		I2C_GenerateSTART(I2C1,ENABLE);
		I2C_SendData(I2C1,0x3D);
		I2C_SendData(I2C1, 0x10);
		while(I2C_GetFlagStatus(I2C1,I2C_FLAG_TC));
		I2C_TransferHandling(I2C1,0x3D,1,I2C_AutoEnd_Mode,I2C_Generate_Start_Write);
	}
}											// End of main

//********************************************************************
// END OF PROGRAM
//********************************************************************
void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB),ENABLE);
	GPIO_InitTypeDef GPIOA_struct,GPIOB_struct;
	//GPIOA PA0-PA3 Inputs
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOA,&GPIOA_struct);
	//GPIOB PB0-PB5 Outputs
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
	GPIOB_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOB,&GPIOB_struct);
}
void init_I2C(void){
	GPIO_InitTypeDef GPIOBI2C_struct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);
	//GPIOB PB6-PB7 I2C SCL/SDA respectively AF1 OPEN DRAIN
	GPIOBI2C_struct.GPIO_Mode=GPIO_Mode_AF;
	GPIOBI2C_struct.GPIO_Pin=(GPIO_Pin_6|GPIO_Pin_7);
	GPIOBI2C_struct.GPIO_OType=GPIO_OType_OD;
	//GPIOBI2C_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOBI2C_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOB,&GPIOBI2C_struct);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_1);

	I2C_InitTypeDef I2C1_struct;
	I2C_StructInit(&I2C1_struct);
	I2C1_struct.I2C_AnalogFilter=I2C_AnalogFilter_Enable;
	I2C1_struct.I2C_DigitalFilter=0x00;
	I2C1_struct.I2C_Mode=I2C_Mode_I2C;
	I2C1_struct.I2C_OwnAddress1=0x39;
	I2C1_struct.I2C_Ack=I2C_Ack_Enable;
	I2C1_struct.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C1_struct.I2C_Timing=0x20310A0D;
	I2C_Init(I2C1,&I2C1_struct);
	I2C_Cmd(I2C1,ENABLE);
}
