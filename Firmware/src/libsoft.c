/*
 * firmware.c
 *
 * For STM32F051 Test software
 *      Author: Nick
 */
#include "libsoft.h"
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_rcc.h"

void init_RC(void){
	//USART2 Module for S.BUS RX
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
}
void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOA),ENABLE);
	GPIO_InitTypeDef GPIOA_struct,GPIOB_struct;
	//PA0-PA3 Input
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3);
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOA,&GPIOA_struct);
	//PB0-PB7 Output
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOB,&GPIOB_struct);
	GPIO_Write(GPIOB,0xff);
}
void init_Servos(servo SERVOx){
	SERVO_pos SERVOx_pos=SERVOx->SERVO_loc;
	switch(SERVOx_pos){
	case SERVOx_pos==0x00:

	}
	//Init TIM OC PWM Mode for Servos

}
void set_Servos(servo SERVOx){

	//Write to servo
}
