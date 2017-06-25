
#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
#include "stdio.h"
#include "stm32f0xx_TIM.h"
char lcdstring[16];
int tmr=0;

void init_GPIO(void);
void init_TIM3(void);
void main(void){
	init_LCD();
	lcd_putstring("Test");
	init_GPIO();
	while(GPIO_ReadInputData(GPIOA)&GPIO_IDR_0){}
	lcd_command(CLEAR);
	GPIO_Write(GPIOB,0xff);
	init_TIM3();
	for(;;){
		lcd_command(CURSOR_HOME);
		sprintf(lcdstring,"%d",tmr);
		lcd_putstring(lcdstring);
	}
}
void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBENR_GPIOBEN),ENABLE);
	GPIO_InitTypeDef GPIOA_struct,GPIOB_struct;
	//PA0-PA1 inputs
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1);
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOA_struct);
	//PB0-PB7 Outputs
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_struct.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOB,&GPIOB_struct);
	GPIO_Write(GPIOB,0x00);
}
void init_TIM3(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_TimeBaseInitTypeDef TIM3_struct;
	TIM3_struct.TIM_ClockDivision=0;
	TIM3_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM3_struct.TIM_Period=64000;
	TIM3_struct.TIM_Prescaler=14;
	TIM3_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM3,&TIM3_struct);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	NVIC_InitTypeDef TIM3_NVIC;
	TIM3_NVIC.NVIC_IRQChannel=TIM3_IRQn;
	TIM3_NVIC.NVIC_IRQChannelPriority=0;
	TIM3_NVIC.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&TIM3_NVIC);
	TIM_Cmd(TIM3,ENABLE);
}
void TIM3_IRQHanlder(void){
	tmr++;
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}
