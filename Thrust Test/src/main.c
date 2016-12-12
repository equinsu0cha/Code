//********************************************************************
//*                    Thrust Test Code                              *
//====================================================================
// Strain guage configured with g_eff = 10000. Full scale deflection
// approx 3V. Sampled with ADC1CH5. TIM2CH3 O/C PWM drives BLDC speed
// controller at 20Hz. BLDC rpm input signal  measured with TIM3CC1
// input capture at 10 MHZ shooting and I/C event capture division = 4
// USART1 TX data packets schedules from TIM14 at 200 HZ.
//====================================================================
// INCLUDES
//====================================================================
#include "stdio.h"
#include "math.h"
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_adc.h"
//====================================================================
// GLOBAL CONSTANTS
//====================================================================

//====================================================================
// GLOBAL VARIABLES
//====================================================================
char lcdstring[16],usart_char[16];
int temp,sysclock,ADCval,tim2_count=3200;
uint32_t DutyCycle,Frequency1,TIM3_CCR1_tick=0,CCR1_comp=0;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_GPIO(void);
void init_ADC(void);
void init_TIM2(void);
void init_TIM3(void);
void init_TIM14(void);
void init_USART1(void);
void send_packet(const char *str);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_LCD();
	init_GPIO();
	//Get operating CLK freq
	RCC_ClocksTypeDef CLK;
	RCC_GetClocksFreq(&CLK);
	sysclock= (CLK.SYSCLK_Frequency)/((float)(pow(10,6)));
	//Sys clocks in MHz
	sprintf(lcdstring,"%d MHz",sysclock);
	lcd_putstring(lcdstring);
	lcd_command(LINE_TWO);
	lcd_putstring("Thrust Test, SW0");
	//Init procedure waits for SW0
	while(GPIO_ReadInputData(GPIOA)&GPIO_IDR_0){}
	lcd_command(CLEAR);
	init_USART1();
	init_ADC();
	init_TIM2();
	init_TIM3();
	init_TIM14();
	for(;;){
		lcd_command(CURSOR_HOME);
		sprintf(lcdstring,"Freq:%d",(int)Frequency1);
		lcd_putstring(lcdstring);
		lcd_command(LINE_TWO);
		sprintf(lcdstring,"ADC:%d",(int)ADCval);
		lcd_putstring(lcdstring);
	}
}											// End of main
void init_GPIO(void){
	RCC_AHBPeriphClockCmd((RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB),ENABLE);
	//PA0-PA2 Inputs
	GPIO_InitTypeDef GPIOA_struct;
	GPIOA_struct.GPIO_Mode=GPIO_Mode_IN;
	GPIOA_struct.GPIO_OType=GPIO_OType_PP;
	GPIOA_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2);
	GPIOA_struct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOA,&GPIOA_struct);
	// PA5 ADC1IN5
	GPIO_InitTypeDef GPIOA_ADC;
	GPIOA_ADC.GPIO_Mode=GPIO_Mode_AN;
	GPIOA_ADC.GPIO_OType=GPIO_OType_PP;
	GPIOA_ADC.GPIO_Pin=GPIO_Pin_5;
	GPIOA_ADC.GPIO_PuPd=GPIO_PuPd_UP;
	GPIOA_ADC.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOA_ADC);
	//GPIOA_TIM3 PA6 TIM3CH1 Input AF1
	GPIO_InitTypeDef GPIOA_TIM3;
	GPIOA_TIM3.GPIO_Mode=GPIO_Mode_AF;
	GPIOA_TIM3.GPIO_OType=GPIO_OType_PP;
	GPIOA_TIM3.GPIO_Pin=(GPIO_Pin_6);
	GPIOA_TIM3.GPIO_PuPd=GPIO_PuPd_DOWN;//GPIO_PuPd_DOWN;
	GPIOA_TIM3.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOA_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_1);
	//PA9-PA10 USART RX/TX
	GPIO_InitTypeDef GPIOAUSART;
	GPIOAUSART.GPIO_Mode=GPIO_Mode_AF;
	GPIOAUSART.GPIO_OType=GPIO_OType_PP;
	GPIOAUSART.GPIO_Pin=(GPIO_Pin_9|GPIO_Pin_10);
	GPIOAUSART.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOAUSART.GPIO_Speed=GPIO_Speed_Level_3;
	GPIO_Init(GPIOA,&GPIOAUSART);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);
	//PB0-PB7 Outputs
	GPIO_InitTypeDef GPIOB_struct;
	GPIOB_struct.GPIO_Mode=GPIO_Mode_OUT;
	GPIOB_struct.GPIO_OType=GPIO_OType_PP;
	GPIOB_struct.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIOB_struct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_struct.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOB,&GPIOB_struct);
	GPIO_Write(GPIOB,0xff);
	//PB10 TIM2CH3 for O/C PWM output
	GPIO_InitTypeDef GPIOB_TIM2;
	GPIOB_TIM2.GPIO_Mode=GPIO_Mode_AF;
	GPIOB_TIM2.GPIO_OType=GPIO_OType_PP;
	GPIOB_TIM2.GPIO_Pin=(GPIO_Pin_10);
	GPIOB_TIM2.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIOB_TIM2.GPIO_Speed=GPIO_Speed_Level_2;
	GPIO_Init(GPIOB,&GPIOB_TIM2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_2);
}
void init_ADC(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	ADC_InitTypeDef ADC_struct;
	ADC_struct.ADC_ContinuousConvMode=ENABLE;
	ADC_struct.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_struct.ADC_ExternalTrigConv=ADC_ExternalTrigConvEdge_None;
	ADC_struct.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
	ADC_struct.ADC_Resolution=ADC_Resolution_12b;
	ADC_struct.ADC_ScanDirection=ADC_ScanDirection_Upward;
	ADC_Init(ADC1,&ADC_struct);
	ADC_ChannelConfig(ADC1,ADC_Channel_5,ADC_SampleTime_55_5Cycles);
	ADC_Cmd(ADC1,ENABLE);
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_ADRDY)){}
	ADC_StartOfConversion(ADC1);
}
void init_USART1(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	USART_InitTypeDef USART1_struct;
	USART1_struct.USART_BaudRate=115200;
	USART1_struct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART1_struct.USART_Mode=(USART_Mode_Rx|USART_Mode_Tx);
	USART1_struct.USART_Parity=USART_Parity_No;
	USART1_struct.USART_StopBits=USART_StopBits_2;
	USART1_struct.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART1,&USART1_struct);
	USART_Cmd(USART1,ENABLE);
}
void init_TIM2(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_DeInit(TIM2);
	TIM_TimeBaseInitTypeDef TIM2_struct;
	TIM2_struct.TIM_ClockDivision=0;
	TIM2_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM2_struct.TIM_Period=64000;
	TIM2_struct.TIM_Prescaler=14;
	TIM2_struct.TIM_RepetitionCounter=0;
	// TIM2 OC base clock at 50 HZ
	TIM_TimeBaseInit(TIM2,&TIM2_struct);
	TIM_OCInitTypeDef TIM2_OCstruct={0,};
	TIM2_OCstruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM2_OCstruct.TIM_Pulse=0;
	TIM2_OCstruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM2_OCstruct.TIM_OCPolarity=TIM_OCPolarity_High;
	//Init OC3 output at 0% Duty Cycle for testing
	TIM_OC3Init(TIM2,&TIM2_OCstruct);
	//TIM_OC4Init(TIM2,&TIM2_OCstruct);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	NVIC_InitTypeDef NVIC_TIM2;
	NVIC_TIM2.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_TIM2.NVIC_IRQChannelPriority=1;
	NVIC_TIM2.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM2);
	TIM_Cmd(TIM2,ENABLE);
}
void init_TIM3(void){
	//TIM3 for CH1 PWM INPUT CAPTURE
	int prescaler;		//Calcs for PSC to get 10MHZ
	TIM_TimeBaseInitTypeDef TIM3_struct;
	TIM_ICInitTypeDef TIM3IC_struct;
	NVIC_InitTypeDef NVIC_TIM3;
	//
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM3_struct.TIM_ClockDivision=0x0;
	TIM3_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM3_struct.TIM_Prescaler=0;
	TIM3_struct.TIM_Period=65535;
	//Set base structure
	TIM_TimeBaseInit(TIM3,&TIM3_struct);
	prescaler = (sysclock/1)-1;		//10 Mhz shooting
	TIM_PrescalerConfig(TIM3,prescaler,TIM_PSCReloadMode_Immediate);
	//Input capture config CH1 only
	TIM3IC_struct.TIM_Channel=(TIM_Channel_1);
	TIM3IC_struct.TIM_ICFilter=0x00;
	TIM3IC_struct.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM3IC_struct.TIM_ICPrescaler=TIM_ICPSC_DIV8;
	TIM3IC_struct.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM3,&TIM3IC_struct);
	//Interrupts
	TIM_ITConfig(TIM3,(TIM_IT_Update|TIM_IT_CC1),ENABLE);
	//Enable TIM3
	TIM_Cmd(TIM3,ENABLE);
	//Unmask global interrupt for TM3
	NVIC_TIM3.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_TIM3.NVIC_IRQChannelPriority=1;
	NVIC_TIM3.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM3);
}
void init_TIM14(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	TIM_TimeBaseInitTypeDef TIM14_struct;
	TIM14_struct.TIM_ClockDivision=0x0;
	TIM14_struct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM14_struct.TIM_Period=60000;
	TIM14_struct.TIM_Prescaler=3;
	TIM14_struct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM14,&TIM14_struct);
	TIM_ITConfig(TIM14,TIM_IT_Update,ENABLE);
	NVIC_InitTypeDef NVIC_TIM14;
	NVIC_TIM14.NVIC_IRQChannel=TIM14_IRQn;
	NVIC_TIM14.NVIC_IRQChannelPriority=1;
	NVIC_TIM14.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM14);
	TIM_Cmd(TIM14,ENABLE);
}
void TIM2_IRQHandler(void){
	tim2_count++;
	if(tim2_count<=6400){
		TIM_SetCompare3(TIM2,tim2_count);
	}
	else{
		tim2_count=3100;
	}
	ADCval=ADC_GetConversionValue(ADC1);
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
}
void TIM3_IRQHandler(void){
	uint32_t deltatick1,temp1;
	if(TIM_GetFlagStatus(TIM3,TIM_IT_Update)){
		TIM3_CCR1_tick++;
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
	if(TIM_GetITStatus(TIM3,TIM_IT_CC1)&(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6))){
		temp1 = TIM_GetCapture1(TIM3);
		deltatick1 = temp1 + TIM3_CCR1_tick*(TIM3->ARR) - CCR1_comp;
		Frequency1 = (float) (8*(pow(10,6)*sysclock/(TIM3->PSC+1))/(deltatick1));
		TIM3_CCR1_tick=0;
		CCR1_comp=temp1;
		TIM_ClearITPendingBit(TIM3,TIM_IT_CC1);
	}
}
void TIM14_IRQHandler(void){
	sprintf(usart_char,"%d\n", 240);
	send_packet(usart_char);
	sprintf(usart_char,"%d\n", tim2_count);
	send_packet(usart_char);
	sprintf(usart_char,"%d\n", (int) (TIM2->CCR3));
	send_packet(usart_char);
	sprintf(usart_char,"%d\n",(int) (ADCval));
	send_packet(usart_char);
	sprintf(usart_char,"%d\n",(int) (Frequency1/7.0));
	send_packet(usart_char);
	TIM_ClearITPendingBit(TIM14,TIM_IT_Update);
}
void send_packet(const char *str){
	while(*str){
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0);
		USART_SendData(USART1,*str++);
	}
}
//********************************************************************
// END OF PROGRAM
//********************************************************************
