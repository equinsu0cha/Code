#include <stdint.h>

#include "stm32f0xx.h"



void main(void);

void init_leds(void);



void main(void)

{

            volatile uint32_t delay_counter = 0;

init_leds();



            for(;;) {

                        for(delay_counter = 0; delay_counter < 655350; delay_counter++);

                        GPIOB->ODR = 0xAA;

for(delay_counter = 0; delay_counter < 655350; delay_counter++);

GPIOB->ODR = 0x55;

            }



}



void init_leds(void) {

            RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable clock for LEDs

            GPIOB->MODER |= GPIO_MODER_MODER0_0; //set PB0 to output

            GPIOB->MODER |= GPIO_MODER_MODER1_0; //set PB1 to output

            GPIOB->MODER |= GPIO_MODER_MODER2_0; //set PB2 to output

            GPIOB->MODER |= GPIO_MODER_MODER3_0; //set PB3 to output

            GPIOB->MODER |= GPIO_MODER_MODER4_0; //set PB4 to output

            GPIOB->MODER |= GPIO_MODER_MODER5_0; //set PB5 to output

            GPIOB->MODER |= GPIO_MODER_MODER6_0; //set PB6 to output

            GPIOB->MODER |= GPIO_MODER_MODER7_0; //set PB7 to output

}
