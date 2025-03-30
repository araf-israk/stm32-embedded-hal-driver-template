/*
 * 001led_toggle.c
 *
 *  Created on: Jan 24, 2025
 *      Author: arafi
 */

#include "stm32f411xx.h"
#include "stm32f411xx_gpio.h"


void delay(void){
	for(uint32_t i = 0; i < 2000000; i++){

	}
}

int main(void){

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLed);

	while(1){
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
		delay();
	}

	return 0;
}
