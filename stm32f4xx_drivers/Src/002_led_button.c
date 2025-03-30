/*
 * 002_led_button.c
 *
 *  Created on: Jan 24, 2025
 *      Author: arafi
 */


#include "stm32f411xx.h"
#include "stm32f411xx_gpio.h"


void delay(void){
	for(uint32_t i = 0; i < 200000; i++){

	}
}

int main(void){

	GPIO_Handle_t GpioLed, GPIOBtn;

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioLed);

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIOBtn);

	while(1){

		if(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)){
			delay();
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
		}
	}

	return 0;
}
