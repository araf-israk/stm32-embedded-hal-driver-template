/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Feb 7, 2025
 *      Author: arafi
 */

#include "stm32f411xx.h"
#include <string.h>
#include <stdio.h>

#define SLAVE_ADDR 0x68

uint8_t rxComplt = RESET;

// receive buffer
uint8_t rcv_buf[32];
uint8_t commandcode;
uint8_t len;

void delay(void){
	for(uint32_t i = 0; i < 250000; i++){

	}
}

GPIO_Handle_t GPIOBtn;
I2C_Handle_t I2C1Handle;

/*
 *
 * 	PB6 --> SCL
 * 	PB7 --> SDA
 *
 */

void I2C1_GPIOInits(void){
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void){
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

//void GPIO_LedInit(void){
//	memset(&GpioLed, 0, sizeof(GpioLed));
//
//	GpioLed.pGPIOx = GPIOC;
//	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
//	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
//	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
//	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
//
//	GPIO_Init(&GpioLed);
//}
//
void GPIO_ButtonInit(void){


	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_Init(&GPIOBtn);
}

int main(void){



	I2C1_GPIOInits();
	GPIO_ButtonInit();
	I2C1_Inits();

	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	//I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data), SLAVE_ADDR);

	while(1){
		while((GPIO_ReadFromInputPin(GPIOBtn.pGPIOx, GPIOBtn.GPIO_PinConfig.GPIO_PinNumber)));
		delay();

		commandcode = 0x51;
		while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

		commandcode = 0x52;
		while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY);
		rxComplt = RESET;

		while(rxComplt != SET);
		rxComplt = RESET;
	}
}

void I2C1_EV_IRQHandler(void){
	I2C_EV_IRQHandling(&I2C1Handle);
}
void I2C1_ER_IRQHandler(void){
	I2C_ER_IRQHandling(&I2C1Handle);

}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv){
//	if(AppEv == I2C_EV_TX_CMPLT){
//
//	}else if(AppEv == I2C_EV_RX_CMPLT){
//
//	}else
	if(AppEv == I2C_ERROR_AF){
		I2C_CloseSendData(pI2CHandle);

		I2C_GenerateStopCondition(I2C1);

		while(1);
	}

}

