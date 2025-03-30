/*
 * stm32411xx_spi_driver.c
 *
 *  Created on: Jan 25, 2025
 *      Author: arafi
 */

#include "stm32f411xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/**********************************************************
 * @fn			- SPI_PeriClockControl
 *
 * @brief		-
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}
		else if(pSPIx == SPI5){
			SPI5_PCLK_EN();
		}
	}
	else{

	}

}


/**********************************************************
 * @fn			- SPI_Init
 *
 * @brief		-
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_Init(SPI_Handle_t *pSPIHandle){

	/* peripheral clock enable*/
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/*first configure the SPI_CR1 register*/

	uint32_t tempreg = 0;
	/*Configure the device mode*/
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;

	/*Configure the bus*/
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		/* 1 line bi-di mode should be cleared*/
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		/* 1 line bi-di mode should be set*/
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		/* 1 line bi-di mode should be cleared*/
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		/* RXONLY bit must be set*/
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	/* Configure the spi serial clock speed (baud rate) */
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	/* Configure the DFF */
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	/* Configure the CPOL */
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	/* Configure the CPHA*/
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	/* Configure the SSM */
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/**********************************************************
 * @fn			- SPI_DeInit
 *
 * @brief		-
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx){

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**********************************************************
 * @fn			- SPI_SendData
 *
 * @brief		-
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0){
		/* Wait until TXE is set*/
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		/* Check the DFF bit in CR1*/
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF))){
			/* 16 bit DFF */
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else{
			/* 8 bit DFF */
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/**********************************************************
 * @fn			- SPI_PeripheralControl
 *
 * @brief		-
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/**********************************************************
 * @fn			- SPI_SSIConfig
 *
 * @brief		-
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/**********************************************************
 * @fn			- SPI_SSOEConfig
 *
 * @brief		-
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/**********************************************************
 * @fn			- SPI_ReceiveData
 *
 * @brief		-
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	while(Len > 0){
		/* Wait until RXNE is set*/
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		/* Check the DFF bit in CR1*/
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF))){
			/* 16 bit DFF */
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else{
			/* 8 bit DFF */
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/**********************************************************
 * @fn			- SPI_IRQInterruptConfig
 *
 * @brief		-
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/**********************************************************
 * @fn			- SPI_IRQPriorityConfig
 *
 * @brief		-
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx) |=  ( IRQPriority << shift_amount );
}

/**********************************************************
 * @fn			- SPI_SendDataIT
 *
 * @brief		-
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){
		/* Save the Tx buffer address and Len information in some global variables*/
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		/* Mark the SPI state as bust in transmission so that
		 * no other code can take over same SPI peripheral until transmission is over*/
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		/* Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR */
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	}

	return state;

}

/**********************************************************
 * @fn			- SPI_ReceiveDataIT
 *
 * @brief		-
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX){
		/* Save the Rx buffer address and Len information in some global variables*/
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		/* Mark the SPI state as bust in transmission so that
		 * no other code can take over same SPI peripheral until transmission is over*/
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		/* Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR */
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	}

	return state;
}



/**********************************************************
 * @fn			- SPI_IRQHandling
 *
 * @brief		-
 *
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */

void SPI_IRQHandling(SPI_Handle_t *pHandle){

	uint8_t temp1, temp2;
	/* first lets check for TXE */
	temp1 = (pHandle->pSPIx->SR & (1 << SPI_SR_TXE));
	temp2 = (pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE));

	if( temp1 && temp2 ){

		/* handle TXE*/
		spi_txe_interrupt_handle(pHandle);
	}

	/* first lets check for RXNE */
	temp1 = (pHandle->pSPIx->SR & (1 << SPI_SR_RXNE));
	temp2 = (pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE));

	if( temp1 && temp2 ){

		/* handle RXNE*/
		spi_rxne_interrupt_handle(pHandle);
	}

	/* first lets check for OVR flag */
	temp1 = (pHandle->pSPIx->SR & (1 << SPI_SR_OVR));
	temp2 = (pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE));

	if( temp1 && temp2 ){

		/* handle OVR error*/
		spi_ovr_err_interrupt_handle(pHandle);
	}
}


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	/* Check the DFF bit in CR1*/
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))){
		/* 16 bit DFF */
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;

		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else{
		/* 8 bit DFF */
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;

		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen){
		/* TxLen is zero, than close the spi communication and inform the application
		 * that TX is over*/

		/* Prevents interrupts from setting from TXE flag*/
		SPI_CloseTransmisson(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	/* Check the DFF bit in CR1*/
	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))){
		/* 16 bit DFF */
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;

	}else{
		/* 8 bit DFF */
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;

	}

	if(! pSPIHandle->RxLen){
		/* RxLen is zero, than close the spi communication and inform the application
		 * that RX is over*/

		/* Prevents interrupts from setting from RXNEIE flag*/
		SPI_CloseReception(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;
	/* Clear the OVR Flag */
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	/* Inform the application */
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}


void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_CLearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){

}
