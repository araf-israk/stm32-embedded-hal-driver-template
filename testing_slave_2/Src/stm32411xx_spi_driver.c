/*
 * stm32411xx_spi_driver.c
 *
 *  Created on: Jan 25, 2025
 *      Author: arafi
 */

#include "stm32f411xx_spi_driver.h"

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

}
