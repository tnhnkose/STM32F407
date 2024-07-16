/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 3, 2024
 *      Author: Tunahan_KOSEOGLU
 */

#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle); // static makes the function private
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*
 * Peripheral Clock setup
 */
/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
}

/*
 * SPI Initialization
 */
/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function initializes  the given SPI
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//peripheral clock enable

		SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

		//first lets configure the SPI_CR1 register

		uint32_t tempreg = 0;

		//1. configure the device mode
		tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;

	//2. Configure the bus config
		if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
		{
			//bidi mode should be cleared
			tempreg &= ~( 1 << SPI_CR1_BIDIMODE);

		}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
		{
			//bidi mode should be set
			tempreg |= ( 1 << SPI_CR1_BIDIMODE);
		}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
		{
			//BIDI mode should be cleared
			tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
			//RXONLY bit must be set
			tempreg |= ( 1 << SPI_CR1_RXONLY);
		}

	// 3. Configure the spi serial clock speed (baud rate)
		tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

		//4.  Configure the DFF
		tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

		//5. configure the CPOL
		tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

		//6 . configure the CPHA
		tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

		tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;


	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*
 * SPI Deinitialization
 */
/*********************************************************************
 * @fn      		  - SPI_Deinit
 *
 * @brief             - This function deinitializes  the given SPI
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}

}
/*
 * SPI flag status information
 */
/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function gives the status for given flag type
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - Flag Name
 * @param[in]         -
 *
 * @return            -  Flag Status
 *
 * @Note              -  none

 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR == FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * SPI Send data
 */
/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function sends data with using SPI peripheral
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - the data which will be sent
 * @param[in]         - data length
 *
 * @return            -  none
 *
 * @Note              -  This is blocking call

 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{

	while(Len > 0)
	{
		//wait until TXE set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET)
		{
			if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
			{
				// 16 bit DFF bit in CR1
				// 1. load data into data register
				pSPIx->DR = *((uint16_t*)pTxBuffer);
				Len--;
				Len--;
				(uint16_t*)pTxBuffer++;
			}
			else
			{
				pSPIx->DR = *((uint8_t*)pTxBuffer);
				Len--;
				pTxBuffer++;
			}

		}

	}



}

/*
 * SPI enable
 */
/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - This function enables or disables peripheral for the given spi
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/*
 * SPI SSI Configuration
 */
/*********************************************************************
 * @fn      		  - SPI_SSIConFig
 *
 * @brief             - This function enables or disables SSI bit for the given spi port
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI_SSIConFig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}
		else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}



/*
 * SPI SSOE Configuration
 */
/*********************************************************************
 * @fn      		  - SPI_SSOEConFig
 *
 * @brief             - This function enables or disables SSOE bit for the given spi port
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void SPI_SSOEConFig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR2_SSOE);
		}
		else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR2_SSOE);
		}
}



/*
 * SPI Receive data
 */
/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function reads data with using SPI peripheral
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - given data
 * @param[in]         - data length
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//wait until RXNE set
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET)
			{
				if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
				{
					// 16 bit DFF bit in CR1
					// read data from data register
					*((uint16_t*)pRxBuffer) = pSPIx->DR;
					Len--;
					Len--;
					(uint16_t*)pRxBuffer++;
				}
				else
				{
					*((uint8_t*)pRxBuffer) = pSPIx->DR;
					Len--;
					pRxBuffer++;
				}

			}

		}
}


/*
 * Peripheral Clock setup
 */
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >= 31 && IRQNumber <= 63)
		{
			// ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}else if(IRQNumber >= 64 && IRQNumber <= 96)
		{
			// ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
}


/*
 * Peripheral Clock setup
 */
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// first find out ipr register

		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section = IRQNumber % 4;

		uint8_t shift_amount = ( 8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
		*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );


}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
	//save Tx buffer address and Len information in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// made sp, state as busy
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	}

	return state;
	//data transmission will be handled by the ISR code

}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
		uint8_t state = pSPIHandle->RxState;

		if(state != SPI_BUSY_IN_RX)
		{
			//save Rx buffer address and Len information in global variables
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = Len;

			// made sp, state as busy
			pSPIHandle->RxState = SPI_BUSY_IN_RX;

			//enable the RXNEIE control bit to get interrupt whenever TXE flag is set in SR
			pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		}

		return state;
		//data transmission will be handled by the ISR code
}



/*
 * Peripheral Clock setup
 */
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	// check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);

	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for OVR flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle RXNE
		spi_ovr_err_interrupt_handle(pHandle);
	}


}

// some helper function implementations
// static makes the function private
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{


	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit DFF bit in CR1
		// 1. load data into data register
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		// 8 bit  DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		// TxLen is zero, close the spi communication and inform the application that
		// Tx is over.

		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}




}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit DFF bit in CR1
		// 1. load data into data register
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	}
	else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}


	if(! pSPIHandle->RxLen)
	{
		// RxLen is zero, close the spi communication and inform the application that
		// Rx is over.

		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	// clear ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}

	// inform the application
	SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	////This is a weak implementation, the app may override this function
}






