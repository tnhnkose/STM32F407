/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 3, 2024
 *      Author: Tunahan_KOSEOGLU
 */

#include "stm32f407xx_spi_driver.h"




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
				pSPIx->DR = ((uint16_t*)pTxBuffer);
				Len--;
				Len--;
				(uint16_t*)pTxBuffer++;
			}
			else
			{
				pSPIx->DR = ((uint8_t*)pTxBuffer);
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

}





