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
 * @param[in]         - base address of the gpio peripheral
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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// first configure CR1 register
	uint32_t tempreg=0;

	// configure device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//configure bus_config
	if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_BUS_CONFIG_FD)
	{
		tempreg &= ~(1 << 15);  // bidi mode cleared
	}
	else if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_BUS_CONFIG_HD)
	{
		tempreg |= (1 << 15); // bidi mode set

	}
	else if(pSPIHandle->SPIConfig.SPI_DeviceMode == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		tempreg &= ~(1 << 15); // bidi mode cleared
		tempreg |=  (1 << 10); // rxonly set
	}

	// configure sclk_speed
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// configure spi_dff
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// configure spi_cpol
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// configure spi_cpha
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// configure spi_cpha
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);


	pSPIHandle->pSPIx->CR[0] = tempreg;
}


/*
 * SPI Deinitilization
 */
/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function deinitializes SPIx
 *
 * @param[in]         - base address of the SPI peripheral
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

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR == FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Peripheral Clock setup
 */
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the spi peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
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
			if(pSPIx->CR[0] & (1 << SPI_CR1_DFF))
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
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





