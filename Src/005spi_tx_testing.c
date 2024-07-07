/*
 * 005spi_tx_testing.c
 *
 *  Created on: Jul 3, 2024
 *      Author: Tunahan_KOSEOGLU
 */

/*
 * PB14 -> MISO
 * PB15 -> MOSI
 * PB13 -> SCLK
 * PB12 -> NSS
 * Alternate Functionality: 5
 */

#include <string.h>
#include "stm32f407xx.h"

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	GPIO_PeriClockControl(GPIOB, ENABLE);

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;


	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);  //SCLK

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins); //MOSI

	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);  //MISO

	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);  //NSS
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

		SPI2handle.pSPIx = SPI2;
		SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
		SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
		SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;//generates sclk of 8MHz
		SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
		SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
		SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
		SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //software slave management enabled for NSS pin

		SPI_Init(&SPI2handle);
}

int main(void)
{
	char user_data[] = "Hello World";


	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSICondig();

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	SPI_SendData(SPI2, user_data, strlen(user_data));

	//disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,DISABLE);



	return 0;
}






