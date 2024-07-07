/*
 * 006spi_tx_arduino.c
 *
 *  Created on: Jul 4, 2024
 *      Author: Tunahan_KOSEOGLU
 */




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

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);  //NSS
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

		SPI2handle.pSPIx = SPI2;
		SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
		SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
		SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;//generates sclk of 8MHz
		SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
		SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
		SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
		SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //software slave management enabled for NSS pin

		SPI_Init(&SPI2handle);
}

void delay(void)
{
	for(uint32_t i=0; i<500000 ; i++);
}

GPIO_ButtonInit()
{
	GPIO_Handle_t GpioButton;

	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioButton);
}

int main(void)
{
	char user_data[] = "Hello World";

	GPIO_ButtonInit();

	SPI2_GPIOInits();

	SPI2_Inits();

	/*
	 * making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * i.e when SPE=1 , NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConFig(SPI2,ENABLE);
	while(1)
		{
			//wait till button is pressed
			while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

			//to avoid button de-bouncing related issues 200ms of delay
			delay();

			//enable the SPI2 peripheral
			SPI_PeripheralControl(SPI2,ENABLE);

			//first send length information
			uint8_t dataLen = strlen(user_data);
			SPI_SendData(SPI2,&dataLen,1);

			//to send data
			SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

			//lets confirm SPI is not busy
			while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

			//Disable the SPI2 peripheral
			SPI_PeripheralControl(SPI2,DISABLE);
		}
	return 0;
}






