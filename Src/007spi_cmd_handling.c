/*
 * 007spi_cmd_handling.c
 *
 *  Created on: Jul 4, 2024
 *      Author: Tunahan_KOSEOGLU
 */


/*
 * PB14 -> MISO
 * PB15 -> MOSI
 * PB13 -> SCLK
 * PB12 -> NSS
 * Alternate Functionality: 5
 */


#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

extern void initialise_monitor_handles();



#define COMMAND_LED_CTRL  	  0x50
#define COMMAND_SENSOR_READ   0x50
#define COMMAND_LED_READ 	  0x50
#define COMMAND_PRINT  		  0x50
#define COMMAND_ID_READ  	  0x50

#define LED_ON    1
#define LED_OFF   0

#define ANALOG_PIN_0      0
#define ANALOG_PIN_1      1
#define ANALOG_PIN_2      2
#define ANALOG_PIN_3      3
#define ANALOG_PIN_4      4

// arduino led
#define LED_PIN 		  9

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

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);  //MISO

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

SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}
	return 0;
}

int main(void)
{

	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read;

	initialise_monitor_handles();

	printf("Application is running\n");


	GPIO_ButtonInit();

	SPI2_GPIOInits();

	SPI2_Inits();
	printf("SPI Initialized\n");

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

			// 1. CMD_LED_CTRL   <PIN_NO_1>   <VALUE:1>
			uint8_t commcode = COMMAND_LED_CTRL;
			uint8_t ackbyte;
			uint8_t args[2];
			SPI_SendData(SPI2, &commcode, 1);

			//do dummy read  to clear of the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//send some dummy bits (1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write,1);
			// read the ack byte received
			SPI_ReceiveData(SPI2, &ackbyte, 1);

			if(SPI_VerifyResponse(ackbyte))
			{
				//send arguments
				args[0] = LED_PIN;
				args[1] = LED_ON;
				SPI_SendData(SPI2, &args, 2);
				printf("COMMAND_LED_CTRL\n");
			}
			//end of COMMAND_LED_CTRL

			// 2. CMD_SENSOR_READ  <analog pin number(1)>

			//wait till button is pressed
			while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

			//to avoid button de-bouncing related issues 200ms of delay
			delay();

			commcode = COMMAND_SENSOR_READ;

			SPI_SendData(SPI2, &commcode, 1);

			//do dummy read  to clear of the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//send some dummy bits (1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write,1);
			// read the ack byte received
			SPI_ReceiveData(SPI2, &ackbyte, 1);

			if(SPI_VerifyResponse(ackbyte))
			{
				//send arguments
				args[0] = ANALOG_PIN_0;
				SPI_SendData(SPI2, &args,1); // sending one byte

				//do dummy read  to clear of the RXNE
				SPI_ReceiveData(SPI2, &dummy_read, 1);

				//insert some delay so that slave can ready with the data
				delay();


				//send some dummy bits (1byte) to fetch the response from the slave
				SPI_SendData(SPI2, &dummy_write,1);

				uint8_t analog_read;
				SPI_ReceiveData(SPI2, &analog_read, 1);

				printf("COMMAND_SENSOR_READ %D\n", analog_read);
			}


			//3.  CMD_LED_READ 	 <pin no(1) >

			//wait till button is pressed
			while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

			//to avoid button de-bouncing related issues 200ms of delay
			delay();

			commcode = COMMAND_LED_READ;

			//send command
			SPI_SendData(SPI2,&commcode,1);

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//Send some dummy byte to fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			//read the ack byte received
			SPI_ReceiveData(SPI2,&ackbyte,1);

			if( SPI_VerifyResponse(ackbyte))
			{
				args[0] = LED_PIN;

				//send arguments
				SPI_SendData(SPI2,args,1); //sending one byte of

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1);

				//insert some delay so that slave can ready with the data
				delay();

				//Send some dummy bits (1 byte) fetch the response from the slave
				SPI_SendData(SPI2,&dummy_write,1);

				uint8_t led_status;
				SPI_ReceiveData(SPI2,&led_status,1);
				printf("COMMAND_READ_LED %d\n",led_status);

			}

			//4. CMD_PRINT 		<len(2)>  <message(len) >

			//wait till button is pressed
			while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

			//to avoid button de-bouncing related issues 200ms of delay
			delay();

			commcode = COMMAND_PRINT;

			//send command
			SPI_SendData(SPI2,&commcode,1);

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//Send some dummy byte to fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			//read the ack byte received
			SPI_ReceiveData(SPI2,&ackbyte,1);

			uint8_t message[] = "Hello ! How are you ??";
			if( SPI_VerifyResponse(ackbyte))
			{
				args[0] = strlen((char*)message);

				//send arguments
				SPI_SendData(SPI2,args,1); //sending length

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1);

				delay();

				//send message
				for(int i = 0 ; i < args[0] ; i++){
					SPI_SendData(SPI2,&message[i],1);
					SPI_ReceiveData(SPI2,&dummy_read,1);
				}

				printf("COMMAND_PRINT Executed \n");

			}

			//5. CMD_ID_READ
			//wait till button is pressed
			while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

			//to avoid button de-bouncing related issues 200ms of delay
			delay();

			commcode = COMMAND_ID_READ;

			//send command
			SPI_SendData(SPI2,&commcode,1);

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//Send some dummy byte to fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			//read the ack byte received
			SPI_ReceiveData(SPI2,&ackbyte,1);

			uint8_t id[11];
			uint32_t i=0;
			if( SPI_VerifyResponse(ackbyte))
			{
				//read 10 bytes id from the slave
				for(  i = 0 ; i < 10 ; i++)
				{
					//send dummy byte to fetch data from slave
					SPI_SendData(SPI2,&dummy_write,1);
					SPI_ReceiveData(SPI2,&id[i],1);
				}

				id[10] = '\0';

				printf("COMMAND_ID : %s \n",id);

			}

			//lets confirm SPI is not busy
			while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

			//Disable the SPI2 peripheral
			SPI_PeripheralControl(SPI2,DISABLE);

			printf("SPI Communication Closed\n");
		}















	return 0;
}






