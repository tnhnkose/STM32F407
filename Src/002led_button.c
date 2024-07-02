/*
 * 002led_button.c
 *
 *  Created on: Jul 2, 2024
 *      Author: Tunahan_KOSEOGLU
 */


#include "stm32f407xx.h"


void delay(void)
{
	for(uint32_t i=0; i<500000 ; i++);
}
int main(void)
{


	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioButton;
	uint8_t value,lastvalue;

	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioButton);


	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);



	while(1)
	{

		value = GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0);
		if(value == 1)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
		}
	}

	return 0;
}
