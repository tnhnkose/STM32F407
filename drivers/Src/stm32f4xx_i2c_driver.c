/*
 * stm32f4xx_i2c_driver.c
 *
 *  Created on: Jul 16, 2024
 *      Author: Tunahan_KOSEOGLU
 */

#include "stm32f407xx_spi_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[8] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t pI2Cx);
/*
 * Peripheral Clock setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_EN();
			}else if(pI2Cx == I2C2)
			{
				I2C2_PCLK_EN();
			}else if(pI2Cx == I2C3)
			{
				I2C3_PCLK_EN();
			}
		}
}

uint32_t RCC_GetPLLOutputClock()
{
	return;
}


uint32_t RCC_GetPCLK1Value()
{
	uint32_t pclk1,SystemClk;

	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 3);

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	// for AHB
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	// for APB1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		ahbp = APB1_PreScaler[temp-4];
	}

	 pclk1 = (SystemClk / ahbp) / apb1p;

	return  pclk1;
}
/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;
	//peripheral clock enable
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// enable the acking
	tempreg |= (pI2CHandle->I2CConfig.I2C_ACKControl << 10);
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field OF CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000;
	pI2CHandle->pI2Cx->CR2 =(tempreg & 0x3F);

	// program the device own address
	tempreg |= pI2CHandle->I2CConfig.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR Calculation
	uint16_t ccr_value;
	tempreg = 0;
	if(pI2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standart mode
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2CConfig.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		//mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2CConfig.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2CConfig.I2C_SCLSpeed);
		}
		else
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2CConfig.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;
	//configure the speed of SCL

	// configure the device address



	//configure the rise time for I2C pins
}


void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

}


/*
 * Data Send and Receive
 */

/*
 * SPI flag status information
 */
/*********************************************************************
 * @fn      		  - I2C_GetFlagStatus
 *
 * @brief             - This function gives the status for given flag type
 *
 * @param[in]         - base address of the i2c peripheral
 * @param[in]         - Flag Name
 * @param[in]         -
 *
 * @return            -  Flag Status
 *
 * @Note              -  none

 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 == FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to low)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//send the address of the slave with w/r bit
}


static void I2C_GenerateStartCondition(I2C_RegDef_t pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}



















