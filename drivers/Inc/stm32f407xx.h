/*
 * stm32f407xx.h
 *
 *  Created on: Jun 12, 2024
 *      Author: Tunahan_KOSEOGLU
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>
#define __vo volatile
/*
 * Base Addresses of SRAM and FLASH memories
 */

#define FLASH_BASEADDR           		0x08000000U
#define SRAM1_BASEADDR					0x20000000U
#define SRAM2_BASEADDR					0x2001C000U
#define ROM_BASEADDR					0x1FFF0000U
#define SRAM 							SRAM1_BASEADDR


/*
 * AHBx and APBx peripheral addresses
 */

#define PERIPH_BASEADDR 				0x40000000U
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHB1PERIPH_BASEADDR				0x40020000U
#define AHB2PERIPH_BASEADDR				0x50000000U


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */

#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1C00U)
#define GPIOI_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x2000U)
#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800U)



/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00U)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00U)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800U)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00U)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000U)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00U)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000U)
#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800U)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000U)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400U)



/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t MODER;                        /*!< GPIO port mode register,                    	Address offset: 0x00      */
	__vo uint32_t OTYPER;                       /*!< GPIO port Output type register					Address offset: 0x04      */
	__vo uint32_t OSPEEDR;						/*!< GPIO port Output speed register				Address offset: 0x08	  */
	__vo uint32_t PUPDR; 						/*!< GPIO port pull-up/pull-down register			Address offset: 0x0C	  */
	__vo uint32_t IDR;							/*!< GPIO port input data register					Address offset: 0x10	  */
	__vo uint32_t ODR;							/*!< GPIO port output data register					Address offset: 0x14	  */
	__vo uint32_t BSRR;							/*!< GPIO port bit set/reset register 				Address offset: 0x18	  */
	__vo uint32_t LCKR;							/*!< GPIO port configuration lock register			Address offset: 0x1C	  */
	__vo uint32_t AFR[2];						 /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;





/*
 * peripheral definitions, typecasted to RegDef
 */

#define GPIOA            ( (GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB            ( (GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC            ( (GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD            ( (GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE            ( (GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF            ( (GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG            ( (GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH            ( (GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI            ( (GPIO_RegDef_t*) GPIOI_BASEADDR)


/*
 * RCC registers
 */

typedef struct
{
	__vo uint32_t CR;   					 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t PLLCFGR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t CFGR;   					 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t CIR;   					 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t AHB1RSTR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t AHB2RSTR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t AHB3RSTR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t APB1RSTR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t APB2RSTR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t AHB1ENR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t AHB2ENR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t AHB3ENR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t APB1ENR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t APB2ENR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t AHB1LPENR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t AHB2LPENR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t AHB3LPENR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t APB1LPENR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t APB2LPENR;   				 /*!< RRCC clock control register,                 Address offset: 0x00      */
	__vo uint32_t BDCR;   					 /*!< RRCC clock control register,                 Address offset: 0x00      */



}RCC_RegDef_t;








#endif /* INC_STM32F407XX_H_ */
