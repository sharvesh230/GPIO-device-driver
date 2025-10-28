/*
 * stm32f401xx.h
 *
 *  Created on: Jun 27, 2025
 *  Author: santosh
 */

#ifndef STM32F401XX_H_
#define STM32F401XX_H_

#include "stdint.h" 		//This lib is necessary when you use datatypes like uint32_t etc

#define ENABLE 							1
#define DISABLE							0
#define SET 							ENABLE
#define RESET 							DISABLE

/********************************************Processor specific macros ***************************************************/

/*
 * The following macros defines the processor specific macros
 * These macros are used to define the NVIC interrupt set registers to there base address
 * The NVIC is used to handle the interrupts in the microcontroller
 * Look the register base address and information in the cortex-m4 devices generic user guide - "https://documentation-service.arm.com/static/5f2ac76d60a93e65927bbdc5?token"
 * Use the link to download the user guide
 */

#define NVIC_ISER0						( ( volatile uint32_t* ) 0xE000E100 )
#define NVIC_ISER1						( ( volatile uint32_t* ) 0xE000E104 )
#define NVIC_ISER2						( ( volatile uint32_t* ) 0xE000E108 )
#define NVIC_ISER3						( ( volatile uint32_t* ) 0xE000E10C )

/*
 * The following macros defines NVIC interrupt clear registers to there base address
 */

#define NVIC_ICER0						( ( volatile uint32_t* ) 0xE000E180 )
#define NVIC_ICER1						( ( volatile uint32_t* ) 0xE000E184 )
#define NVIC_ICER2						( ( volatile uint32_t* ) 0xE000E188 )
#define NVIC_ICER3						( ( volatile uint32_t* ) 0xE000E18C )

/*
 * The following macros defines the NVIC priority registers to there base address
 */

#define NVIC_IPR_BASEADDR				( ( volatile uint32_t* ) 0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED			4 // This is the number of bits implemented in the priority register and this highly manufacturer specific

/*-----------------------------------------------------------------------------------------------------------------------*/
/*
 * The following macros contains the base address of the each memory
 * All the data can be found in the data sheet and the reference manual(The development board i am using is the
 * STM32F401CCU6 Minimum System Board Microcomputer STM32 ARM Core Board)
 * The datasheet of the stm32f401ccu6 can be installed during the project creation
 * This is the link of the reference manual: "https://www.st.com/resource/en/reference_manual/
 * rm0368-stm32f401xbc-and-stm32f401xde-advanced-armbased-32bit-mcus-stmicroelectronics.pdf"
*/

#define FLASH_BASEADDR					0x08000000U
#define SRAM_BASEADDR					0x20000000U

#define ROM								0x1FFF0000U
#define SRAM							SRAM_BASEADDR

/*------------------------------------------------Bus macro definition-----------------------------------------------------------*/

/*
 * The following macros defines the base address of the AHB and the APB buses
 * Again all the information can be found in the reference manual(Pg no: 51)
 * All the peripherals are connected to the microprocessor through there
 * respective bus to which they are connected. Also the peripherals has the base address
 * which is simply equal to "connected bus base address + offset of the peripheral"
 * Please refer the memory map section of the reference manual to understand this concept
 */

#define AHB1_BASEADDR					0x40020000U
#define AHB2_BASEADDR					0x50000000U
#define APB1_BASEADDR					0x40000000U
#define APB2_BASEADDR					0x40010000U

/*------------------------------------------------Peripheral macro definition----------------------------------------------------*/

/*
 *The following block of macros involves in defining the macros of the base addresses
 *of the peripherals (GPIO,I2C,SPI)
 *
 *Refer the MEMORY MAPPING section of the Datasheet
 *
 *#definition					( BASEADDR_OF_THE_BUS_TO_WHICH_IT_IS_CONNECTED + OFFSET)
 *
 */

#define	GPIOA_BASEADDR					( AHB1_BASEADDR + 0x00000000U )
#define GPIOB_BASEADDR					( AHB1_BASEADDR + 0x00000400U )
#define GPIOC_BASEADDR					( AHB1_BASEADDR + 0x00000800U )
#define GPIOD_BASEADDR					( AHB1_BASEADDR + 0x00000C00U )
#define GPIOE_BASEADDR					( AHB1_BASEADDR + 0x00001000U )
#define GPIOH_BASEADDR					( AHB1_BASEADDR + 0x00001C00U )

#define I2C1_BASEADDR					( APB1_BASEADDR + 0x00005400U )
#define I2C2_BASEADDR 					( APB1_BASEADDR + 0x00005800U )
#define I2C3_BASEADDR 					( APB1_BASEADDR + 0x00005C00U )

#define SPI1_BASEADDR 					( APB2_BASEADDR + 0X00003000U )
#define SPI2_BASEADDR 					( APB1_BASEADDR + 0X00003800U )
#define SPI3_BASEADDR 					( APB1_BASEADDR + 0X00003C00U )
#define SPI4_BASEADDR					( APB2_BASEADDR + 0X00003400U )

#define USART1_BASEADDR					( APB2_BASEADDR + 0x00001000U )
#define USART2_BASEADDR					( APB1_BASEADDR + 0x00004400U )
#define USART6_BASEADDR 				( APB2_BASEADDR + 0x00001400U )

#define EXTI_BASEADDR 					( APB2_BASEADDR + 0x00003C00U )

#define SYSCFG_BASEADDR 				( APB2_BASEADDR + 0x00003800U )


/*------------------------------------------------GPIO REGISTER DEFINITION-----------------------------------------------------------*/
/*
 * The following block of macro definition involves in defining the GPIO REGISTERS to there base address
 * a simpler and an efficient way of doing that is by defining them in a structure rather than defining through
 * macros since there is alot of GPIO's and defining macro's for each is inefficient.
 *
 * Refer the GPIO registers section of the reference manual
 */

#define RCC_BASEADDR 					( AHB1_BASEADDR + 0x00003800U )

typedef struct
{
	volatile uint32_t MODER;			//GPIOA_MODER									OFFSET - 0x00
	volatile uint32_t OTYPER;			//GPIOx_OTYPER									OFFSET - 0x04
	volatile uint32_t OSPEEDR;			//GPIOx_OSPEEDR									OFFSET - 0x08
	volatile uint32_t PUPDR;			//GPIOA_PUPDR									OFFSET - 0x0C
	volatile uint32_t IDR;				//GPIOx_IDR										OFFSET - 0x10
	volatile uint32_t ODR;				//GPIOx_ODR										OFFSET - 0x14
	volatile uint32_t BSRR;				//GPIOx_BSRR									OFFSET - 0x18
	volatile uint32_t LCKR;				//GPIOx_LCKR									OFFSET - 0x1C
	volatile uint32_t AFR[2];			//AFR[0] = GPIOx_AFRL ; AFR[1] = GPIOx_AFRH		OFFSET - 0x20 ; OFFSET - 0X24
}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;             	// 0x00: Clock control register
	volatile uint32_t PLLCFGR;        	// 0x04: PLL configuration register
	volatile uint32_t CFGR;           	// 0x08: Clock configuration register
	volatile uint32_t CIR;            	// 0x0C: Clock interrupt register
	volatile uint32_t AHB1RSTR;       	// 0x10: AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;       	// 0x14: AHB2 peripheral reset register
	uint32_t Reserved1;               	// 0x18: Reserved
	uint32_t Reserved2;               	// 0x1C: Reserved
	volatile uint32_t APB1RSTR;       	// 0x20: APB1 peripheral reset register
	volatile uint32_t APB2RSTR;       	// 0x24: APB2 peripheral reset register
	uint32_t Reserved3;               	// 0x28: Reserved
	uint32_t Reserved4;               	// 0x2C: Reserved
	volatile uint32_t AHB1ENR;        	// 0x30: AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;        	// 0x34: AHB2 peripheral clock enable register
	uint32_t Reserved5;               	// 0x38: Reserved
	uint32_t Reserved6;               	// 0x3C: Reserved
	volatile uint32_t APB1ENR;        	// 0x40: APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;        	// 0x44: APB2 peripheral clock enable register
	uint32_t Reserved7;               	// 0x48: Reserved
	uint32_t Reserved8;               	// 0x4C: Reserved
	volatile uint32_t AHB1LPENR;      	// 0x50: AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;      	// 0x54: AHB2 peripheral clock enable in low power mode register
	uint32_t Reserved9;               	// 0x58: Reserved
	uint32_t Reserved10;              	// 0x5C: Reserved
	volatile uint32_t APB1LPENR;      	// 0x60: APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;      	// 0x64: APB2 peripheral clock enable in low power mode register
	uint32_t Reserved11;              	// 0x68: Reserved
	uint32_t Reserved12;             	// 0x6C: Reserved
	volatile uint32_t BDCR;           	// 0x70: Backup domain control register
	volatile uint32_t CSR;            	// 0x74: Clock control & status register
	uint32_t Reserved13;              	// 0x78: Reserved
	uint32_t Reserved14;              	// 0x7C: Reserved
	volatile uint32_t SSCGR;          	// 0x80: Spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;     	// 0x84: PLLI2S configuration register
	uint32_t Reserved15;				// 0x88: Reserved
	volatile uint32_t DCKCFGR;        	// 0x8c: Dedicated clocks configuration register
} RCC_RegDef_t;

typedef struct
{
	volatile uint32_t IMR;				// 0x00: Interrupt mask register
	volatile uint32_t EMR;				// 0x04: Event mask register
	volatile uint32_t RTSR;				// 0x08: Rising trigger selection register
	volatile uint32_t FTSR;				// 0x0C: Falling trigger selection register
	volatile uint32_t SWIER;			// 0x10: Software interrupt event register
	volatile uint32_t PR;				// 0x14: Pending register
} EXTI_RegDef_t;


typedef struct
{
	volatile uint32_t MEMRMP;			// 0x00: Memory remap register
	volatile uint32_t PMC;				// 0x04: Peripheral mode configuration register
	volatile uint32_t EXTICR[4];		// 0x08: External interrupt configuration registers
	uint32_t Reserved[2];				// 0x18,0x1C: Reserved
	volatile uint32_t CMPCR;			// 0x20: Compensation cell control register
}SYSCFG_RegDef_t;

/*
 * Now we pass the base address of the SYSCFG peripheral to a pointer of type SYSCFG_RegDef_t
 */

#define SYSCFG 							( ( SYSCFG_RegDef_t* ) SYSCFG_BASEADDR )

/*
 * Another peripheral definition that is type casted the struct in order to use them to point their registers
 */

#define GPIOA 							( ( GPIO_RegDef_t* ) GPIOA_BASEADDR )
#define GPIOB 							( ( GPIO_RegDef_t* ) GPIOB_BASEADDR )
#define GPIOC 							( ( GPIO_RegDef_t* ) GPIOC_BASEADDR )
#define GPIOD 							( ( GPIO_RegDef_t* ) GPIOD_BASEADDR )
#define GPIOE 							( ( GPIO_RegDef_t* ) GPIOE_BASEADDR )
#define GPIOH 							( ( GPIO_RegDef_t* ) GPIOH_BASEADDR )

#define RCC 							( ( RCC_RegDef_t* ) RCC_BASEADDR )

/*
 * Now we pass the base address of the EXTI peripheral to a pointer of type EXTI_RegDef_t
 */

#define EXTI 							( ( EXTI_RegDef_t* ) EXTI_BASEADDR )

/*
 * GPIOx Peripheral clock enable macros
 */

#define GPIOA_PCLK_EN() 					( RCC -> AHB1ENR |= (1 << 0))
#define GPIOC_PCLK_EN() 					( RCC -> AHB1ENR |= (1 << 2))
#define GPIOB_PCLK_EN() 					( RCC -> AHB1ENR |= (1 << 1))
#define GPIOD_PCLK_EN() 					( RCC -> AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() 					( RCC -> AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN() 					( RCC -> AHB1ENR |= (1 << 7))

/*
 * GPIOx Peripheral clock disable macros
 */

#define GPIOA_PCLK_DIS() 					( RCC -> AHB1ENR &= ~(1 << 0))
#define GPIOC_PCLK_DIS() 					( RCC -> AHB1ENR &= ~(1 << 2))
#define GPIOB_PCLK_DIS() 					( RCC -> AHB1ENR &= ~(1 << 1))
#define GPIOD_PCLK_DIS() 					( RCC -> AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DIS() 					( RCC -> AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DIS() 					( RCC -> AHB1ENR &= ~(1 << 7))

/*
 * I2C Peripheral clock enable macros
 */

#define I2C1_PCLK_EN()						( RCC -> APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()						( RCC -> APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()						( RCC -> APB1ENR |= (1 << 23))

/*
 * I2C Peripheral clock disable macros
 */

#define I2C1_PCLK_DIS()						( RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DIS()						( RCC -> APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DIS()						( RCC -> APB1ENR &= ~(1 << 23))

/*
 * SPI Peripheral clock enable macros
 */

#define SPI1_PCLK_EN()						( RCC -> APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()						( RCC -> APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()						( RCC -> APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()						( RCC -> APB2ENR |= (1 << 13))

/*
 * SPI Peripheral clock disable macros
 */

#define SPI1_PCLK_DIS()						( RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DIS()						( RCC -> APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DIS()						( RCC -> APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DIS()						( RCC -> APB2ENR &= ~(1 << 13))

/*
 * USART Peripheral clock enable macros
 */

#define USART1_PCLK_EN()					( RCC -> APB2ENR |= (1 << 04))
#define USART2_PCLK_EN()					( RCC -> APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()					( RCC -> APB2ENR |= (1 << 05))

/*
 * USART Peripheral clock disable macros
 */

#define USART1_PCLK_DIS()					( RCC -> APB2ENR &= ~(1 << 04))
#define USART2_PCLK_DIS()					( RCC -> APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DIS()					( RCC -> APB2ENR &= ~(1 << 05))

/*
 * System config controller clock enable
 */

#define SYSCFG_CLK_EN()						( RCC -> APB2ENR |= (1 << 14))

/*
 * System config controller clock disable
 */

/*
 * Peripheral reset macros
 */

#define GPIOA_REG_RESET()					do{ (RCC -> AHB1RSTR |= (1 << 0)); (RCC -> AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()					do{ (RCC -> AHB1RSTR |= (1 << 1)); (RCC -> AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()					do{ (RCC -> AHB1RSTR |= (1 << 2)); (RCC -> AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()					do{ (RCC -> AHB1RSTR |= (1 << 3)); (RCC -> AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()					do{ (RCC -> AHB1RSTR |= (1 << 4)); (RCC -> AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()					do{ (RCC -> AHB1RSTR |= (1 << 7)); (RCC -> AHB1RSTR &= ~(1 << 7)); }while(0)

#define SYSCFG_CLK_DIS()					( RCC -> APB2ENR &= ~(1 << 14))

/*
 * Defining the macro GPIO_BASEADDR_TO_CODE
 */

#define GPIO_BASEADDR_TO_CODE(x)			  ( (x == GPIOA) ? 0 : \
												(x == GPIOB) ? 1 : \
												(x == GPIOC) ? 2 : \
												(x == GPIOD) ? 3 : \
												(x == GPIOE) ? 4 : \
												(x == GPIOH) ? 7 : 0 )
#endif /* STM32F401XX_H_ */
