/*
 * stm32f401xx_gpio_drivers.h
 *
 *  Created on: Jun 28, 2025
 *  Author: santosh
 */

#ifndef INC_STM32F401XX_GPIO_DRIVERS_H_
#define INC_STM32F401XX_GPIO_DRIVERS_H_

#include "stm32f401xx.h"
/*
 * This structure is used to configure the GPIO pin
 * Contains the pin number, pin mode, pin speed, pull-up/pull-down control,Pin output type and alternate function mode
 */
typedef struct
{
	uint8_t GPIO_PinNumber;						// Possible values from @GPIO_PinNumber
	uint8_t GPIO_PinMode;						// Possible values from @GPIO_PinMode
	uint8_t GPIO_PinSpeed;						// Possible values from @GPIO_PinSpeed
	uint8_t GPIO_PinPuPdControl;				// Possible values from @GPIO_PinPuPdControl
	uint8_t GPIO_PinOPType;						// Possible values from @GPIO_PinOPType
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * This is the handle structure for a GPIO pin
 * It contains the base address of the GPIO port to which the pin belongs and the pin configuration parameters
 * This structure is used to pass the parameters to the GPIO driver functions
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;						//This is the pointer to hold the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;			//This is the variable that holds the pin configuration parameters
} GPIO_Handle_t;

/*
 * GPIO's possible pin numbers @GPIO_PinNumber
 */
#define GPIO_PIN_0						0		// Pin 0
#define GPIO_PIN_1						1		// Pin 1
#define GPIO_PIN_2						2		// Pin 2
#define GPIO_PIN_3						3		// Pin 3
#define GPIO_PIN_4						4		// Pin 4
#define GPIO_PIN_5						5		// Pin 5
#define GPIO_PIN_6						6		// Pin 6
#define GPIO_PIN_7						7		// Pin 7
#define GPIO_PIN_8						8		// Pin 8
#define GPIO_PIN_9						9		// Pin 9
#define GPIO_PIN_10						10		// Pin 10
#define GPIO_PIN_11						11		// Pin 11
#define GPIO_PIN_12						12		// Pin 12
#define GPIO_PIN_13						13		// Pin 13
#define GPIO_PIN_14						14		// Pin 14
#define GPIO_PIN_15						15		// Pin 15


/*
 * GPIO's possible pin modes @GPIO_PinMode
 */

#define GPIO_MODE_IN 					0		// Input mode
#define GPIO_MODE_OUT          	        1		// Output mode
#define GPIO_MODE_ALTFN         		2		// Alternate function mode
#define GPIO_MODE_ANALOG        		3		// Analog mode
#define GPIO_MODE_IT_FT         		4		// Interrupt mode falling edge trigger
#define GPIO_MODE_IT_RT         		5		// Interrupt mode rising edge trigger
#define GPIO_MODE_IT_RFT				6		// Interrupt mode rising and falling edge trigger

/*
 * GPIO's output speed modes @GPIO_PinSpeed
 */

#define GPIO_SPEED_LOW 					0		// Low speed
#define GPIO_SPEED_MEDIUM 				1		// Medium speed
#define GPIO_SPEED_HIGH					2		// HIGH speed
#define GPIO_SPEED_VERY_HIGH 			3		// Very high speed

/*
 * GPIO's pull-up/pull-down configuration @GPIO_PinPuPdControl
 */

#define GPIO_PIN_NO_PUPD				0		// No pull-up/pull-down
#define GPIO_PIN_PU						1		// Pull-UP
#define GPIO_PIN_PD						2		// Pull-DOWN

/*
 * GPIO's output type @GPIO_PinOPType
 */
#define GPIO_OP_TYPE_PP					0		// Push-Pull output type
#define GPIO_OP_TYPE_OD					1		// Open-Drain output type

/* **************************************************************************************************************************************
 * 													API SUPPORTED BY THIS DRIVER
 * 											These APIs are used to configure the GPIO pin
 ****************************************************************************************************************************************/

// APIs for Peripheral clock control

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// APIs for GPIO initialization and de-initialization

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// APIs for GPIO read and write operations

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// APIs for GPIO interrupt configuration
void IRQ_Config(uint8_t IRQNumber, uint8_t IRQPRIORITY, uint8_t EnorDi);
void IRQ_Handling(uint8_t PinNumber);
























#endif /* INC_STM32F401XX_GPIO_DRIVERS_H_ */
