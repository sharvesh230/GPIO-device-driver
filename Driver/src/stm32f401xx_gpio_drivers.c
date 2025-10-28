/*
 * stm32f401xx_gpio_drivers.c
 *
 *  Created on: Jun 28, 2025
 *  Author: santosh
 */

#include "stm32f401xx_gpio_drivers.h"

// APIs for Peripheral clock control
/******************************************************************************************************
 *  @fn				 	GPIO_Peripheral_Clock_Control
 *
 *  @brief				This function enables or diables the peripheral clock for the specified GPIO port.
 *
 *  @param[in]			pGPIOx - Pointer to the GPIO port base address.
 *  @param[in]			EnorDi - Enable or Disable the clock (The macro ENABLE will make it enable and macro DISABLE will make it disable).
 *
 *  @return				None
 *
 *  @note				None
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DIS();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DIS();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DIS();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DIS();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DIS();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DIS();
		}
	}
}

//APIS FOR GPIO INITIALIZATION AND DE-INITIALIZATION
/*******************************************************************************************************
 *  @fn				 	GPIO_Init
 *
 *  @brief				This function initializes the GPIO pin based on the configuration provided in the GPIO_Handle_t structure.
 *
 *  @param[in]			pGPIOHandle - Pointer to the GPIO handle structure containing the pin configuration.
 *
 *  @return				None
 *
 *  @note				None
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; 					//this is a temporary variable to hold the value of the GPIO port mode register

	//1. Configure the mode of the GPIO pin

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle -> pGPIOx -> MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clear the previous mode values
		pGPIOHandle -> pGPIOx -> MODER |= temp;
	}
	else
	{
		/*
		 * Things that need to be done when the pin is in interrupt mode
		 * 		1. Configure the GPIO pin to be in the input mode
		 * 		2. Configure the edge trigger (RT,FT,RFT) for the interrupt (Cab be done in the EXTI register)
		 * 		3. Enable the interrupt delivery from the peripheral to the processor (on the peripheral side)(done by
		 * 		   mask register of the EXTI)
		 * 		4. Identify the IRQ number on which the processor will be notified about the interrupt (refer the vector table)
		 * 		5. Configure the priority of the interrupt IRQ
		 * 		6. Enable interrupt reception on that IRQ number (in the NVIC)
		 * 		7. Implement the IRQ handler function in the application code
		 */
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	// Configure the falling edge trigger
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	// Disable the rising edge trigger
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	// Configure the rising edge trigger
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	// Disable the falling edge trigger
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	// Configure the rising edge trigger
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 	// Configure the falling edge trigger
		}

		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4; 		// Determine which EXTI configuration register to use (0, 1, 2, or 3)
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4; 		// Determine the pin number within the EXTI configuration register
		uint32_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx); 		// Get the port code (0 for A, 1 for B, etc.)
		SYSCFG_CLK_EN(); 														// Enable the clock for the SYSCFG peripheral
		SYSCFG -> EXTICR[temp1] = portcode << (4 * temp2); 						// Configure the EXTI configuration register for the pin

		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 		// Enable the interrupt mask for the pin
	}

	temp = 0;

	//2. Configure the speed of the GPIO pin

	temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear the previous speed setting
	pGPIOHandle -> pGPIOx -> OSPEEDR |= temp;

	temp = 0;
	//3. Configure the pull-up/pull-down control of the GPIO pin

	temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear the previous pull-up/pull-down setting
	pGPIOHandle -> pGPIOx -> PUPDR |= temp;

	temp = 0;
	//4. Configure the output type of the GPIO pin

	temp = pGPIOHandle -> GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx -> OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear the previous output type setting
	pGPIOHandle -> pGPIOx -> OTYPER |= temp;

	//5. Configure the alternate function mode of the GPIO pin if it is in alternate function mode
	if ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; // Determine which AFR register to use (0 or 1)
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; // Determine the pin number within the AFR register

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // Clear the previous alternate function
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); // Set the new alternate function
	}
}

/*******************************************************************************************************
 *  @fn				 	GPIO_DeInit
 *
 *  @brief				This function de-initializes the GPIO port by resetting its registers.
 *
 *  @param[in]			pGPIOx - Pointer to the GPIO port base address.
 *
 *  @return				None
 *
 *  @note				None
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

// APIs for GPIO read and write operations
/*******************************************************************************************************
 *  @fn				 	GPIO_WriteToOutputPin
 *
 *  @brief				This function writes a value to a specific GPIO pin.
 *
 *  @param[in]			pGPIOx - 	Pointer to the GPIO port base address.
 *  @param[in]			PinNumber - The pin number to write to.
 *  @param[in]			Value - 	SET(To set the pin or to make the pin high) or RESET(To reset the
 *  								pin or make the pin low) macros can be used.
 *
 *  @return				None
 *
 *  @note				None
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == SET)
	{
		pGPIOx->ODR |= (1 << PinNumber); // Set the pin
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber); // Reset the pin
	}
}

/*******************************************************************************************************
 *  @fn				 	GPIO_WriteToOutputPort
 *
 *  @brief				This function writes a value to the entire GPIO port.
 *
 *  @param[in]			pGPIOx - Pointer to the GPIO port base address.
 *  @param[in]			Value - The value to write to the port (16-bit value).
 *
 *  @return				None
 *
 *  @note				None
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	if(Value == SET)
	{
		pGPIOx->ODR |= Value; // Set the port
	}
	else
	{
		pGPIOx->ODR &= ~Value; // Reset the port
	}
}

/*******************************************************************************************************
 *  @fn				 	GPIO_ReadFromInputPin
 *
 *  @brief				This function reads the value from a specific GPIO pin.
 *
 *  @param[in]			pGPIOx - Pointer to the GPIO port base address.
 *  @param[in]			PinNumber - The pin number to read from.
 *
 *  @return				The value of the specified pin (SET or RESET).
 *
 *  @note				None
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value = 0;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001U);
	return value;
}

/*******************************************************************************************************
 *  @fn				 	GPIO_ReadFromInputPort
 *
 *  @brief				This function reads the value from the entire GPIO port.
 *
 *  @param[in]			pGPIOx - Pointer to the GPIO port base address.
 *
 *  @return				The value of the entire port (16-bit value).
 *
 *  @note				None
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value = 0;
	value = (uint16_t)pGPIOx->IDR; // Read the Input Data Register
	return value;
}

/*******************************************************************************************************
 *  @fn				 	GPIO_ToggleOutputPin
 *
 *  @brief				This function toggles the state of a specific GPIO pin.
 *
 *  @param[in]			pGPIOx - Pointer to the GPIO port base address.
 *  @param[in]			PinNumber - The pin number to toggle.
 *
 *  @return				None
 *
 *  @note				None
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	if((pGPIOx->ODR & (1 << PinNumber)) != 0)
	{
		pGPIOx->ODR &= ~(1 << PinNumber); // Reset the pin if it is currently set
	}
	else
	{
		pGPIOx->ODR |= (1 << PinNumber); // Set the pin if it is currently reset
	}
}

// APIs for GPIO interrupt configuration

/*******************************************************************************************************
 *  @fn				 	IRQ_Config
 *
 *  @brief				This function configures the interrupt for a specific GPIO pin.
 *
 *  @param[in]			IRQNumber - The IRQ number to configure.
 *  @param[in]			IRQPRIORITY - The priority of the IRQ.
 *  @param[in]			EnorDi-Enable or Disable the IRQ (The macro "ENABLE" will make it enable and macro "DISABLE" will make it disable).
 *
 *  @return				None
 *
 *  @note				None
 */
void IRQ_Config(uint8_t IRQNumber, uint8_t IRQPRIORITY, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber < 32)
		{
			// Enable the interrupt in the NVIC ISER register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber < 32)
		{
			*NVIC_ICER0 |= (1 << IRQNumber); // Disable the interrupt in the NVIC ICER register
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
	uint8_t iprx = IRQNumber / 4; // Determine the interrupt priority register index
	uint8_t iprx_section = IRQNumber % 4; // Determine the section within the interrupt priority register

	uint8_t shiftamount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED ); // Calculate the shift amount for the priority
	*(NVIC_IPR_BASEADDR + (iprx * 4)) |= (IRQPRIORITY << shiftamount ); // Set the priority of the interrupt
}

/*******************************************************************************************************
 *  @fn				 	IRQ_Handling
 *
 *  @brief				This function handles the interrupt for a specific GPIO pin.
 *
 *  @param[in]			PinNumber - The pin number for which the interrupt is being handled.
 *
 *  @return				None
 *
 *  @note				None
 */

void IRQ_Handling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber))			 // Check if the pending bit for the pin is set
	{
		printf("Inside ISR Handler \n") ;
		printf("Pin No: %d \n " , PinNumber) ;

		EXTI->PR |= (1 << PinNumber) ;			// Clear the pending bit for the pin
	}
}


