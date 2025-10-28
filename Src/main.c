
/******************************************************************************************
 * This is a main file for a simple GPIO toggle application using the driver we developed.
 *******************************************************************************************/
#include <stdint.h>

#include "stm32f401xx_gpio_drivers.h"

void delay()
{
	for(uint32_t i = 0; i < 100000 ; i++);								// Simple delay loop to create a visible toggle effect
}

int main(void)
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOC;												// GPIO port C is used for the LED
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;				// Pin 13 is used for the LED on the STM32F401CCU6 board
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;				// Set the pin mode to output
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;			// Set the output type to push-pull
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;		// No pull-up or pull-down resistor

	GPIO_PeriClockControl(GPIOC, ENABLE);								// Enable the clock for GPIO port C

	GPIO_Init(&GpioLed);												// Initialize the GPIO pin

    /* Loop forever */
	for(;;)
	{
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_13);						// Toggle the LED state
		delay();														// Delay to see the toggle effect
	}
}
