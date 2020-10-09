/*
 * 002led_button.c
 *
 *  Created on: Aug 26, 2020
 *      Author: clancytownsend
 */

#include "stm32f411xx.h"

#define LOW						0
#define HIGH					1
#define BTN_PRESSED 			LOW

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{

	GPIO_Handle_t GpioLed, GPIOButton;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PClckCtl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);


	GPIOButton.pGPIOx = GPIOB;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PClckCtl(GPIOB, ENABLE);
	GPIO_Init(&GPIOButton);


	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED)
		{
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
			delay();
		}


	}

	return 0;
}
