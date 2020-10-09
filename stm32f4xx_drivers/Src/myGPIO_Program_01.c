/*
 * myGPIO_Program_01.c
 *
 *  Created on: 5/10/2020
 *      Author: clancytownsend
 */

#define LED_GREEN		GPIO_PIN_NO_12
#define LED_ORANGE		GPIO_PIN_NO_13
#define LED_RED		GPIO_PIN_NO_14
#define LED_BLUE		GPIO_PIN_NO_15


#include "stm32f411xx.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
}

void LED_GPIO_Init()
{
	GPIO_Handle_t ledPin;

	ledPin.pGPIOx = GPIOD;
	ledPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	ledPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ledPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	ledPin.GPIO_PinConfig.GPIO_PinNumber = LED_GREEN;
	GPIO_Init(&ledPin);
	ledPin.GPIO_PinConfig.GPIO_PinNumber = LED_ORANGE;
	GPIO_Init(&ledPin);
	ledPin.GPIO_PinConfig.GPIO_PinNumber = LED_RED;
	GPIO_Init(&ledPin);
	ledPin.GPIO_PinConfig.GPIO_PinNumber = LED_BLUE;
	GPIO_Init(&ledPin);
}

void Button_GPIO_Init()
{
	GPIO_Handle_t buttonPin;

	buttonPin.pGPIOx = GPIOA;
	buttonPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	buttonPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

	buttonPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIO_Init(&buttonPin);

	buttonPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&buttonPin);

}

int main(void)
{

	// initialise leds
	LED_GPIO_Init();

	Button_GPIO_Init();

	while(1)
	{
		while(!  GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_1) ); // will hang here until the pin gets high
		GPIO_ToggleOutputPin(GPIOD, LED_BLUE);
		GPIO_ToggleOutputPin(GPIOD, LED_ORANGE);
		GPIO_ToggleOutputPin(GPIOD, LED_RED);
		GPIO_ToggleOutputPin(GPIOD, LED_GREEN);

		delay();
	}



	return 0;
}
