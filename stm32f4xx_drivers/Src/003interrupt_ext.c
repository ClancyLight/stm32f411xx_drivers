/*
 * 003interrupt_ext.c
 *
 *  Created on: Aug 27, 2020
 *      Author: ClancyLight
 */


#include <string.h> // for memset
#include "stm32f411xx.h"

#define LOW						0
#define HIGH					1
#define BTN_PRESSED 			LOW
uint8_t divider = 2;
uint8_t flag = 1;

void delay(void)
{
	for(uint32_t i = 0; i < 500000/divider; i++);
}

int main(void)
{

	GPIO_Handle_t GpioLed, GPIOButton, GPIOButton2;
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GPIOButton,0,sizeof(GPIOButton));
	memset(&GPIOButton2,0,sizeof(GPIOButton2));

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_PClckCtl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);


	GPIOButton.pGPIOx = GPIOD;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;		// interrupt falling edge
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	GPIO_PClckCtl(GPIOD, ENABLE);
	GPIO_Init(&GPIOButton);

	GPIOButton2.pGPIOx = GPIOA;
	GPIOButton2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOButton2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOButton2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GPIOButton2);


	// IRQ configurations
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);


	// Second IRQ config
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI0);


	while(1)
	{
		flag = 1;
//		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
	}


	return 0;
}

/*
 * This is the ISR
 */
void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_8, SET);

	while(flag)
	{
		// wait here until flag
	}

}

void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_8, RESET);
	flag = 0;
}


