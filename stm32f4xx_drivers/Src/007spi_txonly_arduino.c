/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: Aug 29, 2020
 *      Author: ClancyLight
 */

#include "stm32f411xx.h"
#include <string.h>

/*
 *  SPI gpio pins (alternate function
 *
 *  spi wire         [pin : alternate function]
 *  SPI2_NSS -   	[PB12 : AF05]
 *  SPI2_SCLK	 	[PB13 : AF05]
 *  SPI2_MISO		[PB14 : AF05]
 *  SPI2_MOSI		[PB15 : AF05]
 */

void delay()
{
	for(uint32_t i = 0; i <= 5000000/2; i++);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SclkSpeed = SPI_SCLK_SPEED_DIV8; // Sclk of 2Mhz
	SPI2handle.SPIConfig.DFF = SPI_DFF_BITS_8;
	SPI2handle.SPIConfig.CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SSM = SPI_SSM_DI;
	SPI2handle.SPIConfig.SSI = SPI_SSI_RESET;

	SPI_Init(&SPI2handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOButton;

	GPIOButton.pGPIOx = GPIOA;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOButton);
}

int main(void)
{
	char user_data[] = "Hello Wendy";

	GPIO_ButtonInit();

	// initialise the GPIO pins for SPI2 functionality
	SPI2_GPIOInits();

	// initialise the SPI2 peripheral params
	SPI2_Inits();

	/*
	 * making SSOE 1 does NSS output enable.
	 * the NSS pin is automatically managed by the hardware.
	 * when SPE=1, NSS wil lbe pulled low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// firstly send length info
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		// send data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		//confirm that SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_SR_FLAG_BUSY));

		// disable the peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
		delay();

	}

	return 0;
}
