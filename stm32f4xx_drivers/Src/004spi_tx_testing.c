/*
 * 004spi_tx_testing.c
 *
 *  Created on: Aug 28, 2020
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
	for(uint32_t i = 0; i <= 5000000; i++);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
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
	SPI2handle.SPIConfig.SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.DFF = SPI_DFF_BITS_8;
	SPI2handle.SPIConfig.CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SSM = SPI_SSM_EN;
	SPI2handle.SPIConfig.SSI = SPI_SSI_SET;

	SPI_Init(&SPI2handle);

}

int main(void)
{
	char user_data[] = "Hello Wendy";

	// initialise the GPIO pins for SPI2 functionality
	SPI2_GPIOInits();

	// initialise the SPI2 peripheral params
	SPI2_Inits();

	// Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	// send data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//confirm that SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_SR_FLAG_BUSY));

	// disable the peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	return 0;
}
