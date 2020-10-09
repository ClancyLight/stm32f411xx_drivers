/*
 * 008spi_cmd_handling.c
 *
 *  Created on: 30/08/2020
 *      Author: clancytownsend
 */

#include "stm32f411xx.h"
#include <string.h>
#include <stdio.h>

/*																22222222222552+9+9++9++++9+6+6+696+6+6696595
 * COmmand codes (for arduino slave)
 */
#define NACK 					0xA5
#define ACK 					0xF5
#define COMMAND_LED_CTRL        0x50
#define COMMAND_SENSOR_READ     0x51
#define COMMAND_LED_READ        0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54
#define LED_ON     				1
#define LED_OFF    				0
#define ANALOG_PIN0				0
#define ANALOG_PIN1   			1
#define ANALOG_PIN2   			2
#define ANALOG_PIN3   			3
#define ANALOG_PIN4   			4
#define LED_PIN					9

void delay()
{
	for(uint32_t i = 0; i <= 5000000/2; i++);
}
/*
 *  SPI gpio pins (alternate function
 *  spi wire        [pin  : alternate function]
 *  SPI2_NSS -   	[PB12 : AF05]
 *  SPI2_SCLK	 	[PB13 : AF05]
 *  SPI2_MISO		[PB14 : AF05]
 *  SPI2_MOSI		[PB15 : AF05]
 */
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

/*
 * Verify the response from slave (ack or nack)
 */
uint8_t SPI_VerifyResponse(uint8_t ack_byte)
{
	if(ack_byte == ACK)
	{
		return 1;
	}

	return 0;
}


int main(void)
{
	uint8_t count = 0;
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	printf("hello world, again\n");
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
		if(count == 0)
		{
			count = 1;
		}else
		{
			count = 0;
		}
		// wait for button to be pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		//debounce
		delay();


		//enable SPI2 periph
		SPI_PeripheralControl(SPI2, ENABLE);


		// CMD_LED_CTR		<pin no(1)>		<value(1)>
		uint8_t cmd_code = COMMAND_LED_CTRL;
		uint8_t args[2];
		uint8_t ack_byte;


		// send command
		SPI_SendData(SPI2, &cmd_code, 1);

		// clear RXNE by dummy read
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send some dummy data to fetch the response from slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ACK byte response from slave
		SPI_ReceiveData(SPI2, &ack_byte, 1);

		// if Acknowledged (command is recognised), then send the args
		if(SPI_VerifyResponse(ack_byte))
		{
			args[0] = LED_PIN;

			if(count == 1)
			{
				args[1] = LED_ON;
			} else
			{
				args[1] = LED_OFF;
			}


			SPI_SendData(SPI2, args, 2);
		}



		// 2. CMD_SENSOR_READ  <analog pin number(1)>
		// wait for button to be pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		//debounce
		delay();

		cmd_code = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &cmd_code, 1);

		// clear RXNE by dummy read, then dummy write to prepare receive
		SPI_ReceiveData(SPI2, &dummy_read, 1);
		SPI_SendData(SPI2, &dummy_write, 1);

		SPI_ReceiveData(SPI2, &ack_byte, 1);

		if(SPI_VerifyResponse(ack_byte))
		{
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);

			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// Insert a delay, so that the slave has time to read the sensor and prepare the value
			delay();

			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
			int display = analog_read;
			printf("Here: %d\n",display);
		}

		//confirm SPI2 not busy
		while(SPI_GetFlagStatus(SPI2, SPI_SR_FLAG_BUSY));

		// disable the SPI2 periph
		SPI_PeripheralControl(SPI2, DISABLE);
	}
	return 0;
}

