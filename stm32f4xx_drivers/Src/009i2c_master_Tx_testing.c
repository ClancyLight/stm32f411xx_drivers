/*
 * 009i2c_master_tx_testing.c
 *
 *  Created on: 2/10/2020
 *      Author: ClancyLight
 */

#include <stdio.h>
#include <string.h>
#include "stm32f411xx.h"

#define MY_ADDR			0x61
#define SLAVE_ADDR		0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
}

I2C_Handle_t I2C1Handle;


// some data to send to slave
uint8_t some_data[] = "0123456701234567012345670123456\n";
uint8_t some_data2[] = "Jingle all the way!\n";

/*
 * PB6 -> SCL
 * PB7 -> SDA
 */
void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFuncMode = 4;			// datasheet table 9 ;-)
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{

	//configure the pins
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR; //
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn, GPIOLed;

	// button
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	//LED
	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOLed);
}

int main(void)
{


	//gpio button,led init
//	GPIO_ButtonInit();


	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();





	while(1)
	{

		delay();
		delay();
		delay();
		delay();
		delay();

		//send some data to the slave
		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data),SLAVE_ADDR );

		delay();

		//send some data to the slave
		I2C_MasterSendData(&I2C1Handle, some_data2, strlen((char*)some_data2),SLAVE_ADDR );
	}

}
