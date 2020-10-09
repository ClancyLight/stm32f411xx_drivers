/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: 14/09/2020
 *      Author: clancytownsend
 */


#include "stm32f411xx.h"

uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_Prescaler[4] = {2, 4, 8, 16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

/*
 * @I2C_PclcCtl
 * Enable peripheral clock control
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	} else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

uint32_t RCC_GetPLLOutputClock()
{
	return 1;
}

uint32_t RCC_GetPCLK1Value()
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1;

	clksrc = (RCC->CFGR >> 2) & 0x3; // masked with 0x3 to zero out all except LSB 2 bits.

	if(clksrc == 0) // clock source = HSI
	{
		SystemClk = 16000000;
	} else if(clksrc == 1) // HSE
	{
		SystemClk = 8000000;
	} else if(clksrc == 2) // PLL
	{
		SystemClk = RCC_GetPLLOutputClock();
	}


	// ahbp prescaler
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	} else
	{
		ahbp = AHB_Prescaler[temp - 8];
	}

	//apb1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1 = 1;
	} else
	{
		apb1 = APB1_Prescaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apb1;


	return pclk1;
}


/*
 * Init and De-Init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ack control bit
	tempreg |=  pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg  = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	tempreg = 0;
	// program the device own address
	tempreg  = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	// bit 14 of the OAR1 register should be kept at 1 (ref manual says, but doesn't say why)
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed) );
		tempreg |= (ccr_value & 0xFFF);
	} else
	{
		// fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}else
		{
			ccr_value = RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	tempreg = 0;
	//TRISE register
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// standard mode

		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;     // (F_pclk1 * Trise) + 1; Trise_max = 1000ns
	}else
	{
		//fast mode
		tempreg = ( ( RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);


}

void I2c_DeInit(I2C_Handle_t *pI2CHandle)
{

}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that the start geneartion is completed by checking the SB flag in the SR1 regsiter
	// 	Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to R(1) total 8 bits
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. wait until the address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	// procedure to read only 1 byte from the slave
	if(Len == 1)
	{
		 // Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);


		//clear the Addr flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		// wait until RxNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//generate STOP condition
		if(Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data in to the buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}

	if(Len > 1)
	{
		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//read the data until Len becomes zero
		for(uint32_t i = Len; i > 0; i--)
		{
			// wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));


			if(i == 2) // last 2 bytes are remaining
			{
				// clear the ack bit (disable acking)
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);


				//generate stop condition (if no repeated start)
				if(Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}

			// read the data from data registers in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}

	// re-anable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}

}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	// 1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that the start generation is completed by checking the SB flag in SR1
	// NOte: until SB is cleared, SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	// 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. confirm the address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// 5. clear the ADDR flag according to its software sequence
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. send the data until Len becomes 0  (check the TxE flag)
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // wait until TXE is set
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	// 7. when Len becomes zero, wait for TXE=1 and BTF=1 before generating the STOP condition
	// NOTE: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin.
	// 		 when BTF=1, SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of the stop condition
	// NOte: generating STOP automatically clears the BTF
	if(Sr == I2C_DISABLE_SR) // if no repeated start
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}



}



void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}

}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}else
	{
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}

