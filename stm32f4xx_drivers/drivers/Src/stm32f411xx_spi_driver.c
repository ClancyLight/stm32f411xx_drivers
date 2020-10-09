/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: Aug 28, 2020
 *      Author: clancytownsend
 */
#include "stm32f411xx_spi_driver.h"


// keyword static to indicate these are private helper functions
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);


/******************************************************************************
 *  @fn				- SPI_PclcCtl
 *
 *  @brief			- This function enables/disables the peripheral clock for the given SPI port
 *
 *  @param[in]		- base address of the SPI peripheral
 *  @param[in]		- ENABLE or DISABLE macros
 * 	@param[in]		-
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */
void SPI_PclcCtl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	} else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}



/******************************************************************************
 *  @fn				- SPI_Init, SPI_DeInit
 *
 *  @brief			- Iinitialises/De-initialises the given SPI
 *
 *  @param[in]		- SPI handle structure
 *  @param[in]		-
 * 	@param[in]		-
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Enable peripheral clock
	SPI_PclcCtl(pSPIHandle->pSPIx, ENABLE);

	// Configure the SPI_CR1 register

	uint32_t tempreg = 0;

	// 1. Configure the device mode
	tempreg |= (pSPIHandle->SPIConfig.DeviceMode << SPI_CR1_MSTR );

	// 2. Configure the bus config
	if(pSPIHandle->SPIConfig.BusConfig == SPI_BUS_CONFIG_FD)
	{
		// FUll-duplex (BIDIMODE=0 and RXONLY=0)
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.BusConfig == SPI_BUS_CONFIG_HD)
	{
		// Half-Duplex
		tempreg |= ( 1 << SPI_CR1_BIDIMODE);

	} else if(pSPIHandle->SPIConfig.BusConfig == SPI_BUS_CONFIG_S_RXONLY)
	{
		// unidirectional receive-only mode (BIDIMODE=0 and RXONLY=1)
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
		tempreg |=  ( 1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the SPI serial clock speed
	tempreg |= pSPIHandle->SPIConfig.SclkSpeed << SPI_CR1_BR;

	// 4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.DFF << SPI_CR1_DFF;

	// 5. Configure the  SSM
	tempreg |= pSPIHandle->SPIConfig.SSM << SPI_CR1_SSM;

	// Set the SSI = 1
	tempreg |= pSPIHandle->SPIConfig.SSI << SPI_CR1_SSI;

	// 6. COnfigure the CPOL
	tempreg |= pSPIHandle->SPIConfig.CPOL << SPI_CR1_CPOL;

	// 7. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;

}
void SPI_DeInit(SPI_Handle_t *pSPIHandle)
{
	// TODO: deinit the spi
}


/*
 * @SPI_GetFlagStatus()
 * Waits until
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}


/*
 * SPI_SOEConfig
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE);
	}
}


/******************************************************************************
 *  @fn				- SPI_SendData
 *
 *  @brief			- Sends data from the SPI_DR register
 *
 *  @param[in]		- SPI register pointer
 *  @param[in]		- buffer pointer
 * 	@param[in]		- Length of the data
 *
 *  @return			- none
 *
 *  @Note			- 3 types: DMA, Interrupt and Polling (blocking)
 *  				  This function is a BLOCKING CALL (will block up the program until finished)
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// 1. wait until TXE is set
		//		while(!(pSPIx->SR & (1 << SPI_SR_TXE)));
		// More readable version below
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_FLAG_TXE) == FLAG_RESET);

		// 2. check the DFF bit in CR1
		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			// load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer); // dereferencing the DR as a 16 bit
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		} else
		{
			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*
 * SPI_PeripheralControl
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE);
	}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// 1. wait until TXE is set
		//		while(!(pSPIx->SR & (1 << SPI_SR_TXE)));
		// More readable version below
		while(SPI_GetFlagStatus(pSPIx, SPI_SR_FLAG_RXNE) == FLAG_RESET);

		// 2. check the DFF bit in CR1
		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			// load the data FROM the DR to RxBuffer
			*((uint16_t*)pRxBuffer)=  pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		} else
		{
			// 8 bit DFF
			*(pRxBuffer)=  pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//1. save the Tx buffer addr. and Len info (global variable)
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. mark the SPI state as busy in trans. so that no other code can take over same SPI peripheral
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

	}

	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		pSPIHandle->RxState = SPI_BUSY_IN_TX;
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
	}

	return state;
}

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;


	// CHeck for TXE
	temp1 = pSPIHandle->pSPIx->SR  & ( 1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// Check for RXNE
	temp1 = pSPIHandle->pSPIx->SR  & ( 1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);


	if(temp1 && temp2)
	{
		//handle TXE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// Check for RXNE
	temp1 = pSPIHandle->pSPIx->SR  & ( 1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);


	if(temp1 && temp2)
	{
		//handle overrun error
		spi_ovr_err_interrupt_handle(pSPIHandle);

	}



}

// helper function implementations
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if ( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF)))
	{
		//16 bit
		pSPIHandle->pSPIx->DR  = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit
		pSPIHandle->pSPIx->DR  = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//prevents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if ( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF)))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR ;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	}else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}

	if(! pSPIHandle->RxLen)
	{

		void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}


static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the overrun flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;

	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{

	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


__weak  void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	// THis is a weak implementation. the application may override this function (gcc attribute, weak)
}
