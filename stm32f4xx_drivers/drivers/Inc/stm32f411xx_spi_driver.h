/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: Aug 28, 2020
 *      Author: clancytownsend
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include "stm32f411xx.h"

/*
 * Config structure for SPIx peripheral
 */
typedef struct
{
	uint8_t DeviceMode;
	uint8_t BusConfig;
	uint8_t SclkSpeed;
	uint8_t DFF;
	uint8_t CPOL;
	uint8_t CPHA;
	uint8_t SSM;
	uint8_t SSI;
}SPI_Config_t;



/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t	*pSPIx;			// holds the base address of SPIx peripheral
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxState;
	uint8_t			RxState;
}SPI_Handle_t;


/*
 * @SPI_STATE
 */
#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2

/*
 * Possible SPI application events
 */
#define SPI_EVENT_TX_CMPLT			1
#define SPI_EVENT_RX_CMPLT			2
#define SPI_EVENT_OVR_ERR			3
#define SPI_EVENT_CRC_ERR			4
/*
 * @SPI_DeviceMode
 */

#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD			1	/* full-duplex 				*/
#define SPI_BUS_CONFIG_HD			2	/* half-duplex 				*/
#define SPI_BUS_CONFIG_S_RXONLY		3	/* simplex: receive  only	*/

/*
 * @SPI_SclkSPeed
 * Serial Clock Speed
 */
#define SPI_SCLK_SPEED_DIV2			0 /* divides the clock by 2	*/
#define SPI_SCLK_SPEED_DIV4			1 /* divides the clock by 4	*/
#define SPI_SCLK_SPEED_DIV8			2 /* etc... */
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * @SPI_DFF
 * Data Frame Format
 */
#define SPI_DFF_BITS_8				0
#define SPI_DFF_BITS_16				1


/*
 * @SPI_CPOL
 * Clock Polarity
 */
#define SPI_CPOL_HIGH				1
#define SPI_CPOL_LOW				0 /* Clock is zero when idle */


/*
 * @CPHA
 * CLock Phase
 */
#define SPI_CPHA_HIGH				1
#define SPI_CPHA_LOW				0


/*
 * @SPI_SSM
 * Software Slave Management
 */
#define SPI_SSM_EN					1
#define SPI_SSM_DI					0 /* Disabled by default */
#define SPI_SSI_SET					1
#define SPI_SSI_RESET				0


/*
 * SPI related status flag definitions
 */
#define SPI_SR_FLAG_TXE				( 1 << SPI_SR_TXE)
#define SPI_SR_FLAG_RXNE			( 1 << SPI_SR_RXNE)
#define SPI_SR_FLAG_BUSY			( 1 << SPI_SR_BSY)


/***********************************************************************************************************************
*										APIs supported by this driver
*
************************************************************************************************************************/
/*
 * Peripheral Clock Setup
 */
void SPI_PclcCtl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);



/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_Handle_t *pSPIHandle);

/*
 * Data Send and Receive
 * 3 types.  DMA, Interrupt, polling.
 * This one is a polling (blocking) implementation
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Data Send and receive wither Interrupts
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


/*
 * Other peripheral control APIs
 */


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);












#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
