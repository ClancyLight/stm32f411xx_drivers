/*
 * stm32f411xx.h
 *
 *  Created on: Aug 25, 2020
 *      Author: clancytownsend
 *
 * Notes: mcu specific header file
 *
 *		Reference Manual: RM0383
 *		Datasheet: STM32F411XC/E
 *		Programming Manual: PM0214
 *
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))


/***********************************************Processor Specific Details*******************************
 *
 *
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 *
 */
#define NVIC_ISER0					((__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3					((__vo uint32_t*)0xE000E10C )

/*
 * Arm Cortex ICERx register addresses
 */
#define NVIC_ICER0					((__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1					((__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2					((__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3					((__vo uint32_t*)0xE000E18C )


/*
 * ARM Cortex Priority register base address
 */
#define NVIC_IPR_BASE_ADDR			((__vo uint32_t*)0xE000E400)  // IPR59 address = 0xE000E4EF


/*
 * ARM Cortex Priority bits implemented
 */
#define NO_PRIORITY_BITS_IMPLEMENTED		4

/*
 * Base addresses
 */

#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR					0x20000000U
#define SRAM2_BASEADDR 					0x2000IC00U
#define ROM_BASEADDR					0x1FFF0000U
#define SRAM							SRAM1_BASEADDR


#define PERIPH_BASE						0x40000000U
#define APB1PERIPH_BASE					PERIPH_BASE
#define APB2PERIPH_BASE					0x40010000U
#define AHB1PERIPH_BASE					0x40020000U
#define AHB2PERIPH_BASE					0x50000000U

#define GPIOA_BASEADDR				(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASE + 0x1C00)
#define RCC_BASEADDR				(AHB1PERIPH_BASE + 0x3800)

#define I2C1_BASEADDR				(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASE + 0x5C00)

#define SPI1_BASEADDR				(APB2PERIPH_BASE + 0x3000)
#define SPI2_BASEADDR				(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASE + 0x3C00)
#define SPI4_BASEADDR				(APB2PERIPH_BASE + 0x3400)

#define USART2_BASEADDR				(APB1PERIPH_BASE + 0x4400)


#define EXTI_BASEADDR				(APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASE + 0x3800)







//Peripheral register definition structures


typedef struct
{
	__vo uint32_t MODER;				// 0x00
	__vo uint32_t OTYPER;				// 0x04
	__vo uint32_t OSPEEDR;				// 0x08
	__vo uint32_t PUPDR;				// 0x0C
	__vo uint32_t IDR;					// 0x10
	__vo uint32_t ODR;					// 0x14
	__vo uint32_t BSRR;					// 0x18
	__vo uint32_t LCKR;					// 0x1C
	__vo uint32_t AFR[2]; 				// 0x20, 0x24: AFR[0]: low, AFR[1]: high
}GPIO_RegDef_t;


typedef struct
{
	__vo uint32_t CR;					// 0x00
	__vo uint32_t PLLCFGR;				// 0x04
	__vo uint32_t CFGR;					// 0x08
	__vo uint32_t CIR;					// 0x0C
	__vo uint32_t AHB1RSTR;				// 0x10
	__vo uint32_t AHB2RSTR;				// 0x14
	__vo uint32_t RESERVED0[2]; 		// 0x18, 0x1C reserved
	__vo uint32_t APB1RSTR;				// 0x20
	__vo uint32_t APB2RSTR;				// 0x24
	__vo uint32_t RESERVED1[2];			// 0x28, 0x2C reserved
	__vo uint32_t AHB1ENR;				// 0x30
	__vo uint32_t AHB2ENR;				// 0x34
	__vo uint32_t RESERVED2[2];			// 0x38, 0x3C reserved
	__vo uint32_t APB1ENR;				// 0x40
	__vo uint32_t APB2ENR;				// 0x44
	__vo uint32_t RESERVED3[2];			// 0x48, 0x4C reserved
	__vo uint32_t AHB1LPENR;			// 0x50
	__vo uint32_t AHB2LPENR;			// 0x54
	__vo uint32_t RESERVED4[2];			// 0x58, 0x5C reserved
	__vo uint32_t APB1LPENR;			// 0x60
	__vo uint32_t APB2LPENR;			// 0x64
	__vo uint32_t RESERVED5[2];			// 0x68, 0x6C
	__vo uint32_t BDCR;					// 0x70
	__vo uint32_t CSR;					// 0x74
	__vo uint32_t RESERVED6[2];			// 0x78, 0x7C
	__vo uint32_t SSCGR;				// 0x88
	__vo uint32_t PLLI2SCFGR;			// 0x84
	__vo uint32_t RESERVED7;			// 0x88
	__vo uint32_t DCKCFGR;				// 0x8C

}RCC_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;					// 0x00
	__vo uint32_t CR2;					// 0x04
	__vo uint32_t OAR1;					// 0x08
	__vo uint32_t OAR2;					// 0x0C
	__vo uint32_t DR;					// 0x10
	__vo uint32_t SR1;					// 0x14
	__vo uint32_t SR2;			 		// 0x18
	__vo uint32_t CCR;					// 0x1C
	__vo uint32_t TRISE;				// 0x20
	__vo uint32_t FLTR;					// 0x24

}I2C_RegDef_t;


/*
 * EXTI peripheral structure (reference manual:
 */

typedef struct
{
	__vo uint32_t IMR;					// 0x00
	__vo uint32_t EMR;					// 0x04
	__vo uint32_t RTSR;					// 0x08
	__vo uint32_t FTSR;					// 0x0C
	__vo uint32_t SWIER;				// 0x10
	__vo uint32_t PR;					// 0x14

}EXTI_RegDef_t;


/*
 * SYSCFG peripheral structure
 */
typedef struct
{
	__vo uint32_t MEMRMP;				// 0x00
	__vo uint32_t PMC;					// 0x04
	__vo uint32_t EXTICR[4];			// 0x08-0x14
	__vo uint32_t RESERVED[2];			// 0x18-0x1C
	__vo uint32_t CMPCR;				// 0x20

}SYSCFG_RegDef_t;

/*
 * Register Definition for SPI
 */
typedef struct
{
	uint32_t CR1;					// 0x00
	uint32_t CR2;					// 0x04
	uint32_t SR;					// 0x08
	uint32_t DR;					// 0x0C
	uint32_t CRCPR;					// 0x10
	uint32_t RXCRCR;				// 0x14
	uint32_t TXCRCR;				// 0x18
	uint32_t I2SCFGR;				// 0x1C
	uint32_t I2SPR;					// 0x20
}SPI_RegDef_t;


#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)


#define RCC 		((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)


// I2C peripheral definition macros
#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASEADDR)
// clock enable macros for GPIOx peripherals

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 0) )
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 1) )
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 2) )
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 3) )
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 4) )
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= ( 1 << 7) )


// clock enable macros for I2Cx peripherals

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 23))


// clock enable macros for SPIx peripherals

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= ( 1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= ( 1 << 13))

// clock enable macros for USARTx peripherals

#define USART2_PCLK_EN()	(RCC->APB1ENR |= ( 1 << 17))



// clock enable macros for SYSCFG peripheral

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 14))






// clock DISABLE macros for GPIOx peripherals

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 0) )
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 1) )
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 2) )
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 3) )
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 4) )
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 7) )



// clock DISABLE macros for I2Cx peripherals

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 23))


// clock DISABLE macros for SPIx peripherals

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~( 1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~( 1 << 13))

// clock DISABLE macros for USARTx peripherals

#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~( 1 << 17))



// clock DISABLE macros for SYSCFG peripheral

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~( 1 << 14))



/*
 * Macros to reset GPIOx peripherals
 * Sets the bit, then clears it.
 */
#define GPIOA_REG_RESET() 				do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET() 				do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET() 				do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET() 				do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET() 				do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET() 				do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)


#define GPIO_BASEADDR_TO_CODE(x)			((x == GPIOA) ? 0 :\
											 (x == GPIOB) ? 1 :\
											 (x == GPIOC) ? 2 :\
											 (x == GPIOD) ? 3 :\
											 (x == GPIOE) ? 4 :\
											 (x == GPIOH) ? 7 :0)


/*
 * EXTI_IRQ (External Interrupt request) numbers for STM32F411E-Disco Mcu
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*
 * SPI-related IRQ numbers
 */


/*
 * IRQ priorities
 *
 * note: lower number = higher priority
 */

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI15		15



// some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET					ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET


/*************************************************************
 * Bit position definitions of SPI peripheral (from reference manual)
 ************************************************************/

// SPI_CR1  (Control Register 1)
#define SPI_CR1_CPHA		0	/* Clock phase. (0: first clock transition is the first data capture edge */
#define SPI_CR1_CPOL		1   /* Clock polarity(0: CK to 0 when idle, 1: CK to 1 when idle)*/
#define SPI_CR1_MSTR		2	/* Masster selection(0:slave config, 1: master config)*/
#define SPI_CR1_BR			3   /* Baud Rate control BR[2:0] */
#define SPI_CR1_SPE			6	/* SPI enable(0: disabled, 1: enabled) */
#define SPI_CR1_LSBFIRST	7	/* Frame format(0: MSB transmitted first, 1: LSB first)*/
#define SPI_CR1_SSI			8	/* SSI Internal slave select*/
#define SPI_CR1_SSM			9	/* Software slave management (0: diabled, 1: enabled*/
#define SPI_CR1_RXONLY		10	/* Receive only (0: full-duplex tx and rx, 1: output disabled rx only mode) */
#define SPI_CR1_DFF			11	/* Data Frame Format (0: 8-bit, 1: 16-bit)*/
#define SPI_CR1_CRCNEXT		12	/* (0: data phase - no CRC , 1: next tx phase - CRC phase)*/
#define SPI_CR1_CRCEN		13	/* Hware CRC calculation (0: CRC calc diabled, 1: CRC enabled*/
#define SPI_CR1_BIDIOE		14	/* Output enable in bi-directional mode (0: output diabled (rx only mode), 1: out enabled tx-only mode)*/
#define SPI_CR1_BIDIMODE	15	/* Bi-directional data mode enable (0: 2-line unidirectional, 1: 1-line bi-directional)*/

// SPI_CR2   (Control Register 2)
#define SPI_CR2_RXDMAEN		0	/* Rx buffer DMA enable (0: disabled, 1: enabled)*/
#define SPI_CR2_TXDMAEN		1	/* Tx buffer DMA enable (0: DMA disabled, 1: DMA enabled)*/
#define SPI_CR2_SSOE		2	/* SS output enable (0: SS output disabled, 1: enabled)*/
#define SPI_CR2_FRF			4	/* Frame format (0: SPI motorola mode, 1: SPI TI mode)*/
#define SPI_CR2_ERRIE		5	/* Error interrupt enable (0: masked, 1: not masked)*/
#define SPI_CR2_RXNEIE		6	/* RX buffer not empty (0: RXNE masked, 1: RXNE not masked) */
#define SPI_CR2_TXEIE		7	/* Tx buffer empty intupt enable (0: TXE intrpt masked, 1: TXE inpt not masked)*/


// SPI_SR  (Status Register)
#define SPI_SR_RXNE			0 /* rx buffer not empty (0: rx empty, 1: rx not empty) */
#define SPI_SR_TXE			1 /* tx buffer empty (0: tx buffer not empty, 1: tx buffer empty)*/
#define SPI_SR_CHSIDE		2	/* Channel side (0: channel left, 1: channel right) */
#define SPI_SR_UDR			3	/* Underrun flag (0: no underrun, 1: underrun)*/
#define SPI_SR_CRCERR		4	/* CRC error flag (0: CRC val matched SPI_RXCRCR, 1: not match)*/
#define SPI_SR_MODF			5	/* Mode fault (0: no fault, 1: fault occ..)*/
#define SPI_SR_OVR			6	/* Overrun flag (0: no overrun occurred, 1: overrun occ..)*/
#define SPI_SR_BSY			7	/* Busy flag (0: SPI not busy, 1: SPI busy or Tx buffer not empty)*/
#define SPI_SR_FRE			8	/* Frame format error (0: no error, 1: error occurred)*/

/*************************************************************
 * Bit position definitions of I2C peripheral (from ref man)
 ************************************************************/

// I2C_CR1
#define I2C_CR1_PE			0   // Peripheral enable
#define I2C_CR1_SMBUS		1 	// SMBus mode
#define I2C_CR1_SMBTYPE		3	// SMBus type (0: device, 1: host)
#define I2C_CR1_ENARP		4	// ARP enable
#define I2C_CR1_ENPEC		5	// PEC enable
#define I2C_CR1_ENGC		6	// General call enable
#define I2C_CR1_NOSTRETCH	7	// clock stretching (0:enabled, 1: disabled)  (slave mode)
#define I2C_CR1_START		8	// start generation
#define I2C_CR1_STOP		9	// stop generation
#define I2C_CR1_ACK			10	// ack enable (0: no ack returned, 1: ack returned after a byte is rcvd)
#define I2C_CR1_POS			11	// ack/pec position
#define I2C_CR1_PEC			12	// packet error checking
#define I2C_CR1_ALERT		13	// SMBus alert
#define I2C_CR1_SWRST		15	// Software reset

// I2C CR2
#define I2C_CR2_FREQ		0	// [5:0] peripheral clock frequency
#define I2C_CR2_ITERREN		8	// Error interrupt enable
#define I2C_CR2_ITEVTEN		9	// event interrupt enable
#define I2C_CR2_ITBUFEN		10	// Buffer interrupt enable
#define I2C_CR2_DMAEN		11	// DMA requests enable
#define I2C_CR2_LAST		12	// DMA last transfer

// I2C SR1
#define I2C_SR1_SB			0	// Start bit (master mode)
#define I2C_SR1_ADDR		1	// Address sent (master mode)/matched (slave mode)
#define I2C_SR1_BTF			2	// Byte transfer finished
#define I2C_SR1_ADD10		3	// 10-bit header sent (Master mode)
#define I2C_SR1_STOPF		4	// stop detection (slave mode)
#define I2C_SR1_RxNE		6	// Data regiser not empty (receivers)
#define I2C_SR1_TxE			7	// Data register empty
#define I2C_SR1_BERR		8	// Bus error
#define I2C_SR1_ARLO		9	// Arbitration lost (master mode)
#define I2C_SR1_AF			10	// Ack failure
#define I2C_SR1_OVR			11	// Overrun/Underrun
#define I2C_SR1_PECERR		12	// PEC Error in reception
#define I2C_SR1_TIMEOUT		14	// Timeout or Tlow error
#define I2C_SR1_SMBALERT	15	// SMBus alert

// I2C SR2
#define I2C_SR2_MSL			0	// Master/Slave (0: slave mode, 1: master mode)
#define I2C_SR2_BUSY		1	// Bus busy
#define I2C_SR2_TRA			2	// Transmitter/receiver (0: data bytes rx'd, 1: data bytes tx'd)
#define I2C_SR2_GENCALL		4	// General call address (slave mode)
#define I2C_SR2_SMBDEFAULT	5	// SMBus device default address (slave mode)
#define I2C_SR2_SMBHOST		6	// SMBus host header (Slave mode)
#define I2C_SR2_DUALF		7	// Dual flag (slave mode)
#define I2C_SR2_PEC			8	// [7:0] Packet error checking regiser

// I2C CCR
#define I2C_CCR_CCR			0	// Clock control register in Fm/Sm mode (master mode)
#define I2C_CCR_DUTY		14	// Fm mode duty cycle (0: tlow/thigh = 2, 1: tlow/thigh = 16/9)
#define I2C_CCR_FS			15	// I2C master mode selection (0: Sm mode I2C, 1: Fm mode I2C)


#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_i2c_driver.h"

#endif /* INC_STM32F411XX_H_ */
