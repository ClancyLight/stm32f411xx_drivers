/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Aug 25, 2020
 *      Author: clancytownsend
 */


#include "stm32f411xx_gpio_driver.h"


/******************************************************************************
 *  @fn				- GPIO_PClckCtl
 *
 *  @brief			- This function enables/disables the peripheral clock for the given GPIO port
 *
 *  @param[in]		- base address of the gpio peripheral
 *  @param[in]		- ENABLE or DISABLE macros
 * 	@param[in]		-
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}


	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/******************************************************************************
 *  @fn				- GPIO_Init
 *
 *  @brief			- This function enables/disables the peripheral clock for the given GPIO port
 *
 *  @param[in]		- base address of the gpio peripheral
 *  @param[in]		- ENABLE or DISABLE macros
 * 	@param[in]		-
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */
//void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
//{
//	 uint32_t temp=0; //temp. register
//
//	 //enable the peripheral clock
//
//	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
//
//	//1 . configure the mode of gpio pin
//
//	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
//	{
//		//the non interrupt mode
//		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
//		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
//		pGPIOHandle->pGPIOx->MODER |= temp; //setting
//
//	}else
//	{
//		//this part will code later . ( interrupt mode)
//		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_FT )
//		{
//			//1. configure the FTSR
//			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//			//Clear the corresponding RTSR bit
//			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//
//		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT )
//		{
//			//1 . configure the RTSR
//			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//			//Clear the corresponding RTSR bit
//			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//
//		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT )
//		{
//			//1. configure both FTSR and RTSR
//			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//			//Clear the corresponding RTSR bit
//			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
//		}
//
//		//2. configure the GPIO port selection in SYSCFG_EXTICR
//		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
//		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
//		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
//		SYSCFG_PCLK_EN();
//		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);
//
//		//3 . enable the exti interrupt delivery using IMR
//		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
//	}
//
//	//2. configure the speed
//	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
//	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
//	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
//
//	//3. configure the pupd settings
//	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
//	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
//	pGPIOHandle->pGPIOx->PUPDR |= temp;
//
//
//	//4. configure the optype
//	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
//	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
//	pGPIOHandle->pGPIOx->OTYPER |= temp;
//
//	//5. configure the alt functionality
//	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
//	{
//		//configure the alt function registers.
//		uint8_t temp1, temp2;
//
//		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
//		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
//		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
//		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << ( 4 * temp2 ) );
//	}
//
//}
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;


	// ENable the peripheral clock (because users usually forget elsewhere)
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);


	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clear the 2 bit fields first
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// set the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// clear the RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// set the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// clear the FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// set the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// set the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		// gpio port selection SYSCFG_EXTICR

		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);


		// enable EXTI IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;
	// speed
	temp =  (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed  << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;


	// pupd
	temp =  (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl  << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	// optype
		temp =  (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType  << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER |= temp;

		temp = 0;

	// alt func
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// configure the alt function registers
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2 ) );
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4 * temp2 ) );
	}
}

/******************************************************************************
 *  @fn				- GPIO_DeInit
 *
 *  @brief			- This function enables/disables the peripheral clock for the given GPIO port
 *
 *  @param[in]		- base address of the gpio peripheral
 *  @param[in]		-
 * 	@param[in]		-
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
		if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		}
}

/******************************************************************************
 *  @fn				- GPIO_ReadFromInputPin
 *
 *  @brief			- Read from a given port and pin number
 *
 *  @param[in]		- base address of the gpio peripheral
 *  @param[in]		- pin number
 * 	@param[in]		-
 *
 *  @return			- 0 or 1
 *
 *  @Note			- none
 *
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value =  (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001) ;


	return value;
}


/******************************************************************************
 *  @fn				- GPIO_ReadFromInputPort
 *
 *  @brief			- Read from a given port (the whole port)
 *
 *  @param[in]		- base address of the gpio peripheral
 *  @param[in]		-
 * 	@param[in]		-
 *
 *  @return			- returns the value of the whole port
 *
 *  @Note			- none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/******************************************************************************
 *  @fn				- GPIO_WriteToOutputPin
 *
 *  @brief			- write to a VALUE, to a given PORT and PIN number
 *
 *  @param[in]		- base address of the gpio peripheral
 *  @param[in]		- pin number
 * 	@param[in]		- value to write
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |=  ( 1 << PinNumber);
	} else
	{
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}

/******************************************************************************
 *  @fn				- GPIO_WriteToOutputPort
 *
 *  @brief			- write to a VALUE, to a given PORT
 *
 *  @param[in]		- base address of the gpio peripheral
 *  @param[in]		- value to write
 * 	@param[in]		-
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/******************************************************************************
 *  @fn				- GPIO_ToggleOutputPin
 *
 *  @brief			- flip the value on the given port and pint
 *
 *  @param[in]		- base address of the gpio peripheral
 *  @param[in]		- pin number
 * 	@param[in]		-
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR = pGPIOx->ODR ^ ( 1 << PinNumber);

}

/******************************************************************************
 *  @fn				- GPIO_IRQInterruptConfig
 *
 *  @brief			-
 *
 *  @param[in]		- Interrupt Request number
 *  @param[in]		- enable or disable
 * 	@param[in]		-
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// ISER1
			*NVIC_ISER1 |= ( 1 << ( IRQNumber % 32 ) );

		} else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// ISER2
			*NVIC_ISER2 |= ( 1 << ( IRQNumber % 64 ) );
		}
	} else
	{
		if(IRQNumber <= 31)
		{
			// ISER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// ISER1
			*NVIC_ICER1 |= ( 1 << ( IRQNumber % 32 ) );

		} else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// ISER2
			*NVIC_ICER2 |= ( 1 << ( IRQNumber % 64 ) );
		}
	}
}

/******************************************************************************
 *  @fn				- GPIO_IRQPriorityConfig
 *
 *  @brief			-
 *
 *  @param[in]		-
 *  @param[in]		-
 * 	@param[in]		-
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;		// truncation on purpose, gets the right IPR number
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PRIORITY_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASE_ADDR + iprx ) |=  (IRQPriority << shift_amount );  // iprx * 4 gives the right memory location

}


/******************************************************************************
 *  @fn				- GPIO_IRQHandling
 *
 *  @brief			-
 *
 *  @param[in]		-
 *  @param[in]		-
 * 	@param[in]		-
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		EXTI->PR |= ( 1 << PinNumber);
	}
}
