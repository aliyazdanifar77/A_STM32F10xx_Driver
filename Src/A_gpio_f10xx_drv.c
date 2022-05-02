/*
 * A_gpio_f10xx_drv.c
 *
 *  Created on: Feb 2, 2021
 *      Author: Ali Yazdanifar
 *      note : 1-
 *
 */

#include "A_gpio_f10xx_drv.h"

/* /////////////////////////////////////////////////////////--\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */


						/* < function definition in detail  > */


/*******************************************************
 * @fn								- GPIO_Clock_CONTROL
 *
 * @brief							- clock enable or disable function
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note
 */

//void GPIO_Clock_CONTROL(GPIO_REG_DEF *GPIOx , uint8_t EnDi)
//{
//	if(EnDi==1)
//			GPIO_Clock_CONTROL_EN(GPIOx);
//		else
//			GPIO_Clock_CONTROL_DI(GPIOx);
//}


/*******************************************************
 * @fn								- GPIO_Init
 *
 * @brief							- configure pin of micro in interrupt/event/input/output/alternate func/analog mode
 * 										this func config pin in this way
 * 										1-1: configuring mode of pin not interrupt mode
 * 										1-2: else interrupt/event mode configuration <<<<<peripheral side>>>>>
 * 										2:	configuring speed of pin
 * 										3:  configuring alternate function of pin
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note							- 1- we first configure pins for non intrrupt mode because ...
 */

void GPIO_Init(GPIO_REG_DEF *GPIOx , GPIO_INIT_STR *pdata )
{
	uint8_t temp1,temp2;
	temp1=((pdata->Pin)/8);
	temp2=((pdata->Pin)%8);
	GPIOx->CR[temp1] &= ~(0xF << (4*temp2));

if(pdata->Mode < GPIO_MODE_IT_FT)
{
	/* < pin - mode - pupd res - speed is configured in this part  > */
	/* < 1-first part configuration of gpio mode - CRL or CRH reg - CNF[1,0] part  > */

	if(pdata->Mode == GPIO_MODE_INPUT_ANALOG || pdata->Mode == GPIO_MODE_OUTPUT_PP)
		GPIOx->CR[temp1] &= ~(0xF << (4*temp2+2));

	else if(pdata->Mode == GPIO_MODE_OUTPUT_OD || pdata->Mode == GPIO_MODE_INPUT_FLT)
		GPIOx->CR[temp1] |= (0x1 << (4*temp2+2));

	else if(pdata->Mode == GPIO_MODE_INPUT_PU || pdata->Mode == GPIO_MODE_ALT_FUNC_PP \
			|| pdata->Mode == GPIO_MODE_INPUT_PD)
	{

		GPIOx->CR[temp1] |= (0x2 << (4*temp2+2));

		if(pdata->Mode == GPIO_MODE_INPUT_PU)
			GPIOx->ODR |= (1 << pdata->Pin);
		else if(pdata->Mode == GPIO_MODE_INPUT_PD)
			GPIOx->ODR &= ~(1 << pdata->Pin);
	}

	else
		GPIOx->CR[temp1] |= (0x3 << (4*temp2+2));

	/* < 2-second part configuration of gpio mode - CRL or CRH reg - MODE[1,0] part  > */

//	if(pdata->Speed == GPIO_SPEED_2M)
//		GPIOx->CR[temp1] |= (0x2 << 4*temp2);
//
//	else if(pdata->Speed == GPIO_SPEED_10M)
//		GPIOx->CR[temp1] |= (0x1 << 4*temp2);
//
//	else
//		GPIOx->CR[temp1] |= (0x3 << 4*temp2);

	GPIOx->CR[temp1] |= (pdata->Speed << 4*temp2);

}

/* < interrupt/event mode configuration , peripheral side - EXTI and AFIO configuration > */
else
{
	// 1. configuration of kind of trigger for event
	if(pdata->Mode >= GPIO_MODE_EVT_FT)
	{
		if(pdata->Mode == GPIO_MODE_EVT_RT)
			{
				EXTI->EXTI_RTSR |= (1 << (pdata->Pin));
				EXTI->EXTI_FTSR &= ~(1 << (pdata->Pin));
			}

			else if(pdata->Mode == GPIO_MODE_EVT_FT)
			{
				EXTI->EXTI_FTSR |= (1 << (pdata->Pin));
				EXTI->EXTI_RTSR &= ~(1 << (pdata->Pin));
			}
			else
			{
				EXTI->EXTI_FTSR |= (1 << (pdata->Pin));
				EXTI->EXTI_RTSR |= (1 << (pdata->Pin));
			}
			// 2. configuring the AFIO registers for choosing pin - witch pin?

			//AFIO_CLK_EN();
			RCC->APB2ENR.afio_en=1;

			AFIO->EXTICR[(pdata->Pin)/4] &= ~(0xF <<((pdata->Pin) % 4)*4);
			AFIO->EXTICR[(pdata->Pin)/4] |= (GPIO_PORT_CODE(GPIOx) << ((pdata->Pin) % 4)*4 );

			// 3. Enabling EXTI engine for event mode
			EXTI->EXTI_EMR |=(1<<(pdata->Pin));

	}

	else
	{
		    // 1. configuration of kind of trigger for interrupt
			if(pdata->Mode == GPIO_MODE_IT_RT)
			{
				EXTI->EXTI_RTSR |= (1 << (pdata->Pin));
				EXTI->EXTI_FTSR &= ~(1 << (pdata->Pin));
			}

			else if(pdata->Mode == GPIO_MODE_IT_FT)
			{
				EXTI->EXTI_FTSR |= (1 << (pdata->Pin));
				EXTI->EXTI_RTSR &= ~(1 << (pdata->Pin));
			}
			else
			{
				EXTI->EXTI_FTSR |= (1 << (pdata->Pin));
				EXTI->EXTI_RTSR |= (1 << (pdata->Pin));
			}

			// 2. configuring the AFIO registers for choosing pin - witch pin?
			//temp1 , temp2

			RCC->APB2ENR.afio_en=1;
			//AFIO_CLK_EN();

			AFIO->EXTICR[(pdata->Pin)/4] &= ~(0xF <<((pdata->Pin) % 4)*4);
			AFIO->EXTICR[(pdata->Pin)/4] |= (GPIO_PORT_CODE(GPIOx) << ((pdata->Pin) % 4)*4 );

			// 3. Enabling EXTI engine for interrupt mode
			EXTI->EXTI_IMR |=(1<<(pdata->Pin));


	}

}


}



/*******************************************************
 * @fn								- GPIO_Deinit
 *
 * @brief							-
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note
 */

void GPIO_Deinit(GPIO_REG_DEF *GPIOx)
{

	GPIO_REG_RESET(GPIOx);
}


/*******************************************************
 * @fn								- GPIO_Write_PIN
 *
 * @brief							-
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note
 */

void GPIO_Write_PIN(GPIO_REG_DEF *GPIOx	 ,  uint8_t PinNum , uint8_t PinVal)
{
	if(PinVal==ON)
		GPIOx->ODR |= (1 << PinNum);
	else
		GPIOx->ODR &= ~(1 << PinNum);

//	GPIOx->ODR &= ~(1 << PinNum);
//	GPIOx->ODR |= (PinVal << PinNum);
}


/*******************************************************
 * @fn								- GPIO_Write_PORT
 *
 * @brief							-
 *
 * @param[in]						-
 *
 *
 * @return			 				-None
 *
 * @Note
 */

void GPIO_Write_PORT(GPIO_REG_DEF *GPIOx ,  uint16_t PinVal)
{
	GPIOx->ODR = PinVal;
	//pdata->ODR &= 0xFFFF;
	//pdata->ODR |= PinVal;
}


/*******************************************************
 * @fn								- GPIO_Read
 *
 * @brief							-
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note							if we want to return a value of specific pin we should use
 * 									(bool )((pdata->IDR >> pinnumber)&(0x00000001));
 */

uint16_t GPIO_Read(GPIO_REG_DEF *GPIOx , uint16_t PinAdd)
{

	return (uint16_t)(GPIOx->IDR & PinAdd);

	//return ((uint16_t)(GPIOx->IDR >> PinAdd) & 0x0000000000000001)
}


uint16_t GPIO_Read_PIN(GPIO_REG_DEF *GPIOx , uint8_t PinNum)
{
	return ((uint8_t)(GPIOx->IDR >> PinNum) & 0x00000001);
}


/*******************************************************
 * @fn								- GPIO_Toggle_PIN
 *
 * @brief					 		-
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note
 */

void GPIO_Toggle_PIN(GPIO_REG_DEF *GPIOx , uint16_t PinNum)
{

	 GPIOx->ODR ^= (1<<PinNum);

//	 if(GPIOx->IDR & PinNum >=1)
//		 GPIOx->ODR &= ~(1<<PinNum);
//	 else
//		 GPIOx->ODR |= (1<<PinNum);



}


/*******************************************************
 * @fn								- GPIO_IRQConf
 *
 * @brief							-
 *
 * @param[in]						uint8_t IRQNum , uint8_t IRQPrio , uint8_t EnDi
 *
 *
 * @return							-None
 *
 * @Note							this function is for initializing interrupts in core side
 */

void GPIO_IRQConf(uint8_t IRQNum , uint8_t IRQPrio , uint8_t EnDi)
{

}


/*******************************************************
 * @fn								- GPIO_IRQHandler
 *
 * @brief							-
 *
 * @param[in]						-
 *
 *
 * @return							-None
 *
 * @Note
 */

void GPIO_IRQHandler(uint8_t PinNum)
{}


/*******************************************************
 * @fn								- SOFT_IRQ_Request
 *
 * @brief							-If interrupt are enabled on line x in the EXTI_IMR register, calling this function, thus resulting in an
									interrupt request generation.
 *
 * @param[in]						-PinNum : the number of pin // pin_0 ,pin_ 1 , .....
 *
 *
 * @return							-None
 *
 * @Note							-this generate interrupt request on which EXTI line that configures in GPIO_Init func
 */

void SOFT_IRQ_Request(uint8_t PinNum)
{}


/*******************************************************
 * @fn								- SOFT_IRQ_Request
 *
 * @brief							-If interrupt are enabled on line x in the EXTI_IMR register, calling this function, thus resulting in an
									interrupt request generation.
 *
 * @param[in]						-PinNum : the number of pin // pin_0 ,pin_ 1 , .....
 *
 *
 * @return							-None
 *
 * @Note							-this generate interrupt request on which EXTI line that configures in GPIO_Init func
 */

uint8_t Event_Pending(uint8_t PinNum)
{


return 0;
}









