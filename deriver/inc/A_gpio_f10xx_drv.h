/*
 * A_gpio_f10xx_drv.h
 *
 *  Created on: Feb 2, 2021
 *      Author: Ali Yazdanifar
 *
 *      note:   1- in the header file we should include all .h files that are needed
 *      		2- in the header file we should implement all prototype of function(API)
 *      		3- the count of shirt is equal to 2^n
 */

#ifndef INC_A_GPIO_F10XX_DRV_H_
#define INC_A_GPIO_F10XX_DRV_H_


#include "A_stm32f10xx.h"

/* /////////////////////////////////////////////////////////--\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */

/* < structure for user application > */
typedef struct
{
		  uint8_t Pin;      	/*!< Specifies the GPIO pins to be configured.*/
		  int8_t Mode;    		/*!< Specifies the operating mode for the selected pins.*/
		  uint8_t Speed;     	/*!< Specifies the speed for the selected pins.*/
		  	  	  	  	  	  	/*!< Peripheral to be connected to the selected pins.*/

}GPIO_INIT_STR;

/* /////////////////////////////////////////////////////////--\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */

/* < macros for pin for initiating relevant register   > */
#define pin_0							0
#define pin_1							1
#define pin_2							2
#define pin_3							3
#define pin_4							4
#define pin_5							5
#define pin_6							6
#define pin_7							7
#define pin_8							8
#define pin_9							9
#define pin_10							10
#define pin_11							11
#define pin_12							12
#define pin_13							13
#define pin_14							14
#define pin_15							15

/* < macro for mode base on MODEx register available state  > */
#define GPIO_MODE_INPUT_ANALOG				0
#define GPIO_MODE_OUTPUT_PP				1
#define GPIO_MODE_OUTPUT_OD				2
#define GPIO_MODE_ALT_FUNC_PP				3
#define GPIO_MODE_ALT_FUNC_OP				4
#define GPIO_MODE_INPUT_FLT				5
#define GPIO_MODE_INPUT_PD				6
#define GPIO_MODE_INPUT_PU				7

#define GPIO_MODE_IT_FT					8
#define GPIO_MODE_IT_RT					9
#define GPIO_MODE_IT_FRT				10

#define GPIO_MODE_EVT_FT				11
#define GPIO_MODE_EVT_RT				12
#define GPIO_MODE_EVT_FRT				13

/* < macro for speed > */
#define None						0
#define GPIO_SPEED_2M					2
#define GPIO_SPEED_10M					1
#define GPIO_SPEED_50M 					3


///* < macro for pulldown_pullup - we dont need  > */
//#define GPIO_PIN_NO_PD_PU				0
//#define GPIO_PIN_PULL_UP				1
//#define GPIO_PIN_PULL_DOWN				2

/* /////////////////////////////////////////////////////////--\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */


#define EXTI0_IRQn                  6      /*	< EXTI Line0 Interrupt  		     >  */
#define EXTI1_IRQn                  7      /*	< EXTI Line1 Interrupt   			 >  */
#define EXTI2_IRQn                  8      /*	< EXTI Line2 Interrupt   			 >	 */
#define EXTI3_IRQn                  9      /*	< EXTI Line3 Interrupt    		 	 >   */
#define EXTI4_IRQn                  10     /*	< EXTI Line4 Interrupt 			     > 	 */
#define EXTI9_5_IRQn                23     /*	< External Line[9:5] Interrupts      > 	 */
#define EXTI15_10_IRQn              40     /*	< External Line[15:10] Interrupts    >   */


#define IRQ_Priority_0				0
#define IRQ_Priority_1				1
#define IRQ_Priority_2				2
#define IRQ_Priority_3				3
#define IRQ_Priority_4				4
#define IRQ_Priority_5				5
#define IRQ_Priority_6				6
#define IRQ_Priority_7				7
#define IRQ_Priority_8				8
#define IRQ_Priority_9				9
#define IRQ_Priority_10				10
#define IRQ_Priority_11				11
#define IRQ_Priority_12				12
#define IRQ_Priority_13				13
#define IRQ_Priority_14				14
#define IRQ_Priority_15				15

/* /////////////////////////////////////////////////////////--\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */

#define ON 								1
#define OFF								0

/* /////////////////////////////////////////////////////////--\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */

										/* < applicable macro for gpio deriver  > */


/* < do ... while condition zero loop > */
#define GPIO_REG_RESET(GPIOx)			do{(RCC->APB2RSTR |= (1<<((uint32_t)GPIOx-APB2_BASE_ADD)/0x400));\
											(RCC->APB2RSTR &= ~(1<<((uint32_t)GPIOx-APB2_BASE_ADD)/0x400));}while(0)

/* < C conditional operator or ternary operation > */

#define GPIO_PORT_CODE(x)		((x==GPIOA)?0:(x==GPIOB)?1:(x==GPIOC)?2:(x==GPIOD)?3:(x==GPIOE)?4:(x==GPIOF)?5:(x==GPIOG)?6:0)





/* /////////////////////////////////////////////////////////--\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */

/* < APIs - functions prototype > */

void GPIO_Clock_CONTROL(GPIO_REG_DEF *GPIOx , uint8_t EnDi);

void GPIO_Init(GPIO_REG_DEF *GPIOx , GPIO_INIT_STR *pdata );

void GPIO_Deinit(GPIO_REG_DEF *GPIOx);

void GPIO_Write_PIN(GPIO_REG_DEF *GPIOx	 ,  uint8_t PinNum , uint8_t PinVal);

void GPIO_Write_PORT(GPIO_REG_DEF *GPIOx ,  uint16_t PinVal);

uint16_t GPIO_Read(GPIO_REG_DEF *GPIOx , uint16_t PinAdd);

uint16_t GPIO_Read_PIN(GPIO_REG_DEF *GPIOx , uint8_t PinNum);

void GPIO_Toggle_PIN(GPIO_REG_DEF *GPIOx , uint16_t PinNum);

void GPIO_IRQConf(uint8_t IRQNum , uint8_t IRQPrio , uint8_t EnDi);

void GPIO_IRQHandler(uint8_t PinNum);

void SOFT_IRQ_Request(uint8_t PinNum);

uint8_t Event_Pending(uint8_t PinNum);


#endif /* INC_A_GPIO_F10XX_DRV_H_ */
