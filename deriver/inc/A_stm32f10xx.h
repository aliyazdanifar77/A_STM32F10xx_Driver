/*
 * A_stm32f10xx.h
 *
 *  Created on: Jan 31, 2021
 *      Author: Ali Yazdanifar
 *
 *      note :  1- use capital letter for C MACROs
 *      		2- use suffix for deriver layer for example DRV_FLASH_BASE_ADD ...
 *      		because in big project we have many layer
 *
 *
 */

#ifndef INC_A_STM32F10XX_H_
#define INC_A_STM32F10XX_H_

/* < necessary files and define > */

#include "stdint.h"



/* /////////////////////////////////////////////////////////--\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */

/* < memory and ram domain > */

#define FLASH_BASE_ADD			(uint32_t) 0x08000000   // (type costed value) main memory = flash memory - 128Kbyte
#define SRAM_BASE_ADD			0x20000000U			    // 20Kbyte
#define ROM_BASE_ADD			0x1FFFF000U				/* < System memory is ROM > */
#define SRAM					SRAM_BASE_ADD			/* < COMMENT > */

/* < APB1/APB2/AHB bus domain > */

#define PRIFH_BASE_ADD			0x40000000U
#define APB1_BASE_ADD			PRIFH_BASE_ADD
#define APB2_BASE_ADD			0x40010000U
#define AHB_BASE_ADD			0x40018000U

/* < APB2 peripheral domain > */

#define gpio_offset				0x0400

#define AFIO_BASE_ADD			APB2_BASE_ADD

#define EXTI_BASE_ADD			(APB2_BASE_ADD + 0x0400)

#define GPIOA_BASE_ADD			(APB2_BASE_ADD + 0x0800)		/* < #define GPIOA_BASE_ADD			0x40010800 > */
#define GPIOB_BASE_ADD			(APB2_BASE_ADD + 0x0C00)		/* < #define GPIOB_BASE_ADD			APB2_BASE_ADD + (gpio_offset * 3)	> */
#define GPIOC_BASE_ADD			(APB2_BASE_ADD + 0x1000)		/* < #define GPIOC_BASE_ADD			APB2_BASE_ADD + (gpio_offset * 4) > */
#define GPIOD_BASE_ADD			(APB2_BASE_ADD + 0x1400)		/* < COMMENT > */
#define GPIOE_BASE_ADD			(APB2_BASE_ADD + 0x1800)		/* < COMMENT > */
#define GPIOF_BASE_ADD			(APB2_BASE_ADD + 0x1C00)		/* < COMMENT > */
#define GPIOG_BASE_ADD			(APB2_BASE_ADD + 0x2000)		/* < COMMENT > */

#define SPI1_BASE_ADD			(APB2_BASE_ADD + 0x3000)

#define EXTI_BASE_ADD			(APB2_BASE_ADD + 0x0400)

#define USART1_BASE_ADD			(APB2_BASE_ADD + 0x3800)


/* < APB1 peripheral domain > */

#define SPI2_BASE_ADD			(APB1_BASE_ADD + 0x3800)
#define SPI3_BASE_ADD			(APB1_BASE_ADD + 0x3C00)

#define USART2_BASE_ADD			(APB1_BASE_ADD + 0x4400)
#define USART3_BASE_ADD			(APB1_BASE_ADD + 0x4800)
#define UART4_BASE_ADD			(APB1_BASE_ADD + 0x4C00)
#define UART5_BASE_ADD			(APB1_BASE_ADD + 0x5000)

#define I2C1_BASE_ADD			(APB1_BASE_ADD + 0x5400)
#define I2C2_BASE_ADD			(APB1_BASE_ADD + 0x5800)


/* < AHP peripheral domain > */

#define RCC_BASE_ADD			(AHB_BASE_ADD + 0x9000)



/* /////////////////////////////////////////////////////////--\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */



			/* < Strutting peripheral register with structure and pointer
			 * note : 	1- shorthand notation is available in stdint.h
			 * 			2- Volatile is not necessary for all register
			 * 			3- usual offset is 0x04 = 0b0100 usual register are 32bit means
			 * 			we must have 2*16 bit add and another register should start from
			 * 			0x04 + present_register_add
			 * 			4- this deriver isn't write for all F1 value line we use RM0008
			 * 			and for value line we should use another ref mul
			 *
			 * > */

//	#define STM32F10X_LD      /* < STM32F10X_LD: STM32 Low density devices > */
//	#define STM32F10X_LD_VL   /* < STM32F10X_LD_VL: STM32 Low density Value Line devices > */
   	#define STM32F10X_MD      /* < STM32F10X_MD: STM32 Medium density devices > */
//   	#define STM32F10X_MD_VL   /* < STM32F10X_MD_VL: STM32 Medium density Value Line devices > */
//   	#define STM32F10X_HD      /* < STM32F10X_HD: STM32 High density devices > */
//   	#define STM32F10X_HD_VL   /* < STM32F10X_HD_VL: STM32 High density value line devices > */
//   	#define STM32F10X_XL      /* < STM32F10X_XL: STM32 XL-density devices > */
//   	#define STM32F10X_CL      /* < STM32F10X_CL: STM32 Connectivity line devices > */



#define __IO volatile		/* < is available in core_cm3.h file  > */

typedef uint32_t  u32;
typedef __IO uint32_t  vu32;


/* <  > */

typedef struct
{
	vu32 CR[2];
	vu32 IDR;
	vu32 ODR;
	vu32 BSRR;
	vu32 BRR;
	vu32 LCKR;

} GPIO_REG_DEF;



/* <  > */

typedef struct
{
	vu32 EVCR;
	vu32 MAPR;
	vu32 EXTICR[4];
    uint32_t RESERVED0;
  	vu32 MAPR2;

} AFIO_REG_DEF;



/* <  > */

typedef struct
{
  vu32 CR;
  vu32 CFGR;
  vu32 CIR;
  vu32 APB2RSTR;
  vu32 APB1RSTR;
  vu32 AHBENR;
  vu32 APB2ENR;
  vu32 APB1ENR;
  vu32 BDCR;
  vu32 CSR;

#ifdef STM32F10X_CL
  vu32 AHBRSTR;
  vu32 CFGR2;
#endif /* STM32F10X_CL */

//
//#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
//  uint32_t RESERVED0;
//  vu32 CFGR2;
//#endif /* STM32F10X_LD_VL || STM32F10X_MD_VL || STM32F10X_HD_VL */

} RCC_REG_DEF;

/* <  > */

typedef struct
{
	vu32 EXTI_IMR;
	vu32 EXTI_EMR;
	vu32 EXTI_RTSR;
	vu32 EXTI_FTSR;
	vu32 EXTI_SWIER;
	vu32 EXTI_PR;

}EXTI_REG_DEF;

/* /////////////////////////////////////////////////////////--\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */


#define GPIOA		((GPIO_REG_DEF *)GPIOA_BASE_ADD)
#define GPIOB		((GPIO_REG_DEF *)GPIOB_BASE_ADD)
#define GPIOC		((GPIO_REG_DEF *)GPIOC_BASE_ADD)
#define GPIOD		((GPIO_REG_DEF *)GPIOD_BASE_ADD)
#define GPIOE		((GPIO_REG_DEF *)GPIOE_BASE_ADD)
#define GPIOF		((GPIO_REG_DEF *)GPIOF_BASE_ADD)
#define GPIOG		((GPIO_REG_DEF *)GPIOG_BASE_ADD)

#define AFIO		((AFIO_REG_DEF *)AFIO_BASE_ADD)

#define EXTI		((EXTI_REG_DEF *)EXTI_BASE_ADD)

#define RCC			((RCC_REG_DEF *)RCC_BASE_ADD)


/* /////////////////////////////////////////////////////////--\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ */

//for gpioa (0x40010800U - 0x40010000U)/0x400 = 0x800/0x400=2
//for gpiob ((0x40010C00U - 0x40010000U)/0x400 = 0xC00/0x400=3)

#define AFIO_CLK_EN()		(RCC->APB2ENR |= (1<<0))
#define AFIO_CLK_DI()		(RCC->APB2ENR &= ~(1<<0))

#define GPIO_Clock_CONTROL_EN(pdata)		(RCC->APB2ENR |= (1<<(((uint32_t )pdata - APB2_BASE_ADD)/0x400)))
#define GPIO_Clock_CONTROL_DI(pdata)		(RCC->APB2ENR &= ~(1<<(((uint32_t )pdata - APB2_BASE_ADD)/0x400)))

#define GPIOA_CLK_EN()		(RCC->APB2ENR |= (1<<2))
#define GPIOB_CLK_EN()		(RCC->APB2ENR |= (1<<3))
#define GPIOC_CLK_EN()		(RCC->APB2ENR |= (1<<4))
#define GPIOD_CLK_EN()		(RCC->APB2ENR |= (1<<5))
#define GPIOE_CLK_EN()		(RCC->APB2ENR |= (1<<6))
#define GPIOF_CLK_EN()		(RCC->APB2ENR |= (1<<7))
#define GPIOG_CLK_EN()		(RCC->APB2ENR |= (1<<8))


#define GPIOA_CLK_DI()		(RCC->APB2ENR &= ~(1<<2))
#define GPIOB_CLK_DI()		(RCC->APB2ENR &= ~(1<<3))
#define GPIOC_CLK_DI()		(RCC->APB2ENR &= ~(1<<4))
#define GPIOD_CLK_DI()		(RCC->APB2ENR &= ~(1<<5))
#define GPIOE_CLK_DI()		(RCC->APB2ENR &= ~(1<<6))
#define GPIOF_CLK_DI()		(RCC->APB2ENR &= ~(1<<7))
#define GPIOG_CLK_DI()		(RCC->APB2ENR &= ~(1<<8))

#define USART1_CLK_EN()		(RCC->APB2ENR |= (1<<14))
#define USART2_CLK_EN()		(RCC->APB1ENR |= (1<<17))
#define USART3_CLK_EN()		(RCC->APB1ENR |= (1<<18))
#define UART4_CLK_EN()		(RCC->APB1ENR |= (1<<19))
#define UART5_CLK_EN()		(RCC->APB1ENR |= (1<<20))

#define USART1_CLK_DI()		(RCC->APB2ENR &= ~(1<<14))
#define USART2_CLK_DI()		(RCC->APB1ENR &= ~(1<<17))
#define USART3_CLK_DI()		(RCC->APB1ENR &= ~(1<<18))
#define UART4_CLK_DI()		(RCC->APB1ENR &= ~(1<<19))
#define UART5_CLK_DI()		(RCC->APB1ENR &= ~(1<<20))

#define SPI1_CLK_EN()		(RCC->APB2ENR |= (1<<12))
#define SPI2_CLK_EN()		(RCC->APB1ENR |= (1<<14))
#define SPI3_CLK_EN()		(RCC->APB1ENR |= (1<<15))

#define SPI1_CLK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define SPI2_CLK_DI()		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_CLK_DI()		(RCC->APB1ENR &= ~(1<<15))

#define I2C_CLK_EN(pdata)		(RCC->APB1ENR |= (1<<((((u32 )pdata - APB1_BASE_ADD)/0x400) + 6)))

#define I2C_CLK_DI(pdata)		(RCC->APB1ENR &= ~(1<<((((u32 )pdata - APB1_BASE_ADD)/0x400) + 6)))









#endif /* INC_A_STM32F10XX_H_ */

