/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include <stdint.h>
//#include "A_gpio_f10xx_drv.h"
//
//void delay_Ms(unsigned int nCount)
//{
//    unsigned int i, j;
//
//    for (i = 0; i < nCount; i++)
//        for (j = 0; j < 0x850 ; j++);
//}

//GPIO_INIT_STR pin_config;
uint8_t *ali=(uint8_t *)0x2222;
//int data=0,ali=0x2222;
uint8_t *data;

int main(void)

{

//	pin_config.Mode=GPIO_MODE_OUTPUT_PP;
//	pin_config.Pin=pin_12;
//	pin_config.Speed=GPIO_SPEED_2M;
//
//	GPIO_Clock_CONTROL(GPIOB , ON);
//	GPIO_Init(GPIOB , &pin_config);

	data= ali+0x0001;
	while(1)
	{
//	delay_Ms(1000);
//	GPIO_Toggle_PIN(GPIOB, pin_12);
	}


}


