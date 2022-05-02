/*
 * 2_read_pin.c
 *
 *  Created on: Feb 7, 2021
 *      Author: Ali Yazdanifar
 */

#include "A_gpio_f10xx_drv.h"

void delay_Ms(unsigned int nCount)
{
    unsigned int i, j;

    for (i = 0; i < nCount; i++)
        for (j = 0; j < 0x850 ; j++);
}

GPIO_INIT_STR pin_config;
uint8_t data;

int main(void)
{

	pin_config.Mode=GPIO_MODE_INPUT_PU;
	pin_config.Pin=pin_0;
	pin_config.Speed=None;

	GPIO_Clock_CONTROL(GPIOA , ON);
	GPIO_Init(GPIOA, &pin_config);

	while(1)
	{
		data=GPIO_Read_PIN(GPIOA,pin_0);
		delay_Ms(50);
	}





}


