/*---------------------------------------------------------------------------------------------------------*/
/*																										   */
/* Copyright (c) Nuvoton Technology	Corp. All rights reserved.											   */
/*																										   */
/*---------------------------------------------------------------------------------------------------------*/

// ---------------------------------------------------------------------------------------------------------
//	Functions:
//		- TimeF ISR: handle de-bounce time for direct trigger key and matrix key.
//		- Time2 ISR: handle UltraIO curve output with F/W PWM mode.
//		- GPIOAB ISR: handle wake up by GPIOA or GPIO B.
//		- Other interrupt ISRs.
//
//	Reference "Readme.txt" for more information.
// ---------------------------------------------------------------------------------------------------------
//#include "Framework.h"
//#include "Keypad.h"
#include "./GPIO/user_gpio.h"
extern const platform_gpio_t platform_gpio_pins[];
extern const platform_pwm_t platform_pwm_peripherals[] ;
extern volatile uint8_t KEY1_Flag;
extern volatile uint8_t KEY2_Flag;
extern volatile uint8_t KEY3_Flag;


void EINT0_IRQHandler(void)
{
	PB->INTSRC |= BIT0;
	//printf("EINT0 Press \r\n");	
}
void EINT1_IRQHandler(void)
{
	PB->INTSRC |= BIT1;
	//printf("EINT1 Press \r\n");		
}

void GPAB_IRQHandler(void)
{
//	static uint32_t Count=0;
	if (platform_gpio_pins[KEY_1].port->INTSRC & platform_gpio_pins[KEY_1].pin_number) 
	{
		platform_gpio_pins[KEY_1].port->INTSRC |= platform_gpio_pins[KEY_1].pin_number; 
		KEY1_Flag = 1;
	} 
	else if (platform_gpio_pins[KEY_2].port->INTSRC & platform_gpio_pins[KEY_2].pin_number) 
	{
		platform_gpio_pins[KEY_2].port->INTSRC |= platform_gpio_pins[KEY_2].pin_number;         
		KEY2_Flag = 1;
	} 
	else if (platform_gpio_pins[KEY_3].port->INTSRC & platform_gpio_pins[KEY_3].pin_number) 
	{
		platform_gpio_pins[KEY_3].port->INTSRC |= platform_gpio_pins[KEY_3].pin_number; 
		KEY3_Flag = 1;
	} 
	else 
	{                     
		PB->INTSRC = 0xFFFF;	      // clear all GPC pins
		PA->INTSRC = 0xFFFF;
	}
//	Count++;
}


