#ifndef USER_GPIO_H
#define USER_GPIO_H

#include "isd9100.h"

typedef enum
{
	BLE_WIFI_RESET,
	MPU6050_INT,
	LED_MAINB_R,
	LED_MAINB_G,
	LED_MAINB_B,
	LED_SENB_R,
	LED_SENB_G,
	LED_SENB_B,
	KEY_1,
	KEY_2,
	KEY_3,
	PWM0_CH0_PIN,
	PWM0_CH1_PIN,
	GAS_EN,
	NUVOTON_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
	NUVOTON_GPIO_NONE,
} nuvoton_gpio_t;

typedef enum
{
	NUVOTON_PWM0_CH0,	
	NUVOTON_PWM0_CH1,
	NUVOTON_PWM_MAX, /* Denotes the total number of PWM cahnnel aliases. Not a valid PWM alias */
	NUVOTON_PWM_NONE,
} nuvoton_pwm_t;

typedef struct
{
	GPIO_T* 			port;    
	uint32_t      pin_number;
	uint8_t				bit;
} platform_gpio_t;

typedef struct
{
	uint8_t                	out_channel;
	uint32_t								gp_mfp_msk;
	uint32_t								gp_mfp;
} platform_pwm_t;

void GPIO_SetPinOutHigh(GPIO_T *gpio, uint32_t u32PinMask);
void GPIO_SetPinOutLow(GPIO_T *gpio, uint32_t u32PinMask);
void Init_GPIO(void);


#endif
