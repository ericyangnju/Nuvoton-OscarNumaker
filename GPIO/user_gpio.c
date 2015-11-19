#include "isd9100.h"
#include "user_gpio.h"


const platform_gpio_t platform_gpio_pins[] =
{
	[BLE_WIFI_RESET]  		= { PA, BIT12, 12	},
	[MPU6050_INT]			= { PA, BIT14, 14	},
	[LED_MAINB_R]			= { PB, BIT5,   5 },
	[LED_MAINB_G]			= { PB, BIT6,   6	},
	[LED_MAINB_B]			= { PB, BIT7,   7	},
	[LED_SENB_R]			= { PA, BIT11, 11	},
	[LED_SENB_G]			= { PA, BIT13, 13	},
	[LED_SENB_B]			= { PA, BIT3,   3	},
	[KEY_1]					= { PA, BIT5,   5	},
	[KEY_2]					= { PA, BIT6,   6	},
	[KEY_3]					= { PA, BIT7,   7	},
	[PWM0_CH0_PIN]			= { PA, BIT12, 12	},
	[PWM0_CH1_PIN]			= { PA, BIT13, 13	},
	[GAS_EN]				= { PA, BIT15, 15 },
};

/* PWM mappings */
const platform_pwm_t platform_pwm_peripherals[] =
{  
	[NUVOTON_PWM0_CH0]  = 
	{     
		.out_channel   	= PWM_CH0,
		.gp_mfp_msk		= SYS_GPA_MFP_PA12MFP_Msk,
		.gp_mfp			= SYS_GPA_MFP_PA12MFP_PWM0CH0,
	},

	[NUVOTON_PWM0_CH1]  = 
	{     
		.out_channel    	= PWM_CH1,
		.gp_mfp_msk		= SYS_GPA_MFP_PA13MFP_Msk,
		.gp_mfp			= SYS_GPA_MFP_PA13MFP_PWM0CH1,
	},
};


/**
 * @brief	Set an individual GPIO output pin to the high state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: port Number (supports port 0 only)
 * @param	pin		: pin number (0..n) to set high
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output.
 */
void GPIO_SetPinOutHigh(GPIO_T *gpio, uint32_t u32PinMask)
{
	uint32_t i;
	for (i=0; i<GPIO_PIN_MAX; i++) 
	{
		if (u32PinMask & (1 << i)) 
		{
	    		gpio->DOUT = gpio->DOUT | (1 << i);
		}
	}
}

/**
 * @brief	Set an individual GPIO output pin to the low state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: port Number (supports port 0 only)
 * @param	pin		: pin number (0..n) to set low
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output.
 */
void GPIO_SetPinOutLow(GPIO_T *gpio, uint32_t u32PinMask)
{
	uint32_t i;
	for (i=0; i<GPIO_PIN_MAX; i++) 
	{
		if (u32PinMask & (1 << i)) 
		{
	    		gpio->DOUT = gpio->DOUT & ~(1 << i);
		}
	}
}

void Init_GPIO(void)
{
	/* BLE or WiFi Reset */
	GPIO_SetMode(platform_gpio_pins[BLE_WIFI_RESET].port, platform_gpio_pins[BLE_WIFI_RESET].pin_number,  GPIO_MODE_OUTPUT);
	GPIO_SetPinOutHigh(platform_gpio_pins[BLE_WIFI_RESET].port, platform_gpio_pins[BLE_WIFI_RESET].pin_number);

	/* MPU6050 Interrupt pin */  
	GPIO_SetMode(platform_gpio_pins[MPU6050_INT].port, platform_gpio_pins[MPU6050_INT].pin_number, GPIO_MODE_QUASI);

	/* GAS Enable */
	GPIO_SetMode(platform_gpio_pins[GAS_EN].port, platform_gpio_pins[GAS_EN].pin_number,  GPIO_MODE_OUTPUT);
	GPIO_SetPinOutHigh(platform_gpio_pins[GAS_EN].port, platform_gpio_pins[GAS_EN].pin_number);

	/* Main Board Red Green Blue LED */
	GPIO_SetMode(platform_gpio_pins[LED_MAINB_R].port, platform_gpio_pins[LED_MAINB_R].pin_number,  GPIO_MODE_OUTPUT);
	GPIO_SetPinOutHigh(platform_gpio_pins[LED_MAINB_R].port, platform_gpio_pins[LED_MAINB_R].pin_number);

	GPIO_SetMode(platform_gpio_pins[LED_MAINB_G].port, platform_gpio_pins[LED_MAINB_G].pin_number,  GPIO_MODE_OUTPUT);
	GPIO_SetPinOutHigh(platform_gpio_pins[LED_MAINB_G].port, platform_gpio_pins[LED_MAINB_G].pin_number);

	GPIO_SetMode(platform_gpio_pins[LED_MAINB_B].port, platform_gpio_pins[LED_MAINB_B].pin_number,  GPIO_MODE_OUTPUT);
	GPIO_SetPinOutHigh(platform_gpio_pins[LED_MAINB_B].port, platform_gpio_pins[LED_MAINB_B].pin_number);

	/* Sensor Board Red Green Blue LED */
	GPIO_SetMode(platform_gpio_pins[LED_SENB_R].port, platform_gpio_pins[LED_SENB_R].pin_number,  GPIO_MODE_OUTPUT);
	GPIO_SetPinOutHigh(platform_gpio_pins[LED_SENB_R].port, platform_gpio_pins[LED_SENB_R].pin_number);

	GPIO_SetMode(platform_gpio_pins[LED_SENB_G].port, platform_gpio_pins[LED_SENB_G].pin_number,  GPIO_MODE_OUTPUT);
	GPIO_SetPinOutHigh(platform_gpio_pins[LED_SENB_G].port, platform_gpio_pins[LED_SENB_G].pin_number);

	GPIO_SetMode(platform_gpio_pins[LED_SENB_B].port, platform_gpio_pins[LED_SENB_B].pin_number,  GPIO_MODE_OUTPUT);
	GPIO_SetPinOutHigh(platform_gpio_pins[LED_SENB_B].port, platform_gpio_pins[LED_SENB_B].pin_number);			
}

