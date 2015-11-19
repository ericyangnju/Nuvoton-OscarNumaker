/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 14/07/10 10:14a $
 * @brief    This sample code demo semihost function
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "isd9100.h"

//#include "DHT11.h"
#include "MPU6050.h"
#include "AHRSLib.h"
#include "Sensors.h"
#include "Timer_Ctrl.h"
#define SYSTICK
#define I2CSpeed 400000
extern uint8_t I2CFail;

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

const platform_gpio_t platform_gpio_pins[] =
{
    [BLE_WIFI_RESET]  = { PA, BIT12, 12	},
		[MPU6050_INT]			= { PA, BIT14, 14	},
		[LED_MAINB_R]			= { PB, BIT5,   5 },
		[LED_MAINB_G]			= { PB, BIT6,   6	},
		[LED_MAINB_B]			= { PB, BIT7,   7	},
		[LED_SENB_R]			= { PA, BIT11, 11	},
		[LED_SENB_G]			= { PA, BIT13, 13	},
		[LED_SENB_B]			= { PA, BIT3,   3	},
		[KEY_1]						= { PA, BIT5,   5	},
		[KEY_2]						= { PA, BIT6,   6	},
		[KEY_3]						= { PA, BIT7,   7	},
		[PWM0_CH0_PIN]		= { PA, BIT12, 12	},
		[PWM0_CH1_PIN]		= { PA, BIT13, 13	},
};

/* PWM mappings */
const platform_pwm_t platform_pwm_peripherals[] =
{  
  [NUVOTON_PWM0_CH0]  = 
  {     
    .out_channel    = PWM_CH0,
		.gp_mfp_msk			= SYS_GPA_MFP_PA12MFP_Msk,
		.gp_mfp					=	SYS_GPA_MFP_PA12MFP_PWM0CH0,
  },
	
	[NUVOTON_PWM0_CH1]  = 
  {     
    .out_channel    = PWM_CH1,
		.gp_mfp_msk			= SYS_GPA_MFP_PA13MFP_Msk,
		.gp_mfp					=	SYS_GPA_MFP_PA13MFP_PWM0CH1,
  },
};
#define RXBUFSIZE 4
#define PGA_GAIN    -600  //default
#define ADC_SAMPLE_RATE  (16000)  //default

float GyroScale[3];
float AccScale[3];
float GyroOffset[3];
float AccOffset[3];


volatile int16_t rawGYRO[3];
volatile int16_t rawACC[3];
int16_t iEuler[3];

SensorInit_T SensorInitState = {true,true,false};
SensorInit_T SensorCalState  = {false,false,false};
Sensor_T Sensor;

volatile char     Text[49] = {'@','A','C','C',0,0,0,0,0,0, //offset 4
															'@','G','Y','R',0,0,0,0,0,0, //offset 14
															'@','T','E','M',0,0,  //offset 24
															'@','H','R','W',0,0, //offset 30
															'@','L','I','R',0,0, //offset 36
															'@','G','A','S',0,0, //offset 42
															'@','D','I','S',0,  //offset 48
															}	; 

volatile char    Text_ACK[4];
volatile uint8_t RX_buffer[RXBUFSIZE];
volatile uint8_t command[4];
volatile uint8_t Flag_report =0;
volatile uint8_t KEY1_Flag = 0;
volatile uint8_t KEY2_Flag = 0;
volatile uint8_t KEY3_Flag = 0, report_rate=1 ,Report_on =0;
volatile uint32_t g_u32TICK = 0,Timer_F;
volatile uint32_t LightValue, GasValue;	
volatile uint16_t AcceX, AcceY,AcceZ,GyroX,GyroY,GyroZ,Humidity,Temp;

volatile uint16_t time_DHT11[42];
volatile uint32_t capture_count =0;
volatile uint16_t DHT11_Data[2];

volatile uint8_t DIS_Value;
															
volatile uint8_t uartno;															

/**
 *    @brief    Set Rx FIFO interrupt Trigger Level
 *
 *    @param    uart        The base address of UART module
 *    @param    u32TriggerLevel   RX FIFO interrupt Trigger Level. ( \ref UART_FIFO_RFITL_1BYTE / \ref UART_FIFO_RFITL_4BYTE / 
 *                                                                   \ref UART_FIFO_RFITL_8BYTE / \ref UART_TLCTL_RFITL_14BYTES )
 *    @return    None
 */
#define UART_SET_RX_FIFO_INTTRGLV(uart, u32TriggerLevel)   (((uart)->FIFO  = ((uart)->FIFO  &  ~UART_FIFO_RFITL_Msk) | (u32TriggerLevel))

/**
 * @brief	Set an individual GPIO output pin to the high state
 * @param	pGPIO	: The base of GPIO peripheral on the chip
 * @param	port	: port Number (supports port 0 only)
 * @param	pin		: pin number (0..n) to set high
 * @return	None
 * @note	Any bit set as a '0' will not have it's state changed. This only
 * applies to ports configured as an output.
 */
void CheckI2C(void);
 
void GPIO_SetPinOutHigh(GPIO_T *gpio, uint32_t u32PinMask)
{
	  uint32_t i;
    for (i=0; i<GPIO_PIN_MAX; i++) {
        if (u32PinMask & (1 << i)) {
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
    for (i=0; i<GPIO_PIN_MAX; i++) {
        if (u32PinMask & (1 << i)) {
            gpio->DOUT = gpio->DOUT & ~(1 << i);
        }
    }
}

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
		static uint32_t Count=0;
    if (platform_gpio_pins[KEY_1].port->INTSRC & platform_gpio_pins[KEY_1].pin_number) {
        platform_gpio_pins[KEY_1].port->INTSRC |= platform_gpio_pins[KEY_1].pin_number; 
			  KEY1_Flag = 1;
				//printf("KEY1 Press %d\r\n",Count);
    } else if (platform_gpio_pins[KEY_2].port->INTSRC & platform_gpio_pins[KEY_2].pin_number) {
        platform_gpio_pins[KEY_2].port->INTSRC |= platform_gpio_pins[KEY_2].pin_number;         
        KEY2_Flag = 1;
				//printf("KEY2 Press %d\r\n",Count);
    } else if (platform_gpio_pins[KEY_3].port->INTSRC & platform_gpio_pins[KEY_3].pin_number) {
        platform_gpio_pins[KEY_3].port->INTSRC |= platform_gpio_pins[KEY_3].pin_number; 
        KEY3_Flag = 1;
				//printf("KEY3 Press %d\r\n",Count);
    } else {                     
        PB->INTSRC = 0xFFFF;	      // clear all GPC pins
				PA->INTSRC = 0xFFFF;
    }
		Count++;
}

void UART0_IRQHandler(void)
{
    uint32_t u32IntSts= UART0->INTSTS;
		uint8_t i;
    
    if(u32IntSts & UART_IS_RX_READY(UART0)) {			
      UART_Read(UART0, RX_buffer, RXBUFSIZE);
			for(i=0;i<4;i++)
				command[i]=RX_buffer[i];
	
			Flag_report=1;
			//print_Line(2,RX_buffer);
			uartno = 0;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTL32K */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Enable External OSC49M */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
	
    /* Enable External OSC10K */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
	
    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKSEL0_HIRCFSEL_48M, CLK_CLKDIV0_HCLK(1));
	
	
    /* Enable IP clock */
    CLK_EnableModuleClock(PWM0_MODULE);
		CLK_EnableModuleClock(UART_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);
		CLK_EnableModuleClock(ADC_MODULE);
		CLK_EnableModuleClock(ANA_MODULE);
CLK_EnableModuleClock(TMR0_MODULE);
    /* Select IP clock source */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL1_PWM0CH01SEL_HCLK, 0);
		
		/* Set ADC divisor from HCLK */
    CLK_SetModuleClock(ADC_MODULE, MODULE_NoMsk, CLK_CLKDIV0_ADC(1));
CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HCLK, 0);
    /* Reset IP */
    SYS_ResetModule(PWM0_RST);
    SYS_ResetModule(UART0_RST);
		SYS_ResetModule(I2C0_RST);	
    SYS_ResetModule(EADC_RST);
		SYS_ResetModule(ANA_RST);
    //SYS_ResetModule(TMR0_RST);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

		/* Set GPB2,GPB3 multi-function pins for I2C0 */
		SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB2MFP_Msk) ) | SYS_GPB_MFP_PB2MFP_I2C_SCL;
		SYS->GPB_MFP  = (SYS->GPB_MFP & (~SYS_GPB_MFP_PB3MFP_Msk) ) | SYS_GPB_MFP_PB3MFP_I2C_SDA;

    /* Lock protected registers */
    SYS_LockReg();
}

void Init_GPIO(void)
{
		/* BLE or WiFi Reset */
	  GPIO_SetMode(platform_gpio_pins[BLE_WIFI_RESET].port, platform_gpio_pins[BLE_WIFI_RESET].pin_number,  GPIO_MODE_OUTPUT);
	  GPIO_SetPinOutHigh(platform_gpio_pins[BLE_WIFI_RESET].port, platform_gpio_pins[BLE_WIFI_RESET].pin_number);
	
		/* MPU6050 Interrupt pin */  
	  GPIO_SetMode(platform_gpio_pins[MPU6050_INT].port, platform_gpio_pins[MPU6050_INT].pin_number, GPIO_MODE_QUASI);
	
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

		DIS_Value |= 0x00;
}

void Init_KEY(void)
{
		/* Set Key 0/1/2 pin to Interrupt mode */
    GPIO_SetMode(platform_gpio_pins[KEY_1].port, platform_gpio_pins[KEY_1].pin_number, GPIO_MODE_QUASI);
		GPIO_SetMode(platform_gpio_pins[KEY_2].port, platform_gpio_pins[KEY_2].pin_number, GPIO_MODE_QUASI);
		GPIO_SetMode(platform_gpio_pins[KEY_3].port, platform_gpio_pins[KEY_3].pin_number, GPIO_MODE_QUASI);


		/* Enable Key 0/1/2 Interrupt mode */
		GPIO_EnableInt(platform_gpio_pins[KEY_1].port, platform_gpio_pins[KEY_1].bit, GPIO_INT_RISING);
		GPIO_EnableInt(platform_gpio_pins[KEY_2].port, platform_gpio_pins[KEY_2].bit, GPIO_INT_RISING);
		GPIO_EnableInt(platform_gpio_pins[KEY_3].port, platform_gpio_pins[KEY_3].bit, GPIO_INT_RISING);

		/* Enbale GPIO Interrupt */
    NVIC_EnableIRQ(GPAB_IRQn);
	  //NVIC_EnableIRQ(EINT0_IRQn);
	  //NVIC_EnableIRQ(EINT1_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_HCLK, GPIO_DBCTL_DBCLKSEL_8192);
	
		/* Enable Key 0/1/2 Debounce */
    GPIO_ENABLE_DEBOUNCE(platform_gpio_pins[KEY_1].port, platform_gpio_pins[KEY_1].pin_number);
    GPIO_ENABLE_DEBOUNCE(platform_gpio_pins[KEY_2].port, platform_gpio_pins[KEY_2].pin_number);
    GPIO_ENABLE_DEBOUNCE(platform_gpio_pins[KEY_3].port, platform_gpio_pins[KEY_3].pin_number);	
}


void Init_PWM0( const platform_pwm_t* pwm, uint32_t frequency, uint32_t duty_cycle )
{	
    /* PWM0 CH X frequency and duty */
    PWM_ConfigOutputChannel(PWM0, pwm->out_channel, frequency, duty_cycle);

		/* Set GPA multi-function pins for PWM0 CH X */
		SYS->GPA_MFP  = (SYS->GPA_MFP & (~pwm->gp_mfp_msk) ) | pwm->gp_mfp;
	    
		/* Enable PWM Output path for PWM0 CH X */
    PWM_EnableOutput(PWM0, 1 << pwm->out_channel);
	 
		/* Enable PWM-Timer X Start/Run */
    PWM_Start(PWM0, 1 << pwm->out_channel);
}

void Init_Bluetooth(void)
{
	  UART_Open(UART0,115200);	// enable UART1 at 9600 baudrate
		UART_SET_RX_FIFO_INTTRGLV(UART0, UART_FIFO_RFITL_4BYTES));
		//UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk);
		UART0->INTEN |= UART_INTEN_RDAIEN_Msk;
		UART0->FIFO |= UART_FIFO_RXRST_Msk;
    NVIC_EnableIRQ(UART0_IRQn);	
}

uint16_t transfer(uint16_t input)
{
		if(input > 0x7FFF)
			return ~(input - 0x8000) + 1;
		else
			return	input;
}

void Send_Data(void)
{
//	int16_t tmp16;
//	int8_t  msb, lsb;
	int16_t R, P, Y;
#if 0
	
	R = transfer(iEuler[0]);
	P = transfer(iEuler[1]);
	Y = transfer(iEuler[2]);
	
	// Transfer Data formate for Android APP
	GyroX = transfer(Read_MPU6050_GyroX());
	GyroY = transfer(Read_MPU6050_GyroY());
	GyroZ = transfer(Read_MPU6050_GyroZ());
#endif
	
	R = 10;//transfer(iEuler[0]);
	P = 20;//transfer(iEuler[1]);
	Y = 30;//transfer(iEuler[2]);
	
	// Transfer Data formate for Android APP
	GyroX = 100;//transfer(Read_MPU6050_GyroX());
	GyroY = 100;//transfer(Read_MPU6050_GyroY());
	GyroZ = 100;//transfer(Read_MPU6050_GyroZ());
	
	DHT11_Data[0]=50<<8;
	DHT11_Data[1]=50<<8;
	LightValue=40<<8;
	GasValue=40<<8;

	Text[4] = (uint8_t) (R >> 8 );
	Text[5] = (uint8_t) (R & 0xFF);
	Text[6] = (uint8_t) (P >> 8 );
	Text[7] = (uint8_t) (P & 0xFF);
	Text[8] = (uint8_t) (Y >> 8 );
	Text[9] = (uint8_t) (Y & 0xFF);
		
	Text[14] = (uint8_t) (GyroZ >> 8 );
	Text[15] = (uint8_t) (GyroZ & 0xFF);
	Text[16] = (uint8_t) (GyroY >> 8 );
	Text[17] = (uint8_t) (GyroY & 0xFF);
	Text[18] = (uint8_t) (GyroX >> 8 );
	Text[19] = (uint8_t) (GyroX & 0xFF);
	
	Text[14] = (uint8_t) (GyroZ >> 8 );
	Text[15] = (uint8_t) (GyroZ & 0xFF);
	Text[16] = (uint8_t) (GyroY >> 8 );
	Text[17] = (uint8_t) (GyroY & 0xFF);
	Text[18] = (uint8_t) (GyroX >> 8 );
	Text[19] = (uint8_t) (GyroX & 0xFF);
	
	Text[24] = (uint8_t) (DHT11_Data[1] >> 8 );
	Text[25] = (uint8_t) (DHT11_Data[1] & 0xFF);
	
	Text[30] = (uint8_t) (DHT11_Data[0] >> 8 );
	Text[31] = (uint8_t) (DHT11_Data[0] & 0xFF);
	
	Text[36] = (uint8_t) (LightValue >> 8 );
	Text[37] = (uint8_t) (LightValue & 0xFF);

	Text[42] = (uint8_t) (GasValue >> 8 );
	Text[43] = (uint8_t) (GasValue & 0xFF);

	Text[48] = DIS_Value;

	UART_Write(UART0, Text, 49);

}

void UART_Init(void)
{
    /* Set GPG multi-function pins for UART0 RXD and TXD */
		SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA8MFP_Msk) ) | SYS_GPA_MFP_PA8MFP_UART_TX;
		SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA9MFP_Msk) ) | SYS_GPA_MFP_PA9MFP_UART_RX;

    /* Configure UART0 and set UART0 Baudrate(115200) */
    UART_Open( UART0,115200 );
}


void ADC_Init(void)
{
	uint32_t u32Div;
	
	/* Reset IP */
	CLK_EnableModuleClock(ADC_MODULE);
	CLK_EnableModuleClock(ANA_MODULE);
    SYS_ResetModule(EADC_RST);
	SYS_ResetModule(ANA_RST);
	
	/* Enable Analog block power */
	ADC_ENABLE_SIGNALPOWER(ADC,
	                       ADC_SIGCTL_ADCMOD_POWER|
						   ADC_SIGCTL_IBGEN_POWER|
	                       ADC_SIGCTL_BUFADC_POWER|
	                       ADC_SIGCTL_BUFPGA_POWER);
	
	/* PGA Setting */
	ADC_MUTEON_PGA(ADC, ADC_SIGCTL_MUTE_PGA);
	ADC_MUTEOFF_PGA(ADC, ADC_SIGCTL_MUTE_IPBOOST);
	//ADC_ENABLE_PGA(ADC, ADC_PGACTL_REFSEL_VMID, ADC_PGACTL_BOSST_GAIN_26DB);
	//ADC_SetPGAGaindB(PGA_GAIN); // 0dB

	ADC_ENABLE_PGA(ADC, ADC_PGACTL_REFSEL_VMID, ADC_PGACTL_BOSST_GAIN_0DB);
	ADC_SetPGAGaindB(PGA_GAIN); // 0dB


	/* MIC circuit configuration */
	ADC_ENABLE_VMID(ADC, ADC_VMID_HIRES_DISCONNECT, ADC_VMID_LORES_CONNECT);
	ADC_EnableMICBias(ADC_MICBSEL_90_VCCA);
	ADC_SetAMUX(ADC_MUXCTL_MIC_PATH, ADC_MUXCTL_POSINSEL_NONE, ADC_MUXCTL_NEGINSEL_NONE);
	
	/* Open ADC block */
	ADC_Open();
	ADC_SET_OSRATION(ADC, ADC_OSR_RATION_192);
	u32Div = CLK_GetHIRCFreq()/ADC_SAMPLE_RATE/192;
	ADC_SET_SDCLKDIV(ADC, u32Div);
	ADC_SET_FIFOINTLEVEL(ADC, 7);
	
	ADC_MUTEOFF_PGA(ADC, ADC_SIGCTL_MUTE_PGA);
	
}

void ADC_SingleModeTest(void)
{
	uint32_t u32ConversionData;
	//uint8_t u8Option, u8InputMode, j;
	
	//printf("\n\n=== ADC single mode test ===\n");
	
	/* Init ADC */
	ADC_Init();

	 while(1)
    {				
        ADC_SetGPIOChannel(ADC_GPIO_SINGLEEND_CH0_N);
                
        // Enable ADC Interrupt function
        ADC_EnableInt(ADC_FIFO_INT);
    
        // Start A/D conversion 
        ADC_START_CONV(ADC);

        // Wait ADC interrupt 
        while(ADC_GetIntFlag(ADC_FIFO_INT));
				while(1){
        //for(j=1;j<=8;j++) {
            u32ConversionData = ADC_GET_FIFODATA(ADC);
            //printf("  0x%X (%d)\n", u32ConversionData, u32ConversionData);
        //}
			}
		
		// stop A/D conversion 
        ADC_STOP_CONV(ADC);
    }   
}

void Init_AHRS()
{
#ifdef SYSTICK
		TIMER_Init();	
		setup_system_tick(1000);
#else
TIMER_Open( TIMER0, TIMER_PERIODIC_MODE, 1000);
TIMER_Start(TIMER0);
TIMER_EnableInt(TIMER0);
NVIC_EnableIRQ(TMR0_IRQn);
ChronographStart(ChronMain);
#endif
		nvtAHRSInit();
	
		//SensorsInit();
		AccOffset[0] = 0;//0;
		AccOffset[1] = 0;//0;
		AccOffset[2] = 0;//0;
		AccScale[0] = IMU_G_PER_LSB_CFG;
		AccScale[1] = IMU_G_PER_LSB_CFG;
		AccScale[2] = IMU_G_PER_LSB_CFG;
		nvtSetAccScale(AccScale);
		nvtSetAccOffset(AccOffset);
		nvtSetAccG_PER_LSB(IMU_G_PER_LSB_CFG);
		
		GyroOffset[0] = 0;
		GyroOffset[1] = 0;
		GyroOffset[2] = 0;
		GyroScale[0] = IMU_DEG_PER_LSB_CFG;
		GyroScale[1] = IMU_DEG_PER_LSB_CFG;
		GyroScale[2] = IMU_DEG_PER_LSB_CFG;
		nvtSetGyroScale(GyroScale);
		nvtSetGyroOffset(GyroOffset);
		nvtSetGYRODegPLSB(IMU_DEG_PER_LSB_CFG);		
}

void SensorsDynamicCalibrate(char SensorType)
{

	if(SensorType&SENSOR_GYRO&&SensorInitState.GYRO_Done) {
		if(!SensorCalState.GYRO_Done) {
			DelayMsec(1);
			if(nvtGyroCenterCalibrate()==STATUS_GYRO_CAL_DONE) {
				float GyroMean[3];
				SensorCalState.GYRO_Done = true;
				nvtGetGyroOffset(GyroMean);
			}
		}
	}
}

void AccCalibrationZ()
{
	signed char status;
	
		nvtCalACCInit();
	do {
		DelayMsec(1);
		rawACC[0] = Read_MPU6050_AccX();
		rawACC[1] = Read_MPU6050_AccY();
		rawACC[2] = Read_MPU6050_AccZ();
		nvtInputSensorRawACC(rawACC);
		status = nvtCalACCBufferFill(0);
	}while(status==STATUS_BUFFER_NOT_FILLED);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Main function                                                                                           */
/*---------------------------------------------------------------------------------------------------------
int32_t main()//SW 1102 test by WeCan suggestion
{
	  int16_t accX, accY, accZ;
	  int16_t gyroX, gyroY, gyroZ;
	
    SYS_Init();	

		printf("Hello, Started.\n\n");

	  I2C_Open(I2C0, 400000);//I2C_Open(I2C1, 50000);	
    Init_MPU6050();
	
    while(1) {
			  accX = Read_MPU6050_AccX();
			  accY = Read_MPU6050_AccY();
			  accZ = Read_MPU6050_AccZ();
			  gyroX= Read_MPU6050_GyroX();
			  gyroY= Read_MPU6050_GyroY();
			  gyroZ= Read_MPU6050_GyroZ();			
			
			  printf("Acc=%6d,%6d,%6d, Gyro=%6d,%6d,%6d\n", accX, accY, accZ, gyroX, gyroY, gyroZ);
    }
}
*/

int main(void)
{	
	  //uint32_t	pwm_frequency = 1000; // HZ
		//uint32_t  pwm_duty_cycle = 30;
		float Euler[3] ;

		uint8_t i=0;

	// Lock protected registers 
    if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    // Init System, IP clock and multi-function I/O 
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    UART_Init();

    //printf("Wecan Chen Oscar Numaker 2015.09.01\r\n");
    //printf("CPU @ %dHz\r\n", SystemCoreClock);
		
		Init_GPIO();
PB->DOUT |= (1 << 0);
GPIO_SetMode(PB, BIT0, GPIO_MODE_OUTPUT);	
		//Init_KEY();
		//Init_PWM0(&platform_pwm_peripherals[NUVOTON_PWM0_CH1], pwm_frequency, pwm_duty_cycle);
		
    // Open I2C0 and set clock to 100k 
    I2C_Open(I2C0, I2CSpeed);
		Init_MPU6050();
		
		//ADC_Init();
		Init_AHRS();
		//ADC_SingleModeTest();
		Init_Bluetooth();

		//Calibrate Gyro
		while (!SensorCalState.GYRO_Done)
		{	
				DelayMsec(1);
				rawGYRO[0] = Read_MPU6050_GyroX();
				rawGYRO[1] = Read_MPU6050_GyroY();
				rawGYRO[2] = Read_MPU6050_GyroZ();
				nvtInputSensorRawGYRO(rawGYRO);
				SensorsDynamicCalibrate (SENSOR_GYRO);
		}
		
		//Calibrate Accel
		AccCalibrationZ();

		GPIO_SetPinOutLow(platform_gpio_pins[LED_MAINB_R].port, platform_gpio_pins[LED_MAINB_R].pin_number);
				
		while(1)
		{

			//ADC_START_CONV(ADC);
#if 1
			rawACC[0] = Read_MPU6050_AccX();
if(I2CFail==1)goto I2CFailed; 			
			rawACC[1] = Read_MPU6050_AccY();
if(I2CFail==1)goto I2CFailed;
			rawACC[2] = Read_MPU6050_AccZ();
if(I2CFail==1)goto I2CFailed;
		
			rawGYRO[0] = Read_MPU6050_GyroX();
if(I2CFail==1)goto I2CFailed;
			rawGYRO[1] = Read_MPU6050_GyroY();
if(I2CFail==1)goto I2CFailed;
			rawGYRO[2] = Read_MPU6050_GyroZ();
if(I2CFail==1)goto I2CFailed;
			nvtInputSensorRawACC(rawACC);
			nvtInputSensorRawGYRO(rawGYRO);
				
			nvtUpdateAHRS(SENSOR_ACC|SENSOR_GYRO);
			nvtGetEulerRPY(Euler);
			for (i=0 ; i<3 ; i++)
					iEuler[i] = (int) (Euler[i]);	
I2CFailed:
if(I2CFail==1)
{
I2CFail=0;
I2C_Close(I2C0);
I2C_Open(I2C0, I2CSpeed);
}
#endif		
#if 1
			if (Flag_report) {
		
				if (strncmp((char *)command,"@STA",4) == 0 ){
					//Buzz(1);Buzz(1);
					sprintf(Text_ACK,"@AKS");
					UART_Write(UART0, Text_ACK, 4);
				}
	
				if (strncmp((char *)command,"@DAT",4) == 0 ){
					sprintf(Text_ACK,"@AKD");
					UART_Write(UART0, Text_ACK, 4);
					//Read_DHT11 (DHT11_Data);
					Send_Data();
				}			
			
				if (strncmp((char *)command,"@STP",4) == 0 ){
					//Buzz(1);Buzz(1);Buzz(1);
					sprintf(Text_ACK,"@AKP");
					UART_Write(UART0, Text_ACK, 4);
				}
				
				if (strncmp((char *)command,"@LB",3) == 0 ){
					DIS_Value = RX_buffer[3];
					if ((DIS_Value& 0x01 ) != 0 ){
						//Buzz(1);
						DIS_Value &= ~0x01;
					}
					
					if ((DIS_Value & 0x02 ) != 0)
						GPIO_SetPinOutLow(platform_gpio_pins[LED_MAINB_R].port, platform_gpio_pins[LED_MAINB_R].pin_number);
					else 
						GPIO_SetPinOutHigh(platform_gpio_pins[LED_MAINB_R].port, platform_gpio_pins[LED_MAINB_R].pin_number);
				
					if ((DIS_Value & 0x04 ) != 0)
						GPIO_SetPinOutLow(platform_gpio_pins[LED_MAINB_G].port, platform_gpio_pins[LED_MAINB_G].pin_number);
					else 
						GPIO_SetPinOutHigh(platform_gpio_pins[LED_MAINB_G].port, platform_gpio_pins[LED_MAINB_G].pin_number);
				
					if ((DIS_Value & 0x08 ) != 0)
						GPIO_SetPinOutLow(platform_gpio_pins[LED_MAINB_B].port, platform_gpio_pins[LED_MAINB_B].pin_number);
				
					else 
						GPIO_SetPinOutHigh(platform_gpio_pins[LED_MAINB_B].port, platform_gpio_pins[LED_MAINB_B].pin_number);
				
					sprintf(Text_ACK,"@AKL");	
					UART_Write(UART0, Text_ACK, 4);
				}
				Flag_report=0;
			}
#endif 	
		}
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

