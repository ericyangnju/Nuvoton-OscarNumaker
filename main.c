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
#include "./GPIO/user_gpio.h"
#include "config.h"
extern const platform_gpio_t platform_gpio_pins[];
extern const platform_pwm_t platform_pwm_peripherals[] ;

#ifdef DHT11_ENABLE
//#include "DHT11.h"
#endif
#ifdef MPU6050_ENABLE
#include "./MPU6050/MPU6050_api.h"
int16_t iEuler[3];
#endif
#ifdef OLED_ENABLE
#include "./OLED/LY096BG30.h"
#endif
#ifdef HDC1000_ENABLE
#include "./HDC1000/HDC1000.h"
#endif
#ifdef ADCSENSOR_ENABLE
#define PGA_GAIN    -600  //default
#define ADC_SAMPLE_RATE  (16000)  //default
#endif

#ifdef NUVOTON_BLE_WIFI_ENABLE
#define RXBUFSIZE  4
char     Text[49] = {'@','A','C','C',0,0,0,0,0,0, //offset 4
															'@','G','Y','R',0,0,0,0,0,0, //offset 14
															'@','T','E','M',0,0,  //offset 24
															'@','H','R','W',0,0, //offset 30
															'@','L','I','R',0,0, //offset 36
															'@','G','A','S',0,0, //offset 42
															'@','D','I','S',0,  //offset 48
															}	; 

char    Text_ACK[4];
uint8_t RX_buffer[RXBUFSIZE];
uint8_t command[4];
uint8_t Flag_report =0;
uint8_t DIS_Value = 0;;															
//uint8_t uartno;	
#else
uint8_t GLOBAL_BUF[256];//ericyang 20151118
uint8_t real_rx_count=0;
#endif
uint8_t KEY1_Flag = 0;
uint8_t KEY2_Flag = 0;
uint8_t KEY3_Flag = 0;
//volatile uint32_t g_u32TICK = 0,Timer_F;
#ifdef LIGHTSENSOR_ENABLE
uint32_t LightValue;
#endif
#ifdef GASSENSOR_ENABLE
uint32_t GasValue;	
#endif
#ifdef IRSENSOR_ENABLE
uint32_t IRValue;
#endif
#if 1//def HUMITURESENSOR_ENABLE
//volatile uint16_t time_DHT11[42];
uint16_t HUMITURESENSOR_Data[2]={0x4000,0x2000};//humidity tempreture;
#endif
char String[16];
//volatile uint32_t capture_count =0;
															

/**
 *    @brief    The function is to write data into TX buffer to transmit data by UART.
 *
 *    @param[in]    uart            The base address of UART module.
 *    @param[in]    pu8TxBuf        The buffer to send the data to UART transmission FIFO.
 *    @param[in]    u32WriteBytes    The byte number of data.
 *
 *  @return u32Count: transfer byte count
 */
uint32_t UART_Write_Wecan(UART_T* uart,char *pu8TxBuf, uint32_t u32WriteBytes)
{
	uint32_t  u32Count, u32delayno;

	for(u32Count=0; u32Count != u32WriteBytes; u32Count++) 
	{
		u32delayno = 0;
		while((uart->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0) 
		{ /* Wait Tx empty and Time-out manner */
			u32delayno++;
			if( u32delayno >= 0x40000000 )
	    			return FALSE;
		}
		uart->DAT = pu8TxBuf[u32Count];    /* Send UART Data from buffer */
	}
	return u32Count;
}															

/**
 *    @brief    Set Rx FIFO interrupt Trigger Level
 *
 *    @param    uart        The base address of UART module
 *    @param    u32TriggerLevel   RX FIFO interrupt Trigger Level. ( \ref UART_FIFO_RFITL_1BYTE / \ref UART_FIFO_RFITL_4BYTE / 
 *                                                                   \ref UART_FIFO_RFITL_8BYTE / \ref UART_TLCTL_RFITL_14BYTES )
 *    @return    None
 */
#define UART_SET_RX_FIFO_INTTRGLV(uart, u32TriggerLevel)   (((uart)->FIFO  = ((uart)->FIFO  &  ~UART_FIFO_RFITL_Msk) | (u32TriggerLevel))

void UART0_IRQHandler(void)
{
	uint32_t u32IntSts= UART0->INTSTS;
	#ifdef NUVOTON_BLE_WIFI_ENABLE
	uint8_t i;
	#else
	uint8_t u8count = 0;
	#endif
	if(u32IntSts & UART_IS_RX_READY(UART0)) 
	{	
	#ifdef NUVOTON_BLE_WIFI_ENABLE
		UART_Read(UART0, RX_buffer, RXBUFSIZE);
		for(i=0;i<4;i++)
			command[i]=RX_buffer[i];
		Flag_report=1;
		//print_Line(3,RX_buffer);
		//uartno = 0;
	#else
		u8count = UART_Read(UART0, GLOBAL_BUF+real_rx_count, 256);
		real_rx_count += u8count;
	#endif

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

	/* Select IP clock source */
	CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL1_PWM0CH01SEL_HCLK, 0);

	/* Set ADC divisor from HCLK */
	CLK_SetModuleClock(ADC_MODULE, MODULE_NoMsk, CLK_CLKDIV0_ADC(1));

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
uint16_t transfer(uint16_t input)
{
	if(input > 0x7FFF)
		return ~(input - 0x8000) + 1;
	else
		return	input;
}
#ifdef NUVOTON_BLE_WIFI_ENABLE
void Send_Data(void)
{
	//int16_t tmp16;
	//int8_t  msb, lsb;
	int16_t R, P, Y;
#ifdef MPU6050_ENABLE	
	R = transfer(iEuler[0]);
	P = transfer(iEuler[1]);
	Y = transfer(iEuler[2]);
	// Transfer Data formate for Android APP
	GyroX = transfer(Read_MPU6050_GyroX());
	GyroY = transfer(Read_MPU6050_GyroY());
	GyroZ = transfer(Read_MPU6050_GyroZ());
#endif	
	Text[4] = (uint8_t) (R >> 8 );
	Text[5] = (uint8_t) (R & 0xFF);
	Text[6] = (uint8_t) (P >> 8 );
	Text[7] = (uint8_t) (P & 0xFF);
	Text[8] = (uint8_t) (Y >> 8 );
	Text[9] = (uint8_t) (Y & 0xFF);
#ifdef MPU6050_ENABLE
	Text[14] = (uint8_t) (GyroZ >> 8 );
	Text[15] = (uint8_t) (GyroZ & 0xFF);
	Text[16] = (uint8_t) (GyroY >> 8 );
	Text[17] = (uint8_t) (GyroY & 0xFF);
	Text[18] = (uint8_t) (GyroX >> 8 );
	Text[19] = (uint8_t) (GyroX & 0xFF);
	
//	Text[14] = (uint8_t) (GyroZ >> 8 );
//	Text[15] = (uint8_t) (GyroZ & 0xFF);
//	Text[16] = (uint8_t) (GyroY >> 8 );
//	Text[17] = (uint8_t) (GyroY & 0xFF);
//	Text[18] = (uint8_t) (GyroX >> 8 );
//	Text[19] = (uint8_t) (GyroX & 0xFF);
#endif	
#ifdef HUMITURESENSOR_ENABLE
	Text[24] = (uint8_t) (HUMITURESENSOR_Data[1] >> 8 );
	Text[25] = (uint8_t) (HUMITURESENSOR_Data[1] & 0xFF);
	Text[30] = (uint8_t) (HUMITURESENSOR_Data[0] >> 8 );
	Text[31] = (uint8_t) (HUMITURESENSOR_Data[0] & 0xFF);
#endif	
#ifdef LIGHTSENSOR_ENABLE
	Text[36] = (uint8_t) (LightValue >> 8 );
	Text[37] = (uint8_t) (LightValue & 0xFF);
#endif
#ifdef GASSENSOR_ENABLE
	Text[42] = (uint8_t) (GasValue >> 8 );
	Text[43] = (uint8_t) (GasValue & 0xFF);
#endif
	Text[48] = DIS_Value;

	UART_Write_Wecan(UART0, Text, 49);
}
#else
void Send_Data(void)
{
	//char tmpString[16];
	char sendbuffer[128];

	#ifdef MPU6050_ENABLE
	int16_t R, P, Y;
	uint16_t iGyroX,iGyroY,iGyroZ;
	#endif

	#ifdef MPU6050_ENABLE
	R = transfer(iEuler[0]);
	P = transfer(iEuler[1]);
	Y = transfer(iEuler[2]);

// Transfer Data formate for Android APP
	iGyroX = transfer(rawGYRO[0]);
	iGyroY = transfer(rawGYRO[1]);
	iGyroZ = transfer(rawGYRO[2]);
	#endif
	memset(sendbuffer,0,sizeof(sendbuffer));
		
	sprintf(sendbuffer,"L0+\nT:%d C  H:%d%% \nR: %x P: %x Y: %x \nGyroX: %x\nGyroY: %x\nGyroZ: %x\nLight : %x \nGas  : %x \nIR   : %x\n",
		HUMITURESENSOR_Data[1]>>8,HUMITURESENSOR_Data[0]>>8,R,P,Y,iGyroX,iGyroY,iGyroZ,
	LightValue,GasValue,IRValue);

	#if 0// not use, take too many seconds, the system will down
	#ifdef HUMITURESENSOR_ENABLE
	memset(tmpString,0,sizeof(tmpString));
	sprintf(tmpString,"T:%d C  H:%d%% \n",HUMITURESENSOR_Data[1]>>8,HUMITURESENSOR_Data[0]>>8);
	strcat(sendbuffer,tmpString);
	#endif
	#ifdef MPU6050_ENABLE
	memset(tmpString,0,sizeof(tmpString));
	sprintf(tmpString,"R: %x P: %x Y: %x \nGyroX: %x\nGyroY: %x\nGyroZ: %x\n",R,P,Y,iGyroX,iGyroY,iGyroZ);
	strcat(sendbuffer,tmpString);
	#endif
	#ifdef LIGHTSENSOR_ENABLE
	memset(tmpString,0,sizeof(tmpString));
	sprintf(tmpString,"Light : %x \n",LightValue);
	strcat(sendbuffer,tmpString);
	#endif
	#ifdef GASSENSOR_ENABLE
	memset(tmpString,0,sizeof(tmpString));
	sprintf(tmpString,"Gas  : %x \n",GasValue);
	strcat(sendbuffer,tmpString);
	#endif
	#ifdef IRSENSOR_ENABLE
	memset(tmpString,0,sizeof(tmpString));
	sprintf(tmpString,"IR   : %x\n",IRValue);
	strcat(sendbuffer,tmpString);
	#endif
	#endif
	UART_Write_Wecan(UART0, sendbuffer, strlen(sendbuffer));
}
#endif
void UART_Init(void)
{
	/* Set GPG multi-function pins for UART0 RXD and TXD */
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA8MFP_Msk) ) | SYS_GPA_MFP_PA8MFP_UART_TX;
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA9MFP_Msk) ) | SYS_GPA_MFP_PA9MFP_UART_RX;

	/* Configure UART0 and set UART0 Baudrate(115200) */
	UART_Open( UART0,115200 );

	#ifdef NUVOTON_BLE_WIFI_ENABLE//ericyang 20151118 test polling not use interrupt
	UART_SET_RX_FIFO_INTTRGLV(UART0, UART_FIFO_RFITL_4BYTES));
	#else
	UART_SET_RX_FIFO_INTTRGLV(UART0, UART_FIFO_RFITL_1BYTE));
	#endif
	//UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk);
	UART0->INTEN |= UART_INTEN_RDAIEN_Msk;
	UART0->FIFO |= UART_FIFO_RXRST_Msk;
	NVIC_EnableIRQ(UART0_IRQn);		
}

#ifdef ADCSENSOR_ENABLE
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
#endif
#ifdef ADCSENSOR_ENABLE
uint32_t ADC_SingleModeTest(void)
{
	uint32_t u32ConversionData;
	uint8_t j;
	                
	// Enable ADC Interrupt function
	ADC_EnableInt(ADC_FIFO_INT);

	// Start A/D conversion 
	ADC_START_CONV(ADC);

	// Wait ADC interrupt 
	while(ADC_GetIntFlag(ADC_FIFO_INT));

	for(j=1;j<=8;j++) 
	{
		DelayMsec(1);;
		u32ConversionData = ADC_GET_FIFODATA(ADC);
	}
	// stop A/D conversion 
	ADC_STOP_CONV(ADC);
	ADC_DisableInt(ADC_FIFO_INT);
	return u32ConversionData;
}
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* Main function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
//	uint16_t tmp0,tmp1,tmp2=0;

int main(void)
{	
	char tmpString[16];
	uint8_t checkSensorDataFlag = 0;
#if 0 //ndef NUVOTON_BLE_WIFI_ENABLE
	static uint32_t systickcounter = 0;
#endif
	uint32_t u32temp=0;

#ifdef HUMITURESENSOR_ENABLE
	uint8_t u8tmp0 = 0;
	uint8_t u8tmp1 = 0;

#endif
#ifdef MPU6050_ENABLE
	float Euler[3] ;
//	uint8_t i=0;
#endif
	uint8_t i=0;
	/* Lock protected registers */
	if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
		SYS_LockReg();

	/* Init System, IP clock and multi-function I/O */
	SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.
	UART_Init();

    //printf("Wecan Chen Oscar Numaker 2015.09.01\r\n");
    //printf("CPU @ %dHz\r\n", SystemCoreClock);

	Init_GPIO();
#if 1
	PB->DOUT |= (1 << 0);
	GPIO_SetMode(PB, BIT0, GPIO_MODE_OUTPUT);
#endif
	Init_KEY();
	//Init_PWM0(&platform_pwm_peripherals[NUVOTON_PWM0_CH1], pwm_frequency, pwm_duty_cycle);
		
	/* Open I2C0 and set clock to 100k */
	I2C_Open(I2C0, 400000);
#ifdef I2C_IRQ
	I2C_EnableInt(I2C0);
	NVIC_EnableIRQ(I2C0_IRQn);
	NVIC_SetPriority(I2C0_IRQn, 0);
#endif
#ifdef MPU6050_ENABLE
	Init_MPU6050();
#endif
#ifdef OLED_ENABLE
	Init_LCD();
	clear_LCD();

	print_Line(0, "Oscar Numaker V0");
//	print_Line(1, "2015.11.12      ");
//	print_Line(2, "Eric Yang      ");
//	print_Line(3, "0.96 OLED 128x64");
#endif

#ifdef HDC1000_ENABLE
	Init_HDC1000();
#endif
#ifdef ADCSENSOR_ENABLE
	ADC_Init();
#endif
#ifdef MPU6050_ENABLE
	Init_AHRS();
#endif
#ifdef MPU6050_ENABLE
	//Calibrate Gyro
	GyroCalibrate();
	//Calibrate Accel
	AccCalibrationZ();
#endif
	GPIO_SetPinOutLow(platform_gpio_pins[LED_MAINB_R].port, platform_gpio_pins[LED_MAINB_R].pin_number);
	GPIO_SetPinOutLow(platform_gpio_pins[LED_SENB_B].port, platform_gpio_pins[LED_SENB_B].pin_number);
	GPIO_SetPinOutLow(platform_gpio_pins[LED_SENB_R].port, platform_gpio_pins[LED_SENB_R].pin_number);
	GPIO_SetPinOutLow(platform_gpio_pins[LED_SENB_G].port, platform_gpio_pins[LED_SENB_G].pin_number);
#ifndef NUVOTON_BLE_WIFI_ENABLE
	memset(GLOBAL_BUF,0x0,sizeof(GLOBAL_BUF));
#endif
	while(1)
	{
#if 0//ndef NUVOTON_BLE_WIFI_ENABLE
		checkSensorDataFlag = 0;
		if(real_rx_count==0)
		{
			u32temp = getTickCount(); 
			u32temp >>= 10;
			if(systickcounter != u32temp)
			{
				systickcounter = u32temp;
				//if(systickcounter%3 == 0)  // set 3s to refresh sensor data
				{
					checkSensorDataFlag  = 1;
			//	 printf("10s int ready\n");
				}
			}
		}
#else
		checkSensorDataFlag = 1;
#endif
		if(KEY1_Flag)
		{
			KEY1_Flag = 0;
			#ifdef OLED_ENABLE
			#if 0//def IRSENSOR_ENABLE
			sprintf(String,"%x",IRValue);
			#else
			sprintf(String,"KEY2 Pressed");
			#endif
			print_Line(3, String);
			#endif
		}
		else if(KEY2_Flag)
		{
			KEY2_Flag = 0;
			#ifdef OLED_ENABLE
			#if 0//def LIGHTSENSOR_ENABLE
			sprintf(String,"%x",LightValue);
			#else
			sprintf(String,"KEY1 Pressed");

			#endif
			print_Line(3, String);
			#endif
			//printf("KEY2 Press %d\r\n",Count);
		}
		else if(KEY3_Flag)
		{
			KEY3_Flag = 0;
			#ifdef OLED_ENABLE
			sprintf(String,"KEY0 Pressed");
			print_Line(3, String);
			#endif

			#if 0
			sprintf(sendbuffer,"L0+T:%d C  H:%d%% \nR: %x P: %x Y: %x \nGyroX: %x\nGyroY: %x\nGyroZ: %x\nLight : %x \nGas  : %x \nIR   : %x\n",
				HUMITURESENSOR_Data[1]>>8,HUMITURESENSOR_Data[0]>>8,R,P,Y,iGyroX,iGyroY,iGyroZ,
			LightValue,GasValue,IRValue);
			#endif
			#ifndef NUVOTON_BLE_WIFI_ENABLE
			Send_Data();
			#endif
		}
	
		if(checkSensorDataFlag)
		{
#ifdef IRSENSOR_ENABLE
			ADC_SetGPIOChannel(ADC_GPIO_SINGLEEND_CH0_N);			
			IRValue = ADC_SingleModeTest();
		#ifdef DEBUG_ENABLE
			printf("IRSensor: %x\n",IRValue);
		#endif
#endif
#ifdef LIGHTSENSOR_ENABLE
			ADC_SetGPIOChannel(ADC_GPIO_SINGLEEND_CH1_N);			
			LightValue = ADC_SingleModeTest();
		#ifdef DEBUG_ENABLE
			printf("LightValue: %x\n",LightValue);
		#endif

#endif
#ifdef GASSENSOR_ENABLE			
			ADC_SetGPIOChannel(ADC_GPIO_SINGLEEND_CH4_N);			
			GasValue = ADC_SingleModeTest();
		#ifdef DEBUG_ENABLE
			printf("GasValue: %x\n",GasValue);
		#endif

#endif
#ifdef ADCSENSOR_ENABLE
			memset(tmpString,0,sizeof(tmpString));
			sprintf(tmpString,"Light:%x ",LightValue);
			print_Line(2, tmpString);
			memset(tmpString,0,sizeof(tmpString));
			sprintf(tmpString,"Gas:%x IR:%x",GasValue,IRValue);
			print_Line(3, tmpString);
#endif
		
		
		//ADC_START_CONV(ADC);
#ifdef MPU6050_ENABLE
			rawACC[0] = Read_MPU6050_AccX();
			rawACC[1] = Read_MPU6050_AccY();
			rawACC[2] = Read_MPU6050_AccZ();
		
			rawGYRO[0] = Read_MPU6050_GyroX();
			rawGYRO[1] = Read_MPU6050_GyroY();
			rawGYRO[2] = Read_MPU6050_GyroZ();
			nvtInputSensorRawACC(rawACC);
			nvtInputSensorRawGYRO(rawGYRO);
				
			nvtUpdateAHRS(SENSOR_ACC|SENSOR_GYRO);
			nvtGetEulerRPY(Euler);
			for (i=0 ; i<3 ; i++)
				iEuler[i] = (int) (Euler[i]);	

	//printf("iEuler[]=%x %x %x %x \n",iEuler[0],iEuler[1],iEuler[2],iEuler[3]);s				
#endif

#ifdef HDC1000_ENABLE
		//	tmp0 = Read_HDC1000_Manufacturer_ID();
			u32temp = Read_HDC1000_Temperature();
			u32temp = (( u32temp*16500)>>16)-4000;
			u8tmp0 = u32temp/100;
			u8tmp1 = u32temp%100;
			HUMITURESENSOR_Data[1] = u8tmp0<<8|u8tmp1;
			u32temp = Read_HDC1000_Humidity();
			u32temp = (u32temp*10000)>>16;
			u8tmp0 = u32temp/100;
			u8tmp1 = u32temp%100;
			HUMITURESENSOR_Data[0] = u8tmp0<<8|u8tmp1;
#ifdef DEBUG_ENABLE
			printf("tempriture is:%x  humidity is:%x \n",HUMITURESENSOR_Data[1],HUMITURESENSOR_Data[0]);
#endif
#ifdef OLED_ENABLE
			memset(tmpString,0,sizeof(tmpString));
			sprintf(tmpString,"T:%d C   H:%d%%  ",HUMITURESENSOR_Data[1]>>8,HUMITURESENSOR_Data[0]>>8);
			print_Line(1, tmpString);
#endif
#endif	
		}

#ifdef NUVOTON_BLE_WIFI_ENABLE
		if (Flag_report) 
		{
			if (strncmp((char *)command,"@STA",4) == 0 )
			{
				//Buzz(1);Buzz(1);
				sprintf(Text_ACK,"@AKS");
				UART_Write_Wecan(UART0, Text_ACK, 4);
			}
			if (strncmp((char *)command,"@DAT",4) == 0 )
			{
				sprintf(Text_ACK,"@AKD");
				UART_Write_Wecan(UART0, Text_ACK, 4);
#ifdef DHT11_ENABLE
				//Read_DHT11 (DHT11_Data);
#endif
				Send_Data();					
			}			
			if (strncmp((char *)command,"@STP",4) == 0 )
			{
				//Buzz(1);Buzz(1);Buzz(1);
				sprintf(Text_ACK,"@AKP");
				UART_Write_Wecan(UART0, Text_ACK, 4);
			}
			if (strncmp((char *)command,"@LB",3) == 0 )
			{
				DIS_Value = RX_buffer[3];
				if ((DIS_Value& 0x01 ) != 0 )
				{
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
				UART_Write_Wecan(UART0, Text_ACK, 4);
			}			
			Flag_report=0;
		}
#else
		if(real_rx_count)
		{
			real_rx_count = 0;
			if (strstr((const char*)GLOBAL_BUF,"DAT"))
			{
				Send_Data();
			}	
			else
			{
				sprintf(tmpString,"L0+\nInvalid Cmd\n");	
				UART_Write_Wecan(UART0,tmpString, 16);
			}
			memset(GLOBAL_BUF,0x0,sizeof(GLOBAL_BUF));
		}
#endif 	
	}
}
/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

