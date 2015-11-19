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
#ifdef AUDIO_REC_PLAYBACK_ENABLE
#include "App.h"
#include "Framework.h"
//#include "Keypad.h"
#include "SPIFlash.h"
//#include "ConfigSysClk.h"
#include "MicSpk.h"
// SPI flash handler.
S_SPIFLASH_HANDLER g_sSpiFlash;
// Application control.
volatile UINT8 g_u8AppCtrl;
// Application handler.
S_APP g_sApp;
extern void App_Initiate(void);
extern BOOL App_ProcessPlay(void);
extern BOOL App_StopPlay(void);
extern BOOL App_ProcessRec(void);
extern BOOL App_StopRec(void);
extern BOOL App_StartRec(void);
extern BOOL App_StartPlay(void);
extern void App_PowerDown(void);
#endif
extern const platform_gpio_t platform_gpio_pins[];
extern const platform_pwm_t platform_pwm_peripherals[] ;


#ifdef DHT11_ENABLE
//#include "DHT11.h"
#endif
#ifdef MPU6050_ENABLE
#include "./MPU6050/MPU6050_api.h"
//#include "./MPU6050/config.h"

int16_t iEuler[3];

#endif
#ifdef OLED_ENABLE
#include "./OLED/LY096BG30.h"
#endif
#ifdef HDC1000_ENABLE
#include "./HDC1000/HDC1000.h"
#endif

#ifdef ADCSENSOR_ENABLE
#if 1
#define PGA_GAIN    -600  //default
#define ADC_SAMPLE_RATE  (16000)  //default
#else
#define CH          0
#define PDMA        PDMA0
//#define PGA_GAIN    0  //default
#define PGA_GAIN    -600  //default

#define ADC_SAMPLE_RATE  (16000)  //default
#define FRAME_SZIE       (8)
#define BUFFER_LENGTH    (FRAME_SZIE*2)

__align(4) int16_t i16Buffer[BUFFER_LENGTH];
uint16_t DPWMModFreqDiv[8] = {228, 156, 76, 52, 780, 524, 396, 268}; //Modulation Division
volatile uint8_t u8CmpMatch;
#endif

#endif


//volatile uint16_t Humidity,Temp;

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


#ifdef AUDIO_REC_PLAYBACK_ENABLE
UINT8 SPIFlash_Initiate(void)
{ 
	UINT16 ui16Temp;
	UINT32 ui32Temp;
	UINT32 u32Count;

	// SPI0: GPA1=SSB00, GPA2=SCLK0, GPA3=MISO0, GPA4=MOSI0 
	SYS->GPA_MFP  = 
		(SYS->GPA_MFP & (~(SYS_GPA_MFP_PA0MFP_Msk|SYS_GPA_MFP_PA1MFP_Msk|SYS_GPA_MFP_PA2MFP_Msk|SYS_GPA_MFP_PA3MFP_Msk)) )
		| (SYS_GPA_MFP_PA0MFP_SPI_MOSI0|SYS_GPA_MFP_PA1MFP_SPI_SCLK|SYS_GPA_MFP_PA2MFP_SPI_SSB0|SYS_GPA_MFP_PA3MFP_SPI_MISO0);	
	
	// Reset IP module
	CLK_EnableModuleClock(SPI0_MODULE);
	SYS_ResetModule(SPI0_RST);
	SPIFlash_Open(SPI0, SPI_SS0, SPI0_CLOCK, &g_sSpiFlash );

	// Make SPI flash leave power down mode if some where or some time had made it entring power down mode
	SPIFlash_PowerDown(&g_sSpiFlash, FALSE);
	
	// Check SPI flash is ready for accessing
	u32Count = ui32Temp = 0;
	while(u32Count!=100)
	{
		SPIFlash_Read(&g_sSpiFlash, 0, (PUINT8) &ui16Temp, 2);
		if ( ui32Temp != (UINT32)ui16Temp )
		{
			ui32Temp = (UINT32)ui16Temp;
			u32Count = 0;
		}
		else
			u32Count++;
	}

	// The following code can be remove to save code if the flash size is not necessary for this application
	SPIFlash_GetChipInfo(&g_sSpiFlash);
	if (g_sSpiFlash.u32FlashSize == 0)
		return 0;
	
	// The above code can be remove to save code if the flash size is not necessary for this application
	return 1;
}
#endif
#ifdef AUDIO_REC_PLAYBACK_ENABLE
void Record_KeypadHandler(UINT32 u32Param)
{
	if ( g_u8AppCtrl&(APPCTRL_PLAY|APPCTRL_PLAY_STOP) )
		return;
	
	if ( (g_u8AppCtrl&APPCTRL_RECORD) ==0 )
		App_StartRec();
	else
		App_StopRec();			
}

void Playback_KeypadHandler(UINT32 u32Param)
{		
	if ( g_u8AppCtrl&APPCTRL_RECORD )
	   return;
	
	if ( (g_u8AppCtrl&APPCTRL_PLAY) == 0 )
		App_StartPlay();
	else
		App_StopPlay();
}

void PowerDown_KeypadHandler(UINT32 u32Param)
{
	App_PowerDown();
}

#endif

//#ifdef BT_ENABLE
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
	#ifdef DEBUG_BT_WIFI_ENABLE
		printf("%d\n",real_rx_count);
	#endif
	#endif

	}
}
//#endif
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
#if 0 //def BT_ENABLE
void Init_Bluetooth(void)
{
	UART_Open(UART0,115200);	// enable UART1 at 9600 baudrate
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
#endif
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
void DPWM_Init(void)
{
	/* Reset IP */
	CLK_EnableModuleClock(DPWM_MODULE);
	SYS_ResetModule(DPWM_RST);
	
	DPWM_Open();	 
	DPWM_SetSampleRate(ADC_SAMPLE_RATE); //Set sample rate
	DPWM_SET_MODFREQUENCY(DPWM,DPWM_CTL_MODUFRQ0);//Set FREQ_0
	
	/* Set GPG multi-function pins for SPK+ and SPK- */
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA12MFP_Msk) ) | SYS_GPA_MFP_PA12MFP_SPKP;
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA13MFP_Msk) ) | SYS_GPA_MFP_PA13MFP_SPKM;
}
#if 0
void PDMA_Init(void)
{
	volatile int32_t i = 10;
	
	/* Reset IP */
	CLK_EnableModuleClock(PDMA_MODULE);
	SYS_ResetModule(PDMA_RST);

	
	PDMA_GCR->GLOCTL |= (1 << CH) << PDMA_GLOCTL_CHCKEN_Pos; //PDMA Controller Channel Clock Enable
			
	PDMA->DSCT_CTL |= PDMA_DSCT_CTL_SWRST_Msk;   //Writing 1 to this bit will reset the internal state machine and pointers
	PDMA->DSCT_CTL |= PDMA_DSCT_CTL_CHEN_Msk;    //Setting this bit to 1 enables PDMA assigned channel operation 
	while(i--);                                  //Need a delay to allow reset
	
	PDMA_GCR->SVCSEL &= 0xfffff0ff;  //DMA channel is connected to ADC peripheral transmit request.
	PDMA_GCR->SVCSEL |= CH << PDMA_SVCSEL_DPWMTXSEL_Pos;  //DMA channel is connected to DPWM peripheral transmit request.
	
	PDMA->DSCT_ENDSA = (uint32_t)&ADC->DAT;    //Set source address
	PDMA->DSCT_ENDDA = (uint32_t)i16Buffer;    //Set destination address
	
	PDMA->DSCT_CTL |= 0x2 << PDMA_DSCT_CTL_SASEL_Pos;    //Transfer Source address is fixed.
	PDMA->DSCT_CTL |= 0x3 << PDMA_DSCT_CTL_DASEL_Pos;    //Transfer Destination Address is wrapped.
	PDMA->DSCT_CTL |= 0x2 << PDMA_DSCT_CTL_TXWIDTH_Pos;  //One half-word (16 bits) is transferred for every PDMA operation
	PDMA->DSCT_CTL |= 0x1 << PDMA_DSCT_CTL_MODESEL_Pos;  //Memory to IP mode (APB-to-SRAM).
	PDMA->DSCT_CTL |= 0x5 << PDMA_DSCT_CTL_WAINTSEL_Pos; //Wrap Interrupt: Both half and end buffer.
	
	PDMA->TXBCCH = BUFFER_LENGTH*2;          // Audio array total length, unit: sample.
	
	PDMA->INTENCH = 0x1 << PDMA_INTENCH_WAINTEN_Pos;;   //Wraparound Interrupt Enable
	
	ADC_ENABLE_PDMA(ADC);
	
	NVIC_ClearPendingIRQ(PDMA_IRQn);
	NVIC_EnableIRQ(PDMA_IRQn);
	PDMA->DSCT_CTL |= PDMA_DSCT_CTL_TXEN_Msk;    //Start PDMA transfer
}
#endif
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
	#ifdef WEBEE_BLE_ENABLE
	char sendbuffer[128];
	int16_t R, P, Y;
	uint16_t iGyroX,iGyroY,iGyroZ;
	#endif
	//uint16_t tmp0,tmp1=0;

	//uint32_t	pwm_frequency = 1000; // HZ
	//uint32_t  pwm_duty_cycle = 30;
#ifdef HUMITURESENSOR_ENABLE
	uint32_t u32temp=0;
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

#ifdef AUDIO_REC_PLAYBACK_ENABLE
#if 0
	if (! SPIFlash_Initiate())		// Initiate SPI interface and checking flows for accessing SPI flash.
		while(1); 					// loop here for easy debug
#else
	SPIFlash_Initiate();
#endif
#endif
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
#ifdef AUDIO_REC_PLAYBACK_ENABLE
	PDMA_INITIATE();				// Initiate PDMA.
//PDMA_Init();
			// After initiation, the PDMA engine clock NVIC are enabled.
			// Use PdmaCtrl_Open() to set PDMA service channel for desired IP.
			// Use PdmaCtrl_Start() to trigger PDMA operation.
			// Reference "PdmaCtrl.h" for PDMA related APIs.
			// PDMA_INITIATE() must be call before SPK_INITIATE() and MIC_INITIATE(), if open MIC or speaker.

	SPK_INITIATE();					// Initiate speaker including pop-sound canceling.
//	DPWM_Init();
			// After initiation, the APU is paused.
			// Use SPK_Resume(0) to start APU operation.
			// Reference "MicSpk.h" for speaker related APIs.

	MIC_INITIATE();					// Initiate MIC.
			// After initiation, the ADC is paused.
			// Use ADC_Resume() to start ADC operation.
			// Reference "MicSpk.h" for MIC related APIs.

											
	App_Initiate();					// Initiate application for audio decode.
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
#if 0 //def BT_ENABLE
	Init_Bluetooth();
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

	while(1)
	{

		//ERICYANG FOR TEST BREAK POINT ONLY
		i=real_rx_count;
		i=real_rx_count;
		i=real_rx_count;

#ifdef AUDIO_REC_PLAYBACK_ENABLE
		if ( g_u8AppCtrl&APPCTRL_RECORD )
		{
			if ( App_ProcessRec() == FALSE )
				App_StopRec();
		}
		else if ( g_u8AppCtrl&APPCTRL_PLAY )
		{
			if ( App_ProcessPlay() == FALSE )
				App_StopPlay();
		}
		

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
		#ifdef AUDIO_REC_PLAYBACK_ENABLE
			Playback_KeypadHandler(0);
		#endif
		}
		else if(KEY3_Flag)
		{
			KEY3_Flag = 0;
			#ifdef OLED_ENABLE
			#if 0//def GASSENSOR_ENABLE
			sprintf(String,"%x",GasValue);
			#else
			sprintf(String,"KEY0 Pressed");
			#ifdef WEBEE_BLE_ENABLE  //ericyang 20151112
			#ifdef MPU6050_ENABLE
			memset(sendbuffer,0,sizeof(sendbuffer));
			R = transfer(iEuler[0]);
			P = transfer(iEuler[1]);
			Y = transfer(iEuler[2]);
		
		// Transfer Data formate for Android APP
			iGyroX = transfer(rawGYRO[0]);
			iGyroY = transfer(rawGYRO[1]);
			iGyroZ = transfer(rawGYRO[2]);

			sprintf(sendbuffer,"DP+T:%d C  H:%d%% \nR: %x P: %x Y: %x \nGyroX: %x\nGyroY: %x\nGyroZ: %x\nLight : %x \nGas  : %x \nIR   : %x\n",
				HUMITURESENSOR_Data[1]>>8,HUMITURESENSOR_Data[0]>>8,R,P,Y,iGyroX,iGyroY,iGyroZ,
			LightValue,GasValue,IRValue);
			UART_Write_Wecan(UART0, sendbuffer, strlen(sendbuffer));
			#endif
//			Send_Data();
			#endif
			#endif
			print_Line(3, String);
			#endif
#if 0//ericyang test webee BLE
				sprintf(Text_ACK,"@AKS");
				UART_Write_Wecan(UART0, Text_ACK, 4);
#endif
		#ifdef AUDIO_REC_PLAYBACK_ENABLE
			Record_KeypadHandler(0);
		#endif
		}
	

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
#ifdef DEBUG_BT_WIFI_ENABLE
	//	UART_Write( UART0, GLOBAL_BUF, real_rx_count );
		real_rx_count = 0;
#endif

#endif 	
	}
}
/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

