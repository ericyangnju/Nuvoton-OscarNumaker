//
// HDC1000 Driver: Low Power, High Accuracy Digital Humidity Sensor with Temperature Sensor 
//
#include <stdio.h>
#include <stdint.h>
#include "isd9100.h"
#include "i2c.h"
#include "HDC1000.h"
#include "../I2c0int.h"  //ericyang 20151110 test

//#define I2C_ACK(i2c) ( (i2c)->CTL = ((i2c)->CTL & ~I2C_CTL_SI_Msk) | I2C_CTL_AA_Msk )


void HDC1000_I2C_SingleWrite(uint8_t index, uint16_t data)
{
#ifdef I2C_IRQ
	g_u8DeviceAddr = HDC1000_I2C_SLA;
		
	g_au8TxData[0] = index;//(uint8_t)();
	g_au8TxData[1] = data>>8;//(uint8_t)();
	g_au8TxData[2] = data|0xff;//(uint8_t)();

	g_u8DataLen = 0;
	g_u8EndFlag = 0;	
#ifdef I2C_TIMEOUT_ENABLE
	i2c_novalidack_timeout = 0;//ericyang 20151111
	i2c_unreachable_timeout = 0;
#endif
	s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx_var;
	I2C_SET_CONTROL_REG(I2C0, I2C_STA);	
#ifdef I2C_TIMEOUT_ENABLE
	while (g_u8EndFlag == 0)
	{
		if(i2c_unreachable_timeout++>=I2C_TIMEOUT_UNREACHABLE_COUNT)
			break;
	}
#else
	while (g_u8EndFlag == 0);
#endif	
#endif
}

uint16_t HDC1000_I2C_SingleRead(uint8_t index)
{
	uint16_t tmp;
#ifdef I2C_IRQ
	/* I2C function to read data from slave */
	s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx_var;

	g_u8DataLen = 0;
	g_u8EndFlag = 0;
#ifdef I2C_TIMEOUT_ENABLE
	i2c_novalidack_timeout = 0;//ericyang 20151111
	i2c_unreachable_timeout = 0;
#endif
	g_u8DeviceAddr = HDC1000_I2C_SLA;
	g_au8TxData[0] = index;//(uint8_t)();
		
	I2C_SET_CONTROL_REG(I2C0, I2C_STA);

	/* Wait I2C Rx Finish */
#ifdef I2C_TIMEOUT_ENABLE
	while (g_u8EndFlag == 0)
	{
		if(i2c_unreachable_timeout++>=I2C_TIMEOUT_UNREACHABLE_COUNT)
			break;
	}
#else
	while (g_u8EndFlag == 0);
#endif
	tmp=g_u8RxData<<8|g_u8RxData1;
#ifdef DEBUG_ENABLE
	printf("received hdc1000 data: %x \n", tmp);
#endif
#endif
	return tmp;
}

void Init_HDC1000(void)
{
 	I2C_SetSlaveAddr(HDC1000_I2C_PORT, 0, HDC1000_I2C_SLA, I2C_GCMODE_DISABLE);	
	//Read_HDC1000_Device_ID();
	//Read_HDC1000_Manufacturer_ID();
	
	//HDC1000_I2C_SingleWrite(HDC1000_Configuration_Reg, ( HDC1000_RST ) >>8  );	// CLL_SEL=0: internal 8MHz, TEMP_DIS=0, SLEEP=0 
//	HDC1000_I2C_SingleWrite(HDC1000_Configuration_Reg, ( HDC1000_MODE | HDC1000_HEAT) >>8  );	// CLL_SEL=0: internal 8MHz, TEMP_DIS=0, SLEEP=0 
	HDC1000_I2C_SingleWrite(HDC1000_Configuration_Reg,  HDC1000_MODE | HDC1000_HEAT  );	// CLL_SEL=0: internal 8MHz, TEMP_DIS=0, SLEEP=0 

	//HDC1000_I2C_SingleWrite(HDC1000_SMPLRT_DIV, 0x01);  // Gyro output sample rate = Gyro Output Rate/(1+SMPLRT_DIV)
	//HDC1000_I2C_SingleWrite(HDC1000_CONFIG, 0x01);      // set TEMP_OUT_L, DLPF=2 (Fs=1KHz)
	//HDC1000_I2C_SingleWrite(HDC1000_GYRO_CONFIG, 0x18); // bit[4:3] 0=+-250d/s,1=+-500d/s,2=+-1000d/s,3=+-2000d/s
	//HDC1000_I2C_SingleWrite(HDC1000_ACCEL_CONFIG, 0x00);// bit[4:3] 0=+-2g,1=+-4g,2=+-8g,3=+-16g, ACC_HPF=On (5Hz)
}

uint16_t Read_HDC1000_Device_ID(void)
{
	uint16_t Device_ID;
	Device_ID = HDC1000_I2C_SingleRead(HDC1000_Device_ID_Reg); // read HDC1000 Device ID
	return Device_ID;
}

uint16_t Read_HDC1000_Manufacturer_ID(void)
{
	uint16_t Manufacturer_ID;
	Manufacturer_ID = HDC1000_I2C_SingleRead(HDC1000_Manufacturer_ID_Reg); // read ID of Texas Instruments
	return Manufacturer_ID;
}

uint16_t Read_HDC1000_Configuration(void)
{
	uint16_t Configuration_Reg;
	Configuration_Reg = HDC1000_I2C_SingleRead(HDC1000_Configuration_Reg); // read ID of Texas Instruments
	return Configuration_Reg;
}

uint16_t Read_HDC1000_Temperature(void)
{
	uint16_t Temperature_Reg;
	Temperature_Reg = HDC1000_I2C_SingleRead(HDC1000_Temperature_Reg); // read ID of Texas Instruments
	return Temperature_Reg;
}

uint16_t Read_HDC1000_Humidity(void)
{
	uint16_t Humidity_Reg;
	Humidity_Reg = HDC1000_I2C_SingleRead(HDC1000_Humidity_Reg); // read ID of Texas Instrument
	return Humidity_Reg;
}


