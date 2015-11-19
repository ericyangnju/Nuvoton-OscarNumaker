//
// MPU6050 Driver: 3-axis Gyroscope + 3-axis accelerometer + temperature
//
// Interface: I2C
// pin1: Vcc to Vcc (+5V)
// pin2: Gnd to Gnd
// pin3: SCL to I2C1_SCL/PC10
// pin4: SDA to I2C1_SDA/PC11
// pin5: XDA -- N.C.
// pin6: XCL -- N.C.
// pin7: AD0 -- N.C.
// pin8: INT -- N.C.

#include <stdio.h>
#include <stdint.h>
#include "isd9100.h"
//#include "sys.h"
//#include "gpio.h"
#include "i2c.h"
#include "MPU6050.h"

#include "config.h"

#ifdef I2C_IRQ
uint8_t g_u8DeviceAddr;
uint8_t g_au8TxData[2];//3];
volatile uint8_t g_u8RxData;
volatile uint8_t g_u8DataLen;
volatile uint8_t g_u8EndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_I2C0HandlerFn = NULL;


void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0)) {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    } else {
        if (s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08) {                    /* START has been transmitted and prepare SLA+W */
        I2C_SET_DATA(I2C0, g_u8DeviceAddr);//(g_u8DeviceAddr << 1)); /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_STO | I2C_SI);
    } else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
        if (g_u8DataLen != 1){//2) {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_SI);
        } else {
            I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_SI);
        }
    } else if (u32Status == 0x10) {             /* Repeat START has been transmitted and prepare SLA+R */
        I2C_SET_DATA(I2C0, g_u8DeviceAddr  | 0x01);//(g_u8DeviceAddr << 1) | 0x01);  /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x40) {             /* SLA+R has been transmitted and ACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x58) {             /* DATA has been received and NACK has been returned */
        g_u8RxData = I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
        g_u8EndFlag = 1;
    } else {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08) {                    /* START has been transmitted */
        I2C_SET_DATA(I2C0, g_u8DeviceAddr);//g_u8DeviceAddr << 1);  /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_STO | I2C_SI);
    } else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
        if (g_u8DataLen != 2){//3) {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_SI);
        } else {
            I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
            g_u8EndFlag = 1;
        }
    } else {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}
#endif

void MPU6050_I2C_SingleWrite(uint8_t index, uint8_t data)
{
#ifdef I2C_IRQ
g_u8DeviceAddr = MPU6050_I2C_SLA;
	
g_au8TxData[0] = index;//(uint8_t)();
g_au8TxData[1] = data;//(uint8_t)();

g_u8DataLen = 0;
g_u8EndFlag = 0;	

s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;
I2C_SET_CONTROL_REG(I2C0, I2C_STA);	
	
while (g_u8EndFlag == 0);
#else
				I2C_START(MPU6050_I2C_PORT);                         //Start
	      I2C_WAIT_READY(MPU6050_I2C_PORT);
        MPU6050_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag	
	
	      I2C_SET_DATA(MPU6050_I2C_PORT, MPU6050_I2C_SLA);         //send slave address
	      I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(MPU6050_I2C_PORT);
        MPU6050_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		

	      I2C_SET_DATA(MPU6050_I2C_PORT, index);               //send index
	      I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(MPU6050_I2C_PORT);
        MPU6050_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		

	      I2C_SET_DATA(MPU6050_I2C_PORT, data);                //send Data
	      I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(MPU6050_I2C_PORT);
        MPU6050_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		
				
				I2C_STOP(MPU6050_I2C_PORT); //STOP
#endif
}

uint8_t MPU6050_I2C_SingleRead(uint8_t index)
{
	uint8_t tmp;
	
#ifdef I2C_IRQ
/* I2C function to read data from slave */
s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;

g_u8DataLen = 0;
g_u8EndFlag = 0;

g_u8DeviceAddr = MPU6050_I2C_SLA;
g_au8TxData[0] = index;//(uint8_t)();
	
I2C_SET_CONTROL_REG(I2C0, I2C_STA);

/* Wait I2C Rx Finish */
while (g_u8EndFlag == 0){;}
tmp=g_u8RxData;
#else
				I2C_START(MPU6050_I2C_PORT);                         //Start
	      I2C_WAIT_READY(MPU6050_I2C_PORT);
        MPU6050_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		
	
	      I2C_SET_DATA(MPU6050_I2C_PORT, MPU6050_I2C_SLA);         //send slave address+W
	      I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(MPU6050_I2C_PORT);
        MPU6050_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		

	      I2C_SET_DATA(MPU6050_I2C_PORT, index);               //send index
	      I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(MPU6050_I2C_PORT);
        MPU6050_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		
	
	      I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_STA | I2C_SI);	//Start
	      I2C_WAIT_READY(MPU6050_I2C_PORT);
        MPU6050_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag	

		    I2C_SET_DATA(MPU6050_I2C_PORT, (MPU6050_I2C_SLA+1));     //send slave address+R
	      I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(MPU6050_I2C_PORT);
        MPU6050_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag								
	
	      I2C_SET_CONTROL_REG(MPU6050_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(MPU6050_I2C_PORT);
        MPU6050_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag								
				tmp = I2C_GET_DATA(MPU6050_I2C_PORT);                //read data   
	
	      I2C_STOP(MPU6050_I2C_PORT); //STOP
#endif			
				return tmp;
}

void Init_MPU6050(void)
{
  I2C_SetSlaveAddr(MPU6050_I2C_PORT, 0, MPU6050_I2C_SLA, I2C_GCMODE_DISABLE);	
	MPU6050_I2C_SingleWrite(MPU6050_PWR_MGMT_1, 0x01);	// CLL_SEL=0: internal 8MHz, TEMP_DIS=0, SLEEP=0 
	MPU6050_I2C_SingleWrite(MPU6050_SMPLRT_DIV, 0x01);  // Gyro output sample rate = Gyro Output Rate/(1+SMPLRT_DIV)
	MPU6050_I2C_SingleWrite(MPU6050_CONFIG, 0x01);      // set TEMP_OUT_L, DLPF=2 (Fs=1KHz)
	MPU6050_I2C_SingleWrite(MPU6050_GYRO_CONFIG, 0x18); // bit[4:3] 0=+-250d/s,1=+-500d/s,2=+-1000d/s,3=+-2000d/s
	MPU6050_I2C_SingleWrite(MPU6050_ACCEL_CONFIG, 0x00);// bit[4:3] 0=+-2g,1=+-4g,2=+-8g,3=+-16g, ACC_HPF=On (5Hz)
}

uint16_t Read_MPU6050_AccX(void)
{
	uint8_t LoByte, HiByte;
	LoByte = MPU6050_I2C_SingleRead(MPU6050_ACCEL_XOUT_L); // read Accelerometer X_Low  value
	HiByte = MPU6050_I2C_SingleRead(MPU6050_ACCEL_XOUT_H); // read Accelerometer X_High value
	return((HiByte<<8) | LoByte);
}

uint16_t Read_MPU6050_AccY(void)
{
	uint8_t LoByte, HiByte;
	LoByte = MPU6050_I2C_SingleRead(MPU6050_ACCEL_YOUT_L); // read Accelerometer X_Low  value
	HiByte = MPU6050_I2C_SingleRead(MPU6050_ACCEL_YOUT_H); // read Accelerometer X_High value
	return ((HiByte<<8) | LoByte);
}

uint16_t Read_MPU6050_AccZ(void)
{
	uint8_t LoByte, HiByte;
	LoByte = MPU6050_I2C_SingleRead(MPU6050_ACCEL_ZOUT_L); // read Accelerometer X_Low  value
	HiByte = MPU6050_I2C_SingleRead(MPU6050_ACCEL_ZOUT_H); // read Accelerometer X_High value
	return ((HiByte<<8) | LoByte);
}

uint16_t Read_MPU6050_GyroX(void)
{
	uint8_t LoByte, HiByte;
	LoByte = MPU6050_I2C_SingleRead(MPU6050_GYRO_XOUT_L); // read Accelerometer X_Low  value
	HiByte = MPU6050_I2C_SingleRead(MPU6050_GYRO_XOUT_H); // read Accelerometer X_High value
	return ((HiByte<<8) | LoByte);
}

uint16_t Read_MPU6050_GyroY(void)
{
	uint8_t LoByte, HiByte;
	LoByte = MPU6050_I2C_SingleRead(MPU6050_GYRO_YOUT_L); // read Accelerometer X_Low  value
	HiByte = MPU6050_I2C_SingleRead(MPU6050_GYRO_YOUT_H); // read Accelerometer X_High value
	return ((HiByte<<8) | LoByte);
}

uint16_t Read_MPU6050_GyroZ(void)
{
	uint8_t LoByte, HiByte;
	LoByte = MPU6050_I2C_SingleRead(MPU6050_GYRO_ZOUT_L); // read Accelerometer X_Low  value
	HiByte = MPU6050_I2C_SingleRead(MPU6050_GYRO_ZOUT_H); // read Accelerometer X_High value
	return ((HiByte<<8) | LoByte);
}
