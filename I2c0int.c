#include <stdio.h>
#include <stdint.h>
#include "isd9100.h"
//#include "sys.h"
//#include "gpio.h"
#include "i2c.h"
#include "I2c0int.h"

#ifdef I2C_IRQ
uint8_t g_u8DeviceAddr;
uint8_t g_au8TxData[3];   //  2  
volatile uint8_t g_u8RxData;
volatile uint8_t g_u8RxData1;
#ifdef I2C_TIMEOUT_ENABLE
volatile uint16_t i2c_novalidack_timeout;
volatile uint16_t i2c_unreachable_timeout;
#endif
volatile uint8_t g_u8DataLen;
volatile uint8_t g_u8EndFlag = 0;


volatile I2C_FUNC s_I2C0HandlerFn = NULL;



void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);
#if 0 //def DEBUG_ENABLE
    printf("Status 0x%x \n", u32Status);
#endif
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
        if (g_u8DataLen != 1){   // 2)  {
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
	#ifdef I2C_TIMEOUT_ENABLE
		if(i2c_novalidack_timeout++>I2C_TIMEOUT_NOVALIDACK_COUNT)
		{
			I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
			g_u8EndFlag = 1;
		}
	#endif

       // printf("Status 0x%x is NOT processed\n", u32Status);
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
        if (g_u8DataLen != 2) {      // 3) {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_SI);
        } else {
            I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
            g_u8EndFlag = 1;
        }
    } else {
        /* TO DO */
	#ifdef I2C_TIMEOUT_ENABLE
		if(i2c_novalidack_timeout++>I2C_TIMEOUT_NOVALIDACK_COUNT)
		{
			I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
			g_u8EndFlag = 1;
		}
	#endif

       // printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

void I2C_MasterRx_var(uint32_t u32Status)
{
	uint16_t i=0;
	if (u32Status == 0x08) 
	{                    /* START has been transmitted and prepare SLA+W */
		I2C_SET_DATA(I2C0, g_u8DeviceAddr);//(g_u8DeviceAddr << 1)); /* Write SLA+W to Register I2CDAT */
		I2C_SET_CONTROL_REG(I2C0, I2C_SI);
	} 
	else if (u32Status == 0x18 )
	{             /* SLA+W has been transmitted and ACK has been received */
		I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
		I2C_SET_CONTROL_REG(I2C0, I2C_SI);
	} 
	else if (u32Status == 0x20) 
	{             /* SLA+W has been transmitted and NACK has been received */
		I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_STO | I2C_SI);
	} 
	else if (u32Status == 0x28) 
	{             /* DATA has been transmitted and ACK has been received */
	#if 0//def DEBUG_ENABLE
		printf("Status rx 0x%x\n", u32Status);
	#endif
		for(i=0;i<65535;i++);//very important
	//	for(i=0;i<50000;i++);
	//	for(i=0;i<50000;i++);

		if (g_u8DataLen != 1)   // 2
		{   
			I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
			I2C_SET_CONTROL_REG(I2C0, I2C_SI);
		}
		else 
		{
			I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_SI);
		}
	} 
	else if (u32Status == 0x10) 
	{             /* Repeat START has been transmitted and prepare SLA+R */
		I2C_SET_DATA(I2C0, g_u8DeviceAddr  | 0x01);//(g_u8DeviceAddr << 1) | 0x01);  /* Write SLA+R to Register I2CDAT */
		I2C_SET_CONTROL_REG(I2C0, I2C_SI);
	} 
	else if (u32Status == 0x40) 
	{             /* SLA+R has been transmitted and ACK has been received */
	#if 0//def DEBUG_ENABLE
		printf("Status rx 0x%x\n", u32Status);
	#endif

		I2C_SET_CONTROL_REG(I2C0, I2C_SI|I2C_AA);
	} 
	#if 1//ericyang add for HDC1000
	else if (u32Status == 0x50) 
	{             /* DATA has been received and ACK has been returned */
		g_u8RxData = I2C_GET_DATA(I2C0);
		//I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
		I2C_SET_CONTROL_REG(I2C0,   I2C_SI);
		//g_u8EndFlag = 1;
	}
	#endif
	else if (u32Status == 0x58) 
	{             /* DATA has been received and NACK has been returned */		
			g_u8RxData1 = I2C_GET_DATA(I2C0);
			I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
			g_u8EndFlag = 1;
	#ifdef DEBUG_ENABLE
		printf("g_u8RxData 0x%x  g_u8RxData1:0x%x\n", g_u8RxData,g_u8RxData1);
	#endif		
	} 
	else 
	{
	/* TO DO */
	#ifdef I2C_TIMEOUT_ENABLE
		if(i2c_novalidack_timeout++>I2C_TIMEOUT_NOVALIDACK_COUNT)
		{
			I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
			g_u8EndFlag = 1;
		}
	#endif
	#ifdef DEBUG_ENABLE
		printf("Status rxvar 0x%x is NOT processed\n", u32Status);
	#endif
	}
}
void I2C_MasterTx_var(uint32_t u32Status)
{
	if (u32Status == 0x08) 
	{                    /* START has been transmitted */
		I2C_SET_DATA(I2C0, g_u8DeviceAddr);//g_u8DeviceAddr << 1);  /* Write SLA+W to Register I2CDAT */
		I2C_SET_CONTROL_REG(I2C0, I2C_SI);
	} 
	else if (u32Status == 0x18) 
	{             /* SLA+W has been transmitted and ACK has been received */
		I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
		I2C_SET_CONTROL_REG(I2C0, I2C_SI);
	} 
	else if (u32Status == 0x20) 
	{             /* SLA+W has been transmitted and NACK has been received */
		I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_STO | I2C_SI);
	} 
	else if (u32Status == 0x28)
	{             /* DATA has been transmitted and ACK has been received */
		if (g_u8DataLen != 3)  // 2
		{     
			I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
			I2C_SET_CONTROL_REG(I2C0, I2C_SI);
		} 
		else 
		{
			I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
			g_u8EndFlag = 1;
		}
	} 
	else 
	{
	/* TO DO */
	#ifdef I2C_TIMEOUT_ENABLE
		if(i2c_novalidack_timeout++>I2C_TIMEOUT_NOVALIDACK_COUNT)
		{
			I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
			g_u8EndFlag = 1;
		}
	#endif

	#ifdef DEBUG_ENABLE
		printf("Status txvar 0x%x is NOT processed\n", u32Status);
	#endif
	}

}


#endif


