//
// HDC1000 Driver: Low Power, High Accuracy Digital Humidity Sensor with Temperature Sensor 
//
#include <stdio.h>
#include <stdint.h>
#include "isd9100.h"
#include "i2c.h"
#include "HDC1000.h"

#define I2C_ACK(i2c) ( (i2c)->CTL = ((i2c)->CTL & ~I2C_CTL_SI_Msk) | I2C_CTL_AA_Msk )


void HDC1000_I2C_SingleWrite(uint8_t index, uint8_t data)
{
        I2C_START(HDC1000_I2C_PORT);                         //Start
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag	
	
	      I2C_SET_DATA(HDC1000_I2C_PORT, HDC1000_I2C_SLA);         //send slave address
	      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		

	      I2C_SET_DATA(HDC1000_I2C_PORT, index);               //send index
	      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		

	      I2C_SET_DATA(HDC1000_I2C_PORT, data);                //send Data
	      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		

	      //I2C_SET_DATA(HDC1000_I2C_PORT, 0x00);                //send Data
	      //I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI);
	      //I2C_WAIT_READY(HDC1000_I2C_PORT);
        //HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		
	
				I2C_STOP(HDC1000_I2C_PORT); //STOP
}

uint16_t HDC1000_I2C_SingleRead(uint8_t index)
{
	uint8_t HiByte=0x00,LoByte=0x00;
	      I2C_START(HDC1000_I2C_PORT);                         //Start
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		
	
	      I2C_SET_DATA(HDC1000_I2C_PORT, HDC1000_I2C_SLA);         //send slave address+W
	      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		

	      I2C_SET_DATA(HDC1000_I2C_PORT, index);               //send index
	      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		
	
do{
	      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_STA | I2C_SI);	//Start
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag	
	
	
		    I2C_SET_DATA(HDC1000_I2C_PORT, (HDC1000_I2C_SLA+1));     //send slave address+R
	      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag
				if (HDC1000_I2C_PORT->STATUS == 0x58) HiByte = 0xff;
				if (HDC1000_I2C_PORT->STATUS == 0x30) HiByte = 0xff;
				if (HDC1000_I2C_PORT->STATUS == 0x48) HiByte = 0xff;

}while ( HiByte == 0xff);//(HDC1000_I2C_PORT->STATUS == 0x58) || (HDC1000_I2C_PORT->STATUS == 0x30)  );
					
	
	      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI | I2C_AA);
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag								
				HiByte = I2C_GET_DATA(HDC1000_I2C_PORT);                //read data
				
				I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI | I2C_AA);
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag								

				LoByte = I2C_GET_DATA(HDC1000_I2C_PORT);                //read data
				
	      I2C_STOP(HDC1000_I2C_PORT); //STOP
				return ((HiByte<<8) | LoByte);
				//return tmp;
}

uint16_t HDC1000_I2C_DoubleRead(uint8_t index)
{
	uint8_t HiByte=0x00,LoByte=0x00;
	uint16_t i = 0;
	      I2C_START(HDC1000_I2C_PORT);                         //Start
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		
	
	      I2C_SET_DATA(HDC1000_I2C_PORT, HDC1000_I2C_SLA);         //send slave address+W
	      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		

	      I2C_SET_DATA(HDC1000_I2C_PORT, index);               //send index
	      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		


		for(i=0;i<3000;i++)
		{
		}
	  	i = 0;
		
//	      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_STA | I2C_SI);	//Start
//	      I2C_WAIT_READY(HDC1000_I2C_PORT);
//	      HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag	
		I2C_START(HDC1000_I2C_PORT);                         //Start
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag		

	
	      I2C_SET_DATA(HDC1000_I2C_PORT, (HDC1000_I2C_SLA+1));     //send slave address+R
	      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI);
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
       	HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag
//				if (HDC1000_I2C_PORT->STATUS == 0x58) HiByte = 0xff;
//				if (HDC1000_I2C_PORT->STATUS == 0x30) HiByte = 0xff;
//				if (HDC1000_I2C_PORT->STATUS == 0x48) HiByte = 0xff;

//}while ( HiByte == 0xff);//(HDC1000_I2C_PORT->STATUS == 0x58) || (HDC1000_I2C_PORT->STATUS == 0x30)  );
					
	
	      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI | I2C_AA);
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
        HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag								
				HiByte = I2C_GET_DATA(HDC1000_I2C_PORT);                //read data
				
		I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI | I2C_AA);
	      I2C_WAIT_READY(HDC1000_I2C_PORT);
       HDC1000_I2C_PORT->CTL |= I2C_CTL_SI_Msk;   //clear flag								

				LoByte = I2C_GET_DATA(HDC1000_I2C_PORT);                //read data
			      I2C_SET_CONTROL_REG(HDC1000_I2C_PORT, I2C_SI|I2C_STO);//Stop
		      I2C_STOP(HDC1000_I2C_PORT); //STOP
				return ((HiByte<<8) | LoByte);
				//return tmp;
}



void Init_HDC1000(void)
{
  I2C_SetSlaveAddr(HDC1000_I2C_PORT, 0, HDC1000_I2C_SLA, I2C_GCMODE_DISABLE);	
	//Read_HDC1000_Device_ID();
	//Read_HDC1000_Manufacturer_ID();
	
	//HDC1000_I2C_SingleWrite(HDC1000_Configuration_Reg, ( HDC1000_RST ) >>8  );	// CLL_SEL=0: internal 8MHz, TEMP_DIS=0, SLEEP=0 
	HDC1000_I2C_SingleWrite(HDC1000_Configuration_Reg, ( HDC1000_MODE | HDC1000_HEAT) >>8  );	// CLL_SEL=0: internal 8MHz, TEMP_DIS=0, SLEEP=0 

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
//	Temperature_Reg = HDC1000_I2C_SingleRead(HDC1000_Temperature_Reg); // read ID of Texas Instruments
	Temperature_Reg = HDC1000_I2C_DoubleRead(HDC1000_Temperature_Reg); // read ID of Texas Instruments

return Temperature_Reg;
}

uint16_t Read_HDC1000_Humidity(void)
{
	uint16_t Humidity_Reg;
//	Humidity_Reg = HDC1000_I2C_SingleRead(HDC1000_Humidity_Reg); // read ID of Texas Instrument
	Humidity_Reg = HDC1000_I2C_DoubleRead(HDC1000_Humidity_Reg); // read ID of Texas Instruments

return Humidity_Reg;
}


