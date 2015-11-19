
//
// HDC1000 Driver: Low Power, High Accuracy Digital Humidity Sensor with Temperature Sensor 
// 
#include <stdint.h>
#define HDC1000_I2C_SLA_ADR0  	0x80	//PIN ADR[1:0] = 00
#define HDC1000_I2C_SLA_ADR1  	0x82	//PIN ADR[1:0] = 01
#define HDC1000_I2C_SLA_ADR2   	0x84	//PIN ADR[1:0] = 02
#define HDC1000_I2C_SLA_ADR3   	0x86	//PIN ADR[1:0] = 03

#define HDC1000_I2C_SLA					HDC1000_I2C_SLA_ADR0
#define HDC1000_I2C_PORT        I2C0
#define HDC1000_Device_ID				0x1000
#define HDC1000_Manufacturer_ID	0x5449

// The temperature and humidity can be calculated from the output data with
// Temperature = [(d15:d0)/2^16]*165c-40c
// Humidity = [(d15:d0)/2^16]*100%

// HDC1000 Internal Registers 
#define HDC1000_Temperature_Reg			0x00	// Temperature measurement output
#define HDC1000_Humidity_Reg				0x01	// Relative Humidity measurement output
#define HDC1000_Configuration_Reg		0x02	// HDC1000 configuration and status

#define HDC1000_Manufacturer_ID_Reg 	0xFE 	// ID of Texas Instruments
#define HDC1000_Device_ID_Reg 				0xFF 	// ID of HDC1000 device

//////////////////////////////////////////////////////////////
#define HDC1000_RST			BIT15 	// 0 Normal Operation, 1 Software Reset
#define HDC1000_HEAT    BIT13 	// 0 Heater Disable, 1 Heater Enable
#define HDC1000_MODE    BIT12 	// 0 Temperature or Humidity is acquired. 
																// 1 Temperature and Humidity are acquired in sequence, Temperature first.
															
#define HDC1000_TRES   	BIT10   // Temperature Measurement Resolution, 0=14 bit, 1=11 bit
#define HDC1000_HRES    BIT9    // Humidity Measurement Resolution, 00=14 bit, 01=11 bit, 10=8 bit
extern void Init_HDC1000(void);
extern uint16_t Read_HDC1000_Device_ID(void);
extern uint16_t Read_HDC1000_Manufacturer_ID(void);
extern uint16_t Read_HDC1000_Configuration(void);
extern uint16_t Read_HDC1000_Temperature(void);
extern uint16_t Read_HDC1000_Humidity(void);
