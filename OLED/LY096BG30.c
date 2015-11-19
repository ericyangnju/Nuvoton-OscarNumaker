//
// LCD Driver: 0.96" OLED
//
// Interface: I2C
// pin1: Gnd
// pin2: Vcc
// pin3: SCL
// pin4: SDA
// pin5: OUT
// pin6: IN
// pin7: SCK
// pin8: CS

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "isd9100.h"
#include "sys.h"
#include "gpio.h"
#include "i2c.h"
//#include "LCD.h"
#include "LY096BG30.h"
#include "Font8x16.h"
#include "../I2c0int.h"  //ericyang 20151110 test

void OLED_SingleWrite(uint8_t index, uint8_t data)
{
#ifdef I2C_IRQ
	g_u8DeviceAddr = LCD_I2C_SLA;
		
	g_au8TxData[0] = index;//(uint8_t)();
	g_au8TxData[1] = data;//(uint8_t)();

	g_u8DataLen = 0;
	g_u8EndFlag = 0;	
#ifdef I2C_TIMEOUT_ENABLE
	i2c_novalidack_timeout = 0;//ericyang 20151111
	i2c_unreachable_timeout = 0;
#endif
	s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;
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

uint8_t OLED_SingleRead(uint8_t index)
{
	uint8_t tmp;

#ifdef I2C_IRQ
	/* I2C function to read data from slave */
	s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;

	g_u8DataLen = 0;
	g_u8EndFlag = 0;
#ifdef I2C_TIMEOUT_ENABLE
	i2c_novalidack_timeout = 0;//ericyang 20151111
	i2c_unreachable_timeout = 0;
#endif
	g_u8DeviceAddr = LCD_I2C_SLA;
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
	tmp=g_u8RxData;
#endif
	return tmp;
}

void oledWriteCommand(unsigned char OLED_Command)
{
   OLED_SingleWrite(0x00, OLED_Command);
}

void oledWriteData(unsigned char OLED_Data)
{
   OLED_SingleWrite(0x40, OLED_Data);
}

void Init_LCD(void)
{
	oledWriteCommand(0xae); //display off
	oledWriteCommand(0x20);	//Set Memory Addressing Mode	
	oledWriteCommand(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	oledWriteCommand(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	oledWriteCommand(0xc8);	//Set COM Output Scan Direction
	oledWriteCommand(0x00);//---set low column address
	oledWriteCommand(0x10);//---set high column address
	oledWriteCommand(0x40);//--set start line address
	oledWriteCommand(0x81);//--set contrast control register
	oledWriteCommand(0x7f);
	oledWriteCommand(0xa1);//--set segment re-map 0 to 127
	oledWriteCommand(0xa6);//--set normal display
	oledWriteCommand(0xa8);//--set multiplex ratio(1 to 64)
	oledWriteCommand(0x3F);//
	oledWriteCommand(0xa4);//0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	oledWriteCommand(0xd3);//-set display offset
	oledWriteCommand(0x00);//-not offset
	oledWriteCommand(0xd5);//--set display clock divide ratio/oscillator frequency
	oledWriteCommand(0xf0);//--set divide ratio
	oledWriteCommand(0xd9);//--set pre-charge period
	oledWriteCommand(0x22); //
	oledWriteCommand(0xda);//--set com pins hardware configuration
	oledWriteCommand(0x12);
	oledWriteCommand(0xdb);//--set vcomh
	oledWriteCommand(0x20);//0x20,0.77xVcc
	oledWriteCommand(0x8d);//--set DC-DC enable
	oledWriteCommand(0x14);//
	oledWriteCommand(0xaf);//--turn on oled panel 
}

void oled_address(uint8_t column, uint8_t page)
{
	oledWriteCommand(0xb0+page);                     // set page address
	oledWriteCommand(0x10 | ((column & 0xf0) >> 4)); // set column address MSB
	oledWriteCommand(0x00 |  (column & 0x0f)      ); // set column address LSB
}

void clear_LCD(void)
{
	int16_t x, Y;
	for (Y=0;Y<LCD_Ymax/8;Y++) 
	{
		oled_address(0, Y);
	  for (x=0;x<LCD_Xmax;x++)
	     oledWriteData(0x00);
	}
}

void draw_LCD(unsigned char *buffer)
{
	int16_t x, Y;
	for (Y=0;Y<8;Y++)
	{
	  oled_address(0, Y);
		for (x=0;x<LCD_Xmax;x++)
	     oledWriteData(buffer[x+Y*LCD_Xmax]);
	}
}

void print_C(uint8_t Col, uint8_t Line, char ascii)
{
	uint8_t j, i, tmp;	
	for (j=0;j<2;j++) {		
	    oled_address(Col*8, Line*2+j);
	    for (i=0;i<8;i++) {
	      tmp=Font8x16[(ascii-0x20)*16+j*8+i];
	      oledWriteData(tmp);
		  }
  }
}

void print_Line(uint8_t Line, char Text[])
{
	uint8_t Col;
	  for (Col=0; Col<strlen(Text); Col++) 
			print_C(Col, Line, Text[Col]);
}
