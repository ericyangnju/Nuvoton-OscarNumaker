//
// LY096BG30 : 0.96" OLED
// 
#define LCD_I2C_SLA_LCD_DC_LO       0x78
#define LCD_I2C_SLA_LCD_DC_HI       0x7A

#define LCD_I2C_SLA				LCD_I2C_SLA_LCD_DC_HI
#define LCD_I2C_PORT       I2C0

#define LCD_Xmax 128
#define LCD_Ymax 64

extern void Init_LCD(void);
extern void clear_LCD(void);
extern void print_LCD(unsigned char *buffer);
void print_Line(uint8_t Line, char Text[]);
