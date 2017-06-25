/// noon funziona
/*
Basic Grafix.ino
 *
 * Created: 9/20/2014 5:09:28 PM
 * Author: BasisV12
 */ 

//Bus Pins (Arduino digital Pin numbers)
#define LCD_D0_PIN 8
#define LCD_D1_PIN 9
#define LCD_D2_PIN 2
#define LCD_D3_PIN 3
#define LCD_D4_PIN 4
#define LCD_D5_PIN 5
#define LCD_D6_PIN 6
#define LCD_D7_PIN 7

//Control Pins (here: Analog pins A0..A4)
#define LCD_RST    A4 //PA6
#define LCD_CS     A3 //PA22
#define LCD_RS     A2 //PA23
#define LCD_WR     A1 //PA24
#define LCD_RD     A0 //PA16

#define BIT0_MSK_8BIT  B00000001
#define BIT1_MSK_8BIT  B00000010
#define BIT2_MSK_8BIT  B00000100
#define BIT3_MSK_8BIT  B00001000
#define BIT4_MSK_8BIT  B00010000
#define BIT5_MSK_8BIT  B00100000
#define BIT6_MSK_8BIT  B01000000
#define BIT7_MSK_8BIT  B10000000

//#define LCD_RST_LOW()  digitalWrite(LCD_RST, LOW) 
//#define LCD_RST_HIGH() digitalWrite(LCD_RST, HIGH)
//
//#define LCD_CS_LOW()  digitalWrite(LCD_CS, LOW)
//#define LCD_CS_HIGH() digitalWrite(LCD_CS, HIGH)
//
//#define LCD_RS_LOW()  digitalWrite(LCD_RS, LOW)
//#define LCD_RS_HIGH() digitalWrite(LCD_RS, HIGH)
//
//#define LCD_WR_LOW()  digitalWrite(LCD_WR, LOW)
//#define LCD_WR_HIGH() digitalWrite(LCD_WR, HIGH)
//
//#define LCD_RD_LOW()  digitalWrite(LCD_RD, LOW)
//#define LCD_RD_HIGH() digitalWrite(LCD_RD, HIGH)

#define LCD_RST_LOW()  REG_PIOA_CODR=(1<<6)
#define LCD_RST_HIGH() REG_PIOA_SODR=(1<<6)

#define LCD_CS_LOW()  REG_PIOA_CODR=(1<<22)
#define LCD_CS_HIGH() REG_PIOA_SODR=(1<<22)

#define LCD_RS_LOW()  REG_PIOA_CODR=(1<<23)
#define LCD_RS_HIGH() REG_PIOA_SODR=(1<<23)

#define LCD_WR_LOW()  REG_PIOA_CODR=(1<<24)
#define LCD_WR_HIGH() REG_PIOA_SODR=(1<<24)

#define LCD_RD_LOW()  REG_PIOA_CODR=(1<<16)
#define LCD_RD_HIGH() REG_PIOA_SODR=(1<<16)
//Basic Colors
#define	LCD_BLACK                   0x0000
#define	LCD_BLUE                    0x001F
#define	LCD_RED                     0xF800
#define	LCD_GREEN                   0x07E0
#define LCD_CYAN                    0x07FF
#define LCD_MAGENTA                 0xF81F
#define LCD_YELLOW                  0xFFE0
#define LCD_WHITE                   0xFFFF
	
	
//Other Colors
#define LCD_CYAN            0x07ff
#define LCD_BRIGHT_RED      0xf810
#define LCD_GRAY1           0x8410
#define LCD_GRAY2           0x4208

//TFT resolution 240*320
#define LCD_MIN_X       0
#define LCD_MIN_Y       0
#define LCD_MAX_X       240
#define LCD_MAX_Y       320


#define swap_uint16_t(a, b) { uint16_t t = a; a = b; b = t; }

#if 1 //Basics


void LCD_BusPinsAsOutput()
{
	 //for all arduino compatible
	 //pinMode(LCD_D0_PIN,OUTPUT);
	 //pinMode(LCD_D1_PIN,OUTPUT);
	 //pinMode(LCD_D2_PIN,OUTPUT);
	 //pinMode(LCD_D3_PIN,OUTPUT);
	 //pinMode(LCD_D4_PIN,OUTPUT);
	 //pinMode(LCD_D5_PIN,OUTPUT);
	 //pinMode(LCD_D6_PIN,OUTPUT);
	 //pinMode(LCD_D7_PIN,OUTPUT);
	 
	 //ATSAM3 specific code, much faster than arduino
	REG_PIOC_OER= 0x17e00000;//0b00010111 11100000 00000000 00000000
	REG_PIOB_OER= 0x02000000;//0b00000010 00000000 00000000 00000000
 
}

void LCD_BusPinsAsInput()
{
	//compatible to all arduino
	//pinMode(LCD_D0_PIN,INPUT);
	//pinMode(LCD_D1_PIN,INPUT);
	//pinMode(LCD_D2_PIN,INPUT);
	//pinMode(LCD_D3_PIN,INPUT);
	//pinMode(LCD_D4_PIN,INPUT);
	//pinMode(LCD_D5_PIN,INPUT);
	//pinMode(LCD_D6_PIN,INPUT);
	//pinMode(LCD_D7_PIN,INPUT);
	
	//ATSAM specific code, much faster than arduino
	REG_PIOC_ODR= 0x17e00000;//0b00010111 11100000 00000000 00000000
	REG_PIOB_ODR= 0x02000000;//0b00000010 00000000 00000000 00000000	
}

static inline void  LCD_BusData8(uint8_t byteToWrite)
{
	//Compatible with all arduino
	// digitalWrite(LCD_D0_PIN,byteToWrite & BIT0_MSK_8BIT);
	// digitalWrite(LCD_D1_PIN,byteToWrite & BIT1_MSK_8BIT);
	// digitalWrite(LCD_D2_PIN,byteToWrite & BIT2_MSK_8BIT);
	// digitalWrite(LCD_D3_PIN,byteToWrite & BIT3_MSK_8BIT);
	// digitalWrite(LCD_D4_PIN,byteToWrite & BIT4_MSK_8BIT);
	// digitalWrite(LCD_D5_PIN,byteToWrite & BIT5_MSK_8BIT);
	// digitalWrite(LCD_D6_PIN,byteToWrite & BIT6_MSK_8BIT);
	// digitalWrite(LCD_D7_PIN,byteToWrite & BIT7_MSK_8BIT);
	
	//ATSAM3 specific code, much faster than arduino!
	REG_PIOC_CODR= 0x17e00000;//0b00010111 11100000 00000000 00000000
	REG_PIOB_CODR= 0x02000000;//0b00000010 00000000 00000000 00000000
	REG_PIOC_SODR= (byteToWrite & 0x01)<<22 //D0
	|(byteToWrite & 0x02)<<20 //D1
	|(byteToWrite & 0x08)<<25 //D3
	|(byteToWrite & 0x10)<<22 //D4
	|(byteToWrite & 0x20)<<20 //D5
	|(byteToWrite & 0x40)<<18 //D6
	|(byteToWrite & 0x80)<<16; //D7
	REG_PIOB_SODR= (byteToWrite & 0x04)<<23;
}

uint8_t LCD_BusWrite(byte byteToWrite)
{
	LCD_BusPinsAsOutput();
	LCD_BusData8(byteToWrite);
			
	return 0;
}

uint8_t LCD_BusRead()
{
	
        uint8_t data=0;
	
	LCD_BusPinsAsInput();
	delayMicroseconds(100);
	
        data = digitalRead(LCD_D0_PIN)<<0;
	data |= digitalRead(LCD_D1_PIN)<<1;
	data |= digitalRead(LCD_D2_PIN)<<2;
	data |= digitalRead(LCD_D3_PIN)<<3;
	data |= digitalRead(LCD_D4_PIN)<<4;
	data |= digitalRead(LCD_D5_PIN)<<5;
	data |= digitalRead(LCD_D6_PIN)<<6;
	data |= digitalRead(LCD_D7_PIN)<<7;
	
	//data = REG_PIOC_PDSR & 0x17e00000;   //0b00010111 11100000 00000000 00000000
	//data |= REG_PIOB_PDSR & 0x02000000;  //0b00000010 00000000 00000000 00000000
	return data;
}

void LCD_WriteData_unsafe(uint16_t data)
{
	LCD_BusWrite(data>>8);
	LCD_WR_LOW();
	LCD_WR_HIGH();
	LCD_BusWrite(data);
	LCD_WR_LOW();
	LCD_WR_HIGH();
}

void LCD_WriteData(uint16_t data)
{
	
	LCD_CS_LOW();
	LCD_RS_HIGH();
	//LCD_RD_HIGH();
	//LCD_WR_HIGH();
	LCD_BusPinsAsOutput();
	LCD_WriteData_unsafe(data);
	LCD_CS_HIGH();
}

uint8_t LCD_WriteCommand(uint16_t  cmd_index)
{
	LCD_CS_LOW();
	LCD_RS_LOW();
	//LCD_RD_HIGH();
	//LCD_WR_HIGH();
	
	LCD_BusPinsAsOutput();
	LCD_BusWrite(cmd_index>>8);
	LCD_WR_LOW();
	LCD_WR_HIGH();
	LCD_BusWrite(cmd_index);
	LCD_WR_LOW();
	LCD_WR_HIGH();
	LCD_CS_HIGH();
	
	return 0;	
}

uint16_t LCD_ReadData()
{
	uint16_t data=0;
	LCD_CS_LOW();
	LCD_RS_HIGH();
	//LCD_RD_HIGH();
	//LCD_WR_HIGH();
	LCD_BusPinsAsInput();
	LCD_RD_LOW();
	delayMicroseconds(100);
	data=LCD_BusRead();
	data <<=8;
	LCD_RD_HIGH();
        LCD_RD_LOW();
	delayMicroseconds(100);
	data |= LCD_BusRead();
	LCD_RD_HIGH();
	LCD_CS_HIGH();
	return data;
	
}

uint16_t LCD_ReadRegister(uint16_t addr)
{
	LCD_WriteCommand(addr);
	return LCD_ReadData();		
}

uint16_t LCD_WriteRegister(uint16_t addr, uint16_t data)
{
	LCD_WriteCommand(addr);
	LCD_WriteData(data);
}

void LCD_Init()
{
	LCD_BusPinsAsOutput();
	LCD_BusWrite(0);
	//Reset Pin
	pinMode(LCD_RST,OUTPUT);
	digitalWrite(LCD_RST, HIGH);
	//Chip Select
	pinMode(LCD_CS,OUTPUT);
	digitalWrite(LCD_CS, HIGH);
	//Register Select
	pinMode(LCD_RS,OUTPUT);
	digitalWrite(LCD_RS, HIGH);
	//Write strobe
	pinMode(LCD_WR,OUTPUT);
	digitalWrite(LCD_WR, HIGH);
	//Read strobe
	pinMode(LCD_RD,OUTPUT);
	digitalWrite(LCD_RD, HIGH);
	
	// -Reset--------------------------------------------------------------------------------------

	LCD_RST_LOW();
	delay(100);
	LCD_RST_HIGH();
	delay(100); //1000
	/*LCD_WriteData(0);   // resync
	LCD_WriteData(0);
	LCD_WriteData(0);
	LCD_WriteData(0);*/
	
	delayMicroseconds (200);
        
        LCD_WriteCommand(0x00FF);
	LCD_WriteData(0x0001);
        LCD_WriteCommand(0x00F3);
	LCD_WriteData(0x0008); 
        LCD_WriteCommand(0xF1);
	LCD_WriteData(0x00F3);

#if 1		
	LCD_WriteCommand(0x0001);
	LCD_WriteData(0x0100);
	LCD_WriteCommand(0x0002);
	LCD_WriteData(0x0700);
	LCD_WriteCommand(0x0003);
	LCD_WriteData(0x1030);
	LCD_WriteCommand(0x0008);
	LCD_WriteData(0x0302);
	LCD_WriteCommand(0x0009);
	LCD_WriteData(0x0000);
	LCD_WriteCommand(0x0010);
	LCD_WriteData(0x0000);
	LCD_WriteCommand(0x0011);
	LCD_WriteData(0x0007);
	LCD_WriteCommand(0x0012);
	LCD_WriteData(0x0000);
	LCD_WriteCommand(0x0013);
	LCD_WriteData(0x0000);
	delayMicroseconds(1000); //1000
	LCD_WriteCommand(0x0010);
	LCD_WriteData(0x14B0);
	delayMicroseconds(500);
	LCD_WriteCommand(0x0011);
	LCD_WriteData(0x0007);
	delayMicroseconds(500);	
	LCD_WriteCommand(0x0012);
	LCD_WriteData(0x008E);
	LCD_WriteCommand(0x0013);
	LCD_WriteData(0x0C00);
	LCD_WriteCommand(0x0029);
	LCD_WriteData(0x0015);
	delayMicroseconds(500);	
	LCD_WriteCommand(0x0030);
	LCD_WriteData(0x0000);
	LCD_WriteCommand(0x0031);
	LCD_WriteData(0x0107);
	LCD_WriteCommand(0x0032);
	LCD_WriteData(0x0000);
	LCD_WriteCommand(0x0035);
	LCD_WriteData(0x0203);
	LCD_WriteCommand(0x0036);
	LCD_WriteData(0x0402);
	LCD_WriteCommand(0x0037);
	LCD_WriteData(0x0000);
	LCD_WriteCommand(0x0038);
	LCD_WriteData(0x0207);
	LCD_WriteCommand(0x0039);
	LCD_WriteData(0x0000);
	LCD_WriteCommand(0x003c);
	LCD_WriteData(0x0203);
	LCD_WriteCommand(0x003d);
	LCD_WriteData(0x0403);
	LCD_WriteCommand(0x0050);
	LCD_WriteData(0x0000);
	LCD_WriteCommand(0x0051);
	LCD_WriteData(0x00EF);
	LCD_WriteCommand(0x0052);
	LCD_WriteData(0x0000);
	LCD_WriteCommand(0x0053);
	LCD_WriteData(0x013F);
	LCD_WriteCommand(0x0060);
	LCD_WriteData(0xA700);
	LCD_WriteCommand(0x0061);
	LCD_WriteData(0x0001);
	LCD_WriteCommand(0x0090);
	LCD_WriteData(0x0029);

	LCD_WriteCommand(0x0007);
	LCD_WriteData(0x0133);
	delayMicroseconds(500);
	
	LCD_exitStandBy();
	LCD_WriteCommand(0x0022);
#endif	
	//paint screen black
	LCD_PaintScreen(LCD_BLACK);
	
}

#endif // Basics

#if 1 //Drawing functions COOCOX Driver

	#if 1 //Font
		const unsigned char simpleFont[][8] =
		{
			{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
			{0x00,0x00,0x5F,0x00,0x00,0x00,0x00,0x00},
			{0x00,0x00,0x07,0x00,0x07,0x00,0x00,0x00},
			{0x00,0x14,0x7F,0x14,0x7F,0x14,0x00,0x00},
			{0x00,0x24,0x2A,0x7F,0x2A,0x12,0x00,0x00},
			{0x00,0x23,0x13,0x08,0x64,0x62,0x00,0x00},
			{0x00,0x36,0x49,0x55,0x22,0x50,0x00,0x00},
			{0x00,0x00,0x05,0x03,0x00,0x00,0x00,0x00},
			{0x00,0x1C,0x22,0x41,0x00,0x00,0x00,0x00},
			{0x00,0x41,0x22,0x1C,0x00,0x00,0x00,0x00},
			{0x00,0x08,0x2A,0x1C,0x2A,0x08,0x00,0x00},
			{0x00,0x08,0x08,0x3E,0x08,0x08,0x00,0x00},
			{0x00,0xA0,0x60,0x00,0x00,0x00,0x00,0x00},
			{0x00,0x08,0x08,0x08,0x08,0x08,0x00,0x00},
			{0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00},
			{0x00,0x20,0x10,0x08,0x04,0x02,0x00,0x00},
			{0x00,0x3E,0x51,0x49,0x45,0x3E,0x00,0x00},
			{0x00,0x00,0x42,0x7F,0x40,0x00,0x00,0x00},
			{0x00,0x62,0x51,0x49,0x49,0x46,0x00,0x00},
			{0x00,0x22,0x41,0x49,0x49,0x36,0x00,0x00},
			{0x00,0x18,0x14,0x12,0x7F,0x10,0x00,0x00},
			{0x00,0x27,0x45,0x45,0x45,0x39,0x00,0x00},
			{0x00,0x3C,0x4A,0x49,0x49,0x30,0x00,0x00},
			{0x00,0x01,0x71,0x09,0x05,0x03,0x00,0x00},
			{0x00,0x36,0x49,0x49,0x49,0x36,0x00,0x00},
			{0x00,0x06,0x49,0x49,0x29,0x1E,0x00,0x00},
			{0x00,0x00,0x36,0x36,0x00,0x00,0x00,0x00},
			{0x00,0x00,0xAC,0x6C,0x00,0x00,0x00,0x00},
			{0x00,0x08,0x14,0x22,0x41,0x00,0x00,0x00},
			{0x00,0x14,0x14,0x14,0x14,0x14,0x00,0x00},
			{0x00,0x41,0x22,0x14,0x08,0x00,0x00,0x00},
			{0x00,0x02,0x01,0x51,0x09,0x06,0x00,0x00},
			{0x00,0x32,0x49,0x79,0x41,0x3E,0x00,0x00},
			{0x00,0x7E,0x09,0x09,0x09,0x7E,0x00,0x00},
			{0x00,0x7F,0x49,0x49,0x49,0x36,0x00,0x00},
			{0x00,0x3E,0x41,0x41,0x41,0x22,0x00,0x00},
			{0x00,0x7F,0x41,0x41,0x22,0x1C,0x00,0x00},
			{0x00,0x7F,0x49,0x49,0x49,0x41,0x00,0x00},
			{0x00,0x7F,0x09,0x09,0x09,0x01,0x00,0x00},
			{0x00,0x3E,0x41,0x41,0x51,0x72,0x00,0x00},
			{0x00,0x7F,0x08,0x08,0x08,0x7F,0x00,0x00},
			{0x00,0x41,0x7F,0x41,0x00,0x00,0x00,0x00},
			{0x00,0x20,0x40,0x41,0x3F,0x01,0x00,0x00},
			{0x00,0x7F,0x08,0x14,0x22,0x41,0x00,0x00},
			{0x00,0x7F,0x40,0x40,0x40,0x40,0x00,0x00},
			{0x00,0x7F,0x02,0x0C,0x02,0x7F,0x00,0x00},
			{0x00,0x7F,0x04,0x08,0x10,0x7F,0x00,0x00},
			{0x00,0x3E,0x41,0x41,0x41,0x3E,0x00,0x00},
			{0x00,0x7F,0x09,0x09,0x09,0x06,0x00,0x00},
			{0x00,0x3E,0x41,0x51,0x21,0x5E,0x00,0x00},
			{0x00,0x7F,0x09,0x19,0x29,0x46,0x00,0x00},
			{0x00,0x26,0x49,0x49,0x49,0x32,0x00,0x00},
			{0x00,0x01,0x01,0x7F,0x01,0x01,0x00,0x00},
			{0x00,0x3F,0x40,0x40,0x40,0x3F,0x00,0x00},
			{0x00,0x1F,0x20,0x40,0x20,0x1F,0x00,0x00},
			{0x00,0x3F,0x40,0x38,0x40,0x3F,0x00,0x00},
			{0x00,0x63,0x14,0x08,0x14,0x63,0x00,0x00},
			{0x00,0x03,0x04,0x78,0x04,0x03,0x00,0x00},
			{0x00,0x61,0x51,0x49,0x45,0x43,0x00,0x00},
			{0x00,0x7F,0x41,0x41,0x00,0x00,0x00,0x00},
			{0x00,0x02,0x04,0x08,0x10,0x20,0x00,0x00},
			{0x00,0x41,0x41,0x7F,0x00,0x00,0x00,0x00},
			{0x00,0x04,0x02,0x01,0x02,0x04,0x00,0x00},
			{0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00},
			{0x00,0x01,0x02,0x04,0x00,0x00,0x00,0x00},
			{0x00,0x20,0x54,0x54,0x54,0x78,0x00,0x00},
			{0x00,0x7F,0x48,0x44,0x44,0x38,0x00,0x00},
			{0x00,0x38,0x44,0x44,0x28,0x00,0x00,0x00},
			{0x00,0x38,0x44,0x44,0x48,0x7F,0x00,0x00},
			{0x00,0x38,0x54,0x54,0x54,0x18,0x00,0x00},
			{0x00,0x08,0x7E,0x09,0x02,0x00,0x00,0x00},
			{0x00,0x18,0xA4,0xA4,0xA4,0x7C,0x00,0x00},
			{0x00,0x7F,0x08,0x04,0x04,0x78,0x00,0x00},
			{0x00,0x00,0x7D,0x00,0x00,0x00,0x00,0x00},
			{0x00,0x80,0x84,0x7D,0x00,0x00,0x00,0x00},
			{0x00,0x7F,0x10,0x28,0x44,0x00,0x00,0x00},
			{0x00,0x41,0x7F,0x40,0x00,0x00,0x00,0x00},
			{0x00,0x7C,0x04,0x18,0x04,0x78,0x00,0x00},
			{0x00,0x7C,0x08,0x04,0x7C,0x00,0x00,0x00},
			{0x00,0x38,0x44,0x44,0x38,0x00,0x00,0x00},
			{0x00,0xFC,0x24,0x24,0x18,0x00,0x00,0x00},
			{0x00,0x18,0x24,0x24,0xFC,0x00,0x00,0x00},
			{0x00,0x00,0x7C,0x08,0x04,0x00,0x00,0x00},
			{0x00,0x48,0x54,0x54,0x24,0x00,0x00,0x00},
			{0x00,0x04,0x7F,0x44,0x00,0x00,0x00,0x00},
			{0x00,0x3C,0x40,0x40,0x7C,0x00,0x00,0x00},
			{0x00,0x1C,0x20,0x40,0x20,0x1C,0x00,0x00},
			{0x00,0x3C,0x40,0x30,0x40,0x3C,0x00,0x00},
			{0x00,0x44,0x28,0x10,0x28,0x44,0x00,0x00},
			{0x00,0x1C,0xA0,0xA0,0x7C,0x00,0x00,0x00},
			{0x00,0x44,0x64,0x54,0x4C,0x44,0x00,0x00},
			{0x00,0x08,0x36,0x41,0x00,0x00,0x00,0x00},
			{0x00,0x00,0x7F,0x00,0x00,0x00,0x00,0x00},
			{0x00,0x41,0x36,0x08,0x00,0x00,0x00,0x00},
			{0x00,0x02,0x01,0x01,0x02,0x01,0x00,0x00},
			{0x00,0x02,0x05,0x05,0x02,0x00,0x00,0x00}
		};		
	#endif

// Macro definitions for char display direction
#define LEFT2RIGHT  0
#define DOWN2UP     1
#define RIGHT2LEFT  2
#define UP2DOWN     3

static unsigned char DisplayDirect = LEFT2RIGHT;

void LCD_SetXY(uint16_t poX, uint16_t poY)
{
	//Better Compatible to the code below
	//LCD_WriteCommand(0x0020);//X
	//LCD_WriteData(poX);
	//LCD_WriteCommand(0x0021);//Y
	//LCD_WriteData(poY);
	//LCD_WriteCommand(0x0022);//Start to write to display RAM
	
	//Same code as above, but faster
	LCD_BusPinsAsOutput();
	LCD_CS_LOW();
	
	//X
	//Write command 
	LCD_RS_LOW();
	LCD_BusData8(0x0020>>8);
	LCD_WR_LOW();
	LCD_WR_HIGH();
	LCD_BusData8(0x0020);
	LCD_WR_LOW();
	LCD_WR_HIGH();
	//Write Data
	LCD_RS_HIGH();
	LCD_BusData8(poX>>8);
	LCD_WR_LOW();
	LCD_WR_HIGH();
	LCD_BusData8(poX);
	LCD_WR_LOW();
	LCD_WR_HIGH();
	
	//Y
	//Write command
	LCD_RS_LOW();
	LCD_BusData8(0x0021>>8);
	LCD_WR_LOW();
	LCD_WR_HIGH();
	LCD_BusData8(0x0021);
	LCD_WR_LOW();
	LCD_WR_HIGH();
	//Write Data
	LCD_RS_HIGH();
	LCD_BusData8(poY>>8);
	LCD_WR_LOW();
	LCD_WR_HIGH();
	LCD_BusData8(poY);
	LCD_WR_LOW();
	LCD_WR_HIGH();
	
	//Start to write to display RAM
	//Write command
	LCD_RS_LOW();
	LCD_BusData8(0x0022>>8);
	LCD_WR_LOW();
	LCD_WR_HIGH();
	LCD_BusData8(0x0022);
	LCD_WR_LOW();
	LCD_WR_HIGH();
		
	LCD_CS_HIGH();		
	
}


void LCD_SetPixel(uint16_t poX, uint16_t poY,uint16_t color)
{
	LCD_SetXY(poX,poY);
	LCD_WriteData(color);
}

void LCD_DrawCircle(int16_t poX, int16_t poY, int16_t r,uint16_t color)
{
	int x = -r, y = 0, err = 2-2*r, e2;
	do {
		LCD_SetPixel(poX-x, poY+y,color);
		LCD_SetPixel(poX+x, poY+y,color);
		LCD_SetPixel(poX+x, poY-y,color);
		LCD_SetPixel(poX-x, poY-y,color);
		e2 = err;
		if (e2 <= y) {
			err += ++y*2+1;
			if (-x == y && e2 <= x) e2 = 0;
		}
		if (e2 > x) err += ++x*2+1;
	} while (x <= 0);
}

void LCD_DrawLine(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t color)
{
    int x = x1-x0;
    int y = y1-y0;
    int dx = abs(x), sx = x0<x1 ? 1 : -1;
    int dy = -abs(y), sy = y0<y1 ? 1 : -1;
    int err = dx+dy, e2; /* error value e_xy */
    for (;;)
	{ 
		// loop
	    LCD_SetPixel(x0,y0,color);
	    e2 = 2*err;
	    if (e2 >= dy) 
		{ 
			// e_xy+e_x > 0
		    if (x0 == x1) break;
		    err += dy; x0 += sx;
	    }
	    if (e2 <= dx) 
		{  
			//e_xy+e_y < 0
		    if (y0 == y1) break;
		    err += dx; y0 += sy;
	    }
    }
}

//void LCD_DrawLine1(int x1, int y1, int x2, int y2)
//{
	//if (y1==y2)
	////drawHLine(x1, y1, x2-x1);
	//else if (x1==x2)
	////drawVLine(x1, y1, y2-y1);
	//else
	//{
		//unsigned int	dx = (x2 > x1 ? x2 - x1 : x1 - x2);
		//short			xstep =  x2 > x1 ? 1 : -1;
		//unsigned int	dy = (y2 > y1 ? y2 - y1 : y1 - y2);
		//short			ystep =  y2 > y1 ? 1 : -1;
		//int				col = x1, row = y1;
//
		//cbi(P_CS, B_CS);
		//if (dx < dy)
		//{
			//int t = - (dy >> 1);
			//while (true)
			//{
				//setXY (col, row, col, row);
				//LCD_WriteData (fch, fcl);
				//if (row == y2)
				//return;
				//row += ystep;
				//t += dx;
				//if (t >= 0)
				//{
					//col += xstep;
					//t   -= dy;
				//}
			//}
		//}
		//else
		//{
			//int t = - (dx >> 1);
			//while (true)
			//{
				//setXY (col, row, col, row);
				//LCD_Write_DATA (fch, fcl);
				//if (col == x2)
				//return;
				//col += xstep;
				//t += dy;
				//if (t >= 0)
				//{
					//row += ystep;
					//t   -= dx;
				//}
			//}
		//}
		//sbi(P_CS, B_CS);
	//}
	//clrXY();
//}

void LCD_DrawVerticalLine(unsigned int poX, unsigned int poY,unsigned int length,unsigned int color)
{
	unsigned int i;

	LCD_SetXY(poX,poY);
	LCD_SetOrientation(1);
	if(length+poY>LCD_MAX_Y)
	{
		length=LCD_MAX_Y-poY;
	}

	for(i=0;i<length;i++)
	{
		LCD_WriteData(color);
	}
}

void LCD_DrawHorizontalLine(unsigned int poX, unsigned int poY,unsigned int length,unsigned int color)
{
	unsigned int i;

	LCD_SetXY(poX,poY);
	LCD_SetOrientation(0);
	if(length+poX>LCD_MAX_X)
	{
		length=LCD_MAX_X-poX;
	}
	for(i=0;i<length;i++)
	{
		LCD_WriteData(color);
	}
}

void LCD_DrawRectangle(unsigned int poX, unsigned int poY, unsigned int length,unsigned int width,unsigned int color)
{
	LCD_DrawHorizontalLine(poX, poY, length, color);
	LCD_DrawHorizontalLine(poX, poY+width, length, color);

	LCD_DrawVerticalLine(poX, poY, width,color);
	LCD_DrawVerticalLine(poX + length, poY, width,color);
}

void LCD_DrawRoundRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	if (x1>x2)
	{
		//swap(int, x1, x2);
	}
	if (y1>y2)
	{
		//swap(int, y1, y2);
	}
	if ((x2-x1)>4 && (y2-y1)>4)
	{
		LCD_SetPixel(x1+1,y1+1,color);
		LCD_SetPixel(x2-1,y1+1,color);
		LCD_SetPixel(x1+1,y2-1,color);
		LCD_SetPixel(x2-1,y2-1,color);
		LCD_DrawHorizontalLine(x1+2, y1, x2-x1-4,color);
		LCD_DrawHorizontalLine(x1+2, y2, x2-x1-4,color);
		LCD_DrawVerticalLine(x1, y1+2, y2-y1-4,color);
		LCD_DrawVerticalLine(x2, y1+2, y2-y1-4,color);
	}
}

void LCD_PaintScreen(uint16_t color)
{
	unsigned int i, f;
	uint8_t ch,cl;
	ch=color>>8;
	cl=color;
	
	LCD_BusPinsAsOutput();
	LCD_CS_LOW();
	LCD_RS_HIGH();					
	for(i=0; i<320; i++)
	{
		for(f=0; f<240; f++)
		{
			//LCD_WriteData(color);
			LCD_BusData8(ch);
			LCD_WR_LOW();
			LCD_WR_HIGH();
			LCD_BusData8(cl);
			LCD_WR_LOW();
			LCD_WR_HIGH();	
		}
	}
	LCD_CS_HIGH();


}

void LCD_FillCircle(int poX, int poY, int r,unsigned int color)
{
	int x = -r, y = 0, err = 2-2*r, e2;
	do {
		LCD_DrawVerticalLine(poX-x,poY-y,2*y,color);
		LCD_DrawVerticalLine(poX+x,poY-y,2*y,color);

		e2 = err;
		if (e2 <= y) {
			err += ++y*2+1;
			if (-x == y && e2 <= x) e2 = 0;
		}
		if (e2 > x) err += ++x*2+1;
	} while (x <= 0);
}

void LCD_FillRectangle(unsigned int poX, unsigned int poY, unsigned int length, unsigned int width, unsigned int color)
{
	unsigned int i;

	for(i=0;i<width;i++)
	{
		if(DisplayDirect == LEFT2RIGHT)
		LCD_DrawHorizontalLine(poX, poY+i, length, color);
		else if (DisplayDirect ==  DOWN2UP)
		LCD_DrawHorizontalLine(poX, poY-i, length, color);
		else if(DisplayDirect == RIGHT2LEFT)
		LCD_DrawHorizontalLine(poX, poY-i, length, color);
		else if(DisplayDirect == UP2DOWN)
		LCD_DrawHorizontalLine(poX, poY+i, length, color);

	}
}

void LCD_DrawChar(unsigned int poX, unsigned int poY, unsigned char ascii, unsigned int size, unsigned int fgcolor)
{
	unsigned char i, f, temp = 0;

	LCD_SetXY(poX,poY);

	if((ascii < 0x20)||(ascii > 0x7e))//Unsupported char.
	{
		ascii = '?';
	}
	
	for(i=0;i<8;i++)
	{
		temp = simpleFont[ascii-0x20][i];
		for(f=0;f<8;f++)
		{
			if((temp>>f)&0x01)
			{
				if(DisplayDirect == LEFT2RIGHT)
				LCD_FillRectangle(poX+i*size, poY+f*size, size, size, fgcolor);
				else if(DisplayDirect == DOWN2UP)
				LCD_FillRectangle(poX+f*size, poY-i*size, size, size, fgcolor);
				else if(DisplayDirect == RIGHT2LEFT)
				LCD_FillRectangle(poX-i*size, poY-f*size, size, size, fgcolor);
				else if(DisplayDirect == UP2DOWN)
				LCD_FillRectangle(poX-f*size, poY+i*size, size, size, fgcolor);
			}
		}
	}
}

void LCD_DrawString(unsigned int poX, unsigned int poY, char *string, unsigned int size, unsigned int fgcolor)
{
    //unsigned char i;

    while(*string != '\0')
    {
		
        //for(i=0;i<8;i++)
        {
        	LCD_DrawChar(poX, poY, *string, size, fgcolor);
			
        }
		string++;
        //*string++;
        if(DisplayDirect == LEFT2RIGHT){
            if(poX < LCD_MAX_X){
                poX += 8*size; // Move cursor right
            }
        } else if(DisplayDirect == DOWN2UP){
            if(poY > 0){
                poY -= 8*size; // Move cursor right
            }
        } else if(DisplayDirect == RIGHT2LEFT) {
            if(poX > 0){
                poX -= 8*size; // Move cursor right
            }
        } else if(DisplayDirect == UP2DOWN){
            if(poY < LCD_MAX_Y){
                poY += 8*size; // Move cursor right
            }
        }
    }
	
}

void LCD_DrawBmp(unsigned short ulXs, unsigned short ulYs, unsigned short length, unsigned short height, unsigned short *buf)
{
	unsigned short i, j;
	unsigned short length_null = 0;

	if(ulXs + length > LCD_MAX_X){
		length_null = ulXs + length - LCD_MAX_X;
	}

	if(ulYs + height > LCD_MAX_Y){
		height -= ulYs + height - LCD_MAX_Y;
	}

	for(j = 0; j < height; j++){
		LCD_SetXY(ulXs, ulYs);
		for(i = 0; i < length; i++){
			LCD_WriteData(*buf++);
		}
		ulYs++;
		buf += length_null;
	}
}

void LCD_exitStandBy(void)
{
	LCD_WriteCommand(0x0010);
	LCD_WriteData(0x14E0);
	delayMicroseconds(100);
	LCD_WriteCommand(0x0007);
	LCD_WriteData(0x0133);
}

void LCD_SetOrientation(uint16_t HV)//horizontal or vertical
{
	LCD_WriteCommand(0x03);
	if(HV==1)//vertical
	{
		LCD_WriteData(0x5038);
	}
	else//horizontal
	{
		LCD_WriteData(0x5030);
	}
	LCD_WriteCommand(0x0022); //Start to write to display RAM
}

void LCD_SetDisplayDirect(uint8_t Direction)
{
	DisplayDirect = Direction;
}

//void LCD_printChar(byte c, int x, int y)
//{
	//byte i,ch;
	//word j;
	//word temp;
//
	//LCD_CS_LOW();
	//
	//if (!_transparent)
	//{
		//if (orient==PORTRAIT)
		//{
			//setXY(x,y,x+cfont.x_size-1,y+cfont.y_size-1);
			//
			//temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;
			//for(j=0;j<((cfont.x_size/8)*cfont.y_size);j++)
			//{
				//ch=pgm_read_byte(&cfont.font[temp]);
				//for(i=0;i<8;i++)
				//{
					//if((ch&(1<<(7-i)))!=0)
					//{
						//setPixel((fch<<8)|fcl);
					//}
					//else
					//{
						//setPixel((bch<<8)|bcl);
					//}
				//}
				//temp++;
			//}
		//}
		//else
		//{
			//temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;
//
			//for(j=0;j<((cfont.x_size/8)*cfont.y_size);j+=(cfont.x_size/8))
			//{
				//setXY(x,y+(j/(cfont.x_size/8)),x+cfont.x_size-1,y+(j/(cfont.x_size/8)));
				//for (int zz=(cfont.x_size/8)-1; zz>=0; zz--)
				//{
					//ch=pgm_read_byte(&cfont.font[temp+zz]);
					//for(i=0;i<8;i++)
					//{
						//if((ch&(1<<i))!=0)
						//{
							//setPixel((fch<<8)|fcl);
						//}
						//else
						//{
							//setPixel((bch<<8)|bcl);
						//}
					//}
				//}
				//temp+=(cfont.x_size/8);
			//}
		//}
	//}
	//else
	//{
		//temp=((c-cfont.offset)*((cfont.x_size/8)*cfont.y_size))+4;
		//for(j=0;j<cfont.y_size;j++)
		//{
			//for (int zz=0; zz<(cfont.x_size/8); zz++)
			//{
				//ch=pgm_read_byte(&cfont.font[temp+zz]);
				//for(i=0;i<8;i++)
				//{
					//setXY(x+i+(zz*8),y+j,x+i+(zz*8)+1,y+j+1);
					//
					//if((ch&(1<<(7-i)))!=0)
					//{
						//setPixel((fch<<8)|fcl);
					//}
				//}
			//}
			//temp+=(cfont.x_size/8);
		//}
	//}
//
	//sbi(P_CS, B_CS);
	//clrXY();
//}
//
#endif

void setup()
{
	char buffer[50];
	Serial.begin(9600);
	Serial.println("Display Test..\r\n");
	LCD_Init();
	DisplayDirect = UP2DOWN;
        LCD_SetOrientation(2);
        LCD_PaintScreen(LCD_BLACK);

        LCD_DrawCircle(100,100,10,LCD_BLUE);
	//DisplayDirect = LEFT2RIGHT;
	LCD_DrawString(20,20," Hallo",3,LCD_WHITE);
	#define BOXSIZE 40
	LCD_FillRectangle(0,0,BOXSIZE,BOXSIZE,LCD_RED);
        for(int x=119;x<319;x++)
	{
		LCD_DrawLine(139,x,239,x,LCD_YELLOW);
	}

	//Read Device ID
	//static uint16_t identifier;
	uint16_t identifier = LCD_ReadRegister(0x0);
        Serial.println(identifier,HEX);
	for (int z=0;z<21;z++)
	{
		sprintf(&buffer[0],"Wert=%d",z);
		LCD_FillRectangle(150,278,15,18,LCD_YELLOW);
		LCD_DrawString(159,239,&buffer[0],1,LCD_BLUE);
		delay(100);
	}
	for (float x=-10.0;x<10.0;x+=0.01)
	{
		float y=sin(x)/x*30;
		//LCD_SetPixel(x*10,100+y,LCD_RED);
		LCD_FillCircle(190+y,219+x*10,2,LCD_RED);
	}
	
	


	
	  

}

void loop()
{

	  /* add main program code here, this code starts again each time it ends */

}

