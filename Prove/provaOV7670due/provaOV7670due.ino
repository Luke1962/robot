//
// Source code for application to transmit image from ov7670 to PC via USB
// Example for Arduino Due
// By Siarhei Charkes in 2016
// http://privateblog.info 
//http://www.commenthow.com/article/display/13456?offset=0&max=1

#include <digitalWriteFast\digitalWriteFast.h>
#include <Wire.h>
#include <SPI.h>
#pragma region LIBRERIE ILI9341_due tft 


#include <SPI.h>
#include <ILI9341_due/ILI9341_due_config.h>
#include <ILI9341_due/ILI9341_due.h>

#include "ILI9341_due\fonts\Arial_bold_14.h"

//#define TFT_CS 10
//#define TFT_DC 9
//#define TFT_RST 8
#define TFT_CS 2
#define TFT_DC 3
#define TFT_RST 4

ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);


#pragma endregion


//uint8_t myImage[240][320];//was uint8_t myImage[240][320];
uint8_t myImage[240][320];//was uint8_t myImage[240][320];
#pragma region OV7670
	#pragma region Collegamenti OV7670
	#define PCLK 32
	#define RESET 33
	#define VSYNC 52
	#define OE 5
	#define HREF 6
	#define XCLK 7

	//
	//#define D8 44
	//#define D7 45
	//#define D6 46
	//#define D5 47
	//#define D4 48
	//#define D3 49
	//#define D2 50
	//#define D1 51

	// Camera input/output pin connection to Arduino
	#define WRST  25      // Output Write Pointer Reset
	#define RRST  23      // Output Read Pointer Reset
	#define WEN   24      // Output Write Enable
	//#define VSYNC 6      // Input Vertical Sync marking frame capture
	//#define PCLK  8      // Output FIFO buffer output clock
	//WAS
	//#define WRST  25      // Output Write Pointer Reset
	//#define RRST  23      // Output Read Pointer Reset
	//#define WEN   24      // Output Write Enable
	//#define VSYNC 22      // Input Vertical Sync marking frame capture
	//#define RCLK  26      // set OE to low gnd

	// FIFO Ram input pins
	#define DO7   51     
	#define DO6   50   
	#define DO5   49   
	#define DO4   48   
	#define DO3   47   
	#define DO2   46   
	#define DO1   45
	#define DO0   44 

	//#define DO7   40     
	//#define DO6   39   
	//#define DO5   38   
	//#define DO4   37   
	//#define DO3   36   
	//#define DO2   35   
	//#define DO1   34
	//#define DO0   33 

	//ORIGINAL was
	//#define DO7   28     
	//#define DO6   29   
	//#define DO5   30   
	//#define DO4   31   
	//#define DO3   32   
	//#define DO2   33   
	//#define DO1   34
	//#define DO0   35 

	// SDCARD
	// MISO, MOSI, and SCK are also available in a consistent physical location on the ICSP header; 
	// this is useful, for example, in designing a shield that works on the Uno and the Mega. 
	// On the Arduino Mega, this is 
	// 50 (MISO) 
	// 51 (MOSI) 
	// 52 (SCK) 
	// 53 (SS) 
	#pragma endregion

	#pragma region OV7670 VARIABLES
	#define address  (0x42 >> 1)

	#define vga   0
	#define qvga  1
	#define qqvga   2
	#define yuv422  0
	#define rgb565  1
	#define bayerRGB  2
							  // VGA Default 
	int PHOTO_WIDTH = 640;
	int PHOTO_HEIGHT = 480;
	int PHOTO_BYTES_PER_PIXEL = 2;
	// Command and Parameter related Strings
	String RawCommandLine = "";
	String Command = "QVGA";
	String FPSParam = "ThirtyFPS";
	String AWBParam = "SAWB";
	String AECParam = "HistAEC";
	String YUVMatrixParam = "YUVMatrixOn";
	String DenoiseParam = "DenoiseNo";
	String EdgeParam = "EdgeNo";
	String ABLCParam = "AblcON";

	enum ResolutionType
	{
		VGA,
		VGAP,
		QVGA,
		QQVGA,
		None
	};

	ResolutionType resolution ;
	char format;
	// Serial Input 
	const int BUFFERLENGTH = 255;
	char IncomingByte[BUFFERLENGTH];   // for incoming serial data


	#pragma endregion

	#pragma region indirizzi dei registri OV7670
		//da TestOV7670due
						/* Registers */
		#define REG_GAIN    0x00  /* Gain lower 8 bits (rest in vref) */
		#define REG_BLUE    0x01  /* blue gain */
		#define REG_RED       0x02  /* red gain */
		#define REG_VREF    0x03  /* Pieces of GAIN, VSTART, VSTOP */
		#define REG_COM1    0x04  /* Control 1 */
		#define COM1_CCIR656  0x40    /* CCIR656 enable */

		#define REG_BAVE    0x05  /* U/B Average level */
		#define REG_GbAVE   0x06  /* Y/Gb Average level */
		#define REG_AECHH   0x07  /* AEC MS 5 bits */
		#define REG_RAVE    0x08  /* V/R Average level */
		#define REG_COM2    0x09  /* Control 2 */
		#define COM2_SSLEEP         0x10  /* Soft sleep mode */
		#define REG_PID           0x0a  /* Product ID MSB */
		#define REG_VER           0x0b  /* Product ID LSB */
		#define REG_COM3    0x0c  /* Control 3 */
		#define COM3_SWAP         0x40  /* Byte swap */
		#define COM3_SCALEEN          0x08  /* Enable scaling */
		#define COM3_DCWEN          0x04  /* Enable downsamp/crop/window */
		#define REG_COM4    0x0d  /* Control 4 */
		#define REG_COM5    0x0e  /* All "reserved" */
		#define REG_COM6    0x0f  /* Control 6 */
		#define REG_AECH    0x10  /* More bits of AEC value */
		#define REG_CLKRC   0x11  /* Clocl control */
		#define CLK_EXT           0x40  /* Use external clock directly */
		#define CLK_SCALE   0x3f  /* Mask for internal clock scale */
		#define REG_COM7    0x12  /* Control 7 */ //REG mean address.
		#define COM7_RESET          0x80  /* Register reset */
		#define COM7_FMT_MASK         0x38
		#define COM7_FMT_VGA          0x00
		#define COM7_FMT_CIF          0x20  /* CIF format */
		#define COM7_FMT_QVGA         0x10  /* QVGA format */
		#define COM7_FMT_QCIF         0x08  /* QCIF format */
		#define COM7_RGB          0x04  /* bits 0 and 2 - RGB format */
		#define COM7_YUV          0x00  /* YUV */
		#define COM7_BAYER          0x01  /* Bayer format */
		#define COM7_PBAYER         0x05  /* "Processed bayer" */
		#define REG_COM8    0x13  /* Control 8 */
		#define COM8_FASTAEC          0x80  /* Enable fast AGC/AEC */
		#define COM8_AECSTEP          0x40  /* Unlimited AEC step size */
		#define COM8_BFILT    0x20  /* Band filter enable */
		#define COM8_AGC    0x04  /* Auto gain enable */
		#define COM8_AWB    0x02  /* White balance enable */
		#define COM8_AEC    0x01  /* Auto exposure enable */
		#define REG_COM9    0x14  /* Control 9- gain ceiling */
		#define REG_COM10   0x15  /* Control 10 */
		#define COM10_HSYNC         0x40  /* HSYNC instead of HREF */
		#define COM10_PCLK_HB         0x20  /* Suppress PCLK on horiz blank */
		#define COM10_HREF_REV          0x08  /* Reverse HREF */
		#define COM10_VS_LEAD         0x04  /* VSYNC on clock leading edge */
		#define COM10_VS_NEG          0x02  /* VSYNC negative */
		#define COM10_HS_NEG          0x01  /* HSYNC negative */
		#define REG_HSTART    0x17  /* Horiz start high bits */
		#define REG_HSTOP   0x18  /* Horiz stop high bits */
		#define REG_VSTART    0x19  /* Vert start high bits */
		#define REG_VSTOP   0x1a  /* Vert stop high bits */
		#define REG_PSHFT   0x1b  /* Pixel delay after HREF */
		#define REG_MIDH    0x1c  /* Manuf. ID high */
		#define REG_MIDL    0x1d  /* Manuf. ID low */
		#define REG_MVFP    0x1e  /* Mirror / vflip */
		#define MVFP_MIRROR         0x20  /* Mirror image */
		#define MVFP_FLIP   0x10  /* Vertical flip */

		#define REG_AEW           0x24  /* AGC upper limit */
		#define REG_AEB           0x25    /* AGC lower limit */
		#define REG_VPT           0x26  /* AGC/AEC fast mode op region */
		#define REG_HSYST   0x30  /* HSYNC rising edge delay */
		#define REG_HSYEN   0x31  /* HSYNC falling edge delay */
		#define REG_HREF    0x32  /* HREF pieces */
		#define REG_TSLB    0x3a  /* lots of stuff */
		#define TSLB_YLAST    0x04  /* UYVY or VYUY - see com13 */
		#define REG_COM11   0x3b  /* Control 11 */
		#define COM11_NIGHT         0x80  /* NIght mode enable */
		#define COM11_NMFR          0x60  /* Two bit NM frame rate */
		#define COM11_HZAUTO          0x10  /* Auto detect 50/60 Hz */
		#define COM11_50HZ          0x08  /* Manual 50Hz select */
		#define COM11_EXP   0x02
		#define REG_COM12   0x3c  /* Control 12 */
		#define COM12_HREF          0x80  /* HREF always */
		#define REG_COM13   0x3d  /* Control 13 */
		#define COM13_GAMMA         0x80  /* Gamma enable */
		#define COM13_UVSAT         0x40  /* UV saturation auto adjustment */
		#define COM13_UVSWAP          0x01  /* V before U - w/TSLB */
		#define REG_COM14   0x3e  /* Control 14 */
		#define COM14_DCWEN         0x10  /* DCW/PCLK-scale enable */
		#define REG_EDGE    0x3f  /* Edge enhancement factor */
		#define REG_COM15   0x40  /* Control 15 */
		#define COM15_R10F0         0x00  /* Data range 10 to F0 */
		#define COM15_R01FE         0x80  /*      01 to FE */
		#define COM15_R00FF         0xc0  /*      00 to FF */
		#define COM15_RGB565          0x10  /* RGB565 output */
		#define COM15_RGB555          0x30  /* RGB555 output */
		#define REG_COM16   0x41  /* Control 16 */
		#define COM16_AWBGAIN         0x08  /* AWB gain enable */
		#define REG_COM17   0x42  /* Control 17 */
		#define COM17_AECWIN          0xc0  /* AEC window - must match COM4 */
		#define COM17_CBAR          0x08  /* DSP Color bar */
		/*
		* This matrix defines how the colors are generated, must be
		* tweaked to adjust hue and saturation.
		*
		* Order: v-red, v-green, v-blue, u-red, u-green, u-blue
		* They are nine-bit signed quantities, with the sign bit
		* stored in0x58.Sign for v-red is bit 0, and up from there.
		*/
		#define REG_CMATRIX_BASE  0x4f
		#define CMATRIX_LEN           6
		#define REG_CMATRIX_SIGN  0x58
		#define REG_BRIGHT    0x55  /* Brightness */
		#define REG_CONTRAS         0x56  /* Contrast control */
		#define REG_GFIX    0x69  /* Fix gain control */
		#define REG_REG76   0x76  /* OV's name */
		#define R76_BLKPCOR         0x80  /* Black pixel correction enable */
		#define R76_WHTPCOR         0x40  /* White pixel correction enable */
		#define REG_RGB444          0x8c  /* RGB 444 control */
		#define R444_ENABLE         0x02  /* Turn on RGB444, overrides 5x5 */
		#define R444_RGBX   0x01  /* Empty nibble at end */
		#define REG_HAECC1    0x9f  /* Hist AEC/AGC control 1 */
		#define REG_HAECC2    0xa0  /* Hist AEC/AGC control 2 */
		#define REG_BD50MAX         0xa5  /* 50hz banding step limit */
		#define REG_HAECC3    0xa6  /* Hist AEC/AGC control 3 */
		#define REG_HAECC4    0xa7  /* Hist AEC/AGC control 4 */
		#define REG_HAECC5    0xa8  /* Hist AEC/AGC control 5 */
		#define REG_HAECC6    0xa9  /* Hist AEC/AGC control 6 */
		#define REG_HAECC7    0xaa  /* Hist AEC/AGC control 7 */
		#define REG_BD60MAX         0xab  /* 60hz banding step limit */
		#define REG_GAIN    0x00  /* Gain lower 8 bits (rest in vref) */
		#define REG_BLUE    0x01  /* blue gain */
		#define REG_RED           0x02  /* red gain */
		#define REG_VREF    0x03  /* Pieces of GAIN, VSTART, VSTOP */
		#define REG_COM1    0x04  /* Control 1 */
		#define COM1_CCIR656          0x40  /* CCIR656 enable */
		#define REG_BAVE    0x05  /* U/B Average level */
		#define REG_GbAVE   0x06  /* Y/Gb Average level */
		#define REG_AECHH   0x07  /* AEC MS 5 bits */
		#define REG_RAVE    0x08  /* V/R Average level */
		#define REG_COM2    0x09  /* Control 2 */
		#define COM2_SSLEEP         0x10  /* Soft sleep mode */
		#define REG_PID           0x0a  /* Product ID MSB */
		#define REG_VER           0x0b  /* Product ID LSB */
		#define REG_COM3    0x0c  /* Control 3 */
		#define COM3_SWAP         0x40  /* Byte swap */
		#define COM3_SCALEEN          0x08  /* Enable scaling */
		#define COM3_DCWEN          0x04  /* Enable downsamp/crop/window */
		#define REG_COM4    0x0d  /* Control 4 */
		#define REG_COM5    0x0e  /* All "reserved" */
		#define REG_COM6    0x0f  /* Control 6 */
		#define REG_AECH    0x10  /* More bits of AEC value */
		#define REG_CLKRC   0x11  /* Clocl control */
		#define CLK_EXT           0x40  /* Use external clock directly */
		#define CLK_SCALE   0x3f  /* Mask for internal clock scale */
		#define REG_COM7    0x12  /* Control 7 */
		#define COM7_RESET          0x80  /* Register reset */
		#define COM7_FMT_MASK         0x38
		#define COM7_FMT_VGA          0x00
		#define COM7_FMT_CIF          0x20  /* CIF format */
		#define COM7_FMT_QVGA         0x10  /* QVGA format */
		#define COM7_FMT_QCIF         0x08  /* QCIF format */
		#define COM7_RGB    0x04  /* bits 0 and 2 - RGB format */
		#define COM7_YUV    0x00  /* YUV */
		#define COM7_BAYER          0x01  /* Bayer format */
		#define COM7_PBAYER         0x05  /* "Processed bayer" */
		#define REG_COM8    0x13  /* Control 8 */
		#define COM8_FASTAEC          0x80  /* Enable fast AGC/AEC */
		#define COM8_AECSTEP          0x40  /* Unlimited AEC step size */
		#define COM8_BFILT    0x20  /* Band filter enable */
		#define COM8_AGC    0x04  /* Auto gain enable */
		#define COM8_AWB    0x02  /* White balance enable */
		#define COM8_AEC    0x01  /* Auto exposure enable */
		#define REG_COM9    0x14  /* Control 9- gain ceiling */
		#define REG_COM10   0x15  /* Control 10 */
		#define COM10_HSYNC         0x40  /* HSYNC instead of HREF */
		#define COM10_PCLK_HB         0x20  /* Suppress PCLK on horiz blank */
		#define COM10_HREF_REV          0x08  /* Reverse HREF */
		#define COM10_VS_LEAD           0x04  /* VSYNC on clock leading edge */
		#define COM10_VS_NEG          0x02  /* VSYNC negative */
		#define COM10_HS_NEG          0x01  /* HSYNC negative */
		#define REG_HSTART    0x17  /* Horiz start high bits */
		#define REG_HSTOP   0x18  /* Horiz stop high bits */
		#define REG_VSTART    0x19  /* Vert start high bits */
		#define REG_VSTOP   0x1a  /* Vert stop high bits */
		#define REG_PSHFT   0x1b  /* Pixel delay after HREF */
		#define REG_MIDH    0x1c  /* Manuf. ID high */
		#define REG_MIDL    0x1d  /* Manuf. ID low */
		#define REG_MVFP    0x1e  /* Mirror / vflip */
		#define MVFP_MIRROR         0x20  /* Mirror image */
		#define MVFP_FLIP   0x10  /* Vertical flip */
		#define REG_AEW           0x24  /* AGC upper limit */
		#define REG_AEB           0x25  /* AGC lower limit */
		#define REG_VPT           0x26  /* AGC/AEC fast mode op region */
		#define REG_HSYST   0x30  /* HSYNC rising edge delay */
		#define REG_HSYEN   0x31  /* HSYNC falling edge delay */
		#define REG_HREF    0x32  /* HREF pieces */
		#define REG_TSLB    0x3a  /* lots of stuff */
		#define TSLB_YLAST    0x04  /* UYVY or VYUY - see com13 */
		#define REG_COM11   0x3b  /* Control 11 */
		#define COM11_NIGHT         0x80  /* NIght mode enable */
		#define COM11_NMFR          0x60  /* Two bit NM frame rate */
		#define COM11_HZAUTO          0x10  /* Auto detect 50/60 Hz */
		#define COM11_50HZ          0x08  /* Manual 50Hz select */
		#define COM11_EXP   0x02
		#define REG_COM12   0x3c  /* Control 12 */
		#define COM12_HREF          0x80  /* HREF always */
		#define REG_COM13   0x3d  /* Control 13 */
		#define COM13_GAMMA         0x80  /* Gamma enable */
		#define COM13_UVSAT         0x40  /* UV saturation auto adjustment */
		#define COM13_UVSWAP          0x01  /* V before U - w/TSLB */
		#define REG_COM14   0x3e  /* Control 14 */
		#define COM14_DCWEN         0x10  /* DCW/PCLK-scale enable */
		#define REG_EDGE    0x3f  /* Edge enhancement factor */
		#define REG_COM15   0x40  /* Control 15 */
		#define COM15_R10F0         0x00  /* Data range 10 to F0 */
		#define COM15_R01FE         0x80  /*      01 to FE */
		#define COM15_R00FF         0xc0  /*      00 to FF */
		#define COM15_RGB565          0x10  /* RGB565 output */
		#define COM15_RGB555          0x30  /* RGB555 output */
		#define REG_COM16   0x41  /* Control 16 */
		#define COM16_AWBGAIN         0x08  /* AWB gain enable */
		#define REG_COM17   0x42  /* Control 17 */
		#define COM17_AECWIN          0xc0  /* AEC window - must match COM4 */
		#define COM17_CBAR          0x08  /* DSP Color bar */

		#define CMATRIX_LEN             6
		#define REG_BRIGHT    0x55  /* Brightness */
		#define REG_REG76   0x76  /* OV's name */
		#define R76_BLKPCOR         0x80  /* Black pixel correction enable */
		#define R76_WHTPCOR         0x40  /* White pixel correction enable */
		#define REG_RGB444          0x8c  /* RGB 444 control */
		#define R444_ENABLE         0x02  /* Turn on RGB444, overrides 5x5 */
		#define R444_RGBX   0x01  /* Empty nibble at end */
		#define REG_HAECC1    0x9f  /* Hist AEC/AGC control 1 */
		#define REG_HAECC2    0xa0  /* Hist AEC/AGC control 2 */
		#define REG_BD50MAX         0xa5  /* 50hz banding step limit */
		#define REG_HAECC3    0xa6  /* Hist AEC/AGC control 3 */
		#define REG_HAECC4    0xa7  /* Hist AEC/AGC control 4 */
		#define REG_HAECC5    0xa8  /* Hist AEC/AGC control 5 */
		#define REG_HAECC6    0xa9  /* Hist AEC/AGC control 6 */
		#define REG_HAECC7    0xaa  /* Hist AEC/AGC control 7 */
		#define REG_BD60MAX         0xab  /* 60hz banding step limit */
		#define MTX1            0x4f  /* Matrix Coefficient 1 */
		#define MTX2            0x50  /* Matrix Coefficient 2 */
		#define MTX3            0x51  /* Matrix Coefficient 3 */
		#define MTX4            0x52  /* Matrix Coefficient 4 */
		#define MTX5            0x53  /* Matrix Coefficient 5 */
		#define MTX6            0x54  /* Matrix Coefficient 6 */
		#define REG_CONTRAS         0x56  /* Contrast control */
		#define MTXS            0x58  /* Matrix Coefficient Sign */
		#define AWBC7           0x59  /* AWB Control 7 */
		#define AWBC8           0x5a  /* AWB Control 8 */
		#define AWBC9           0x5b  /* AWB Control 9 */
		#define AWBC10            0x5c  /* AWB Control 10 */
		#define AWBC11            0x5d  /* AWB Control 11 */
		#define AWBC12            0x5e  /* AWB Control 12 */
		#define REG_GFI           0x69  /* Fix gain control */
		#define GGAIN           0x6a  /* G Channel AWB Gain */
		#define DBLV            0x6b  
		#define AWBCTR3           0x6c  /* AWB Control 3 */
		#define AWBCTR2           0x6d  /* AWB Control 2 */
		#define AWBCTR1           0x6e  /* AWB Control 1 */
		#define AWBCTR0           0x6f  /* AWB Control 0 */

		#pragma endregion

	#pragma region OV7670 REGISTERS AND SETTINGS da OV7670 FIFO
	// da OV7670 FIFO
 
	// Register addresses and values
	#define CLKRC                 0x11 
	#define CLKRC_VALUE_VGA       0x01  // Raw Bayer
	#define CLKRC_VALUE_QVGA      0x01
	#define CLKRC_VALUE_QQVGA     0x01
	#define CLKRC_VALUE_NIGHTMODE_FIXED   0x03 // Fixed Frame
	#define CLKRC_VALUE_NIGHTMODE_AUTO    0x80 // Auto Frame Rate Adjust

	#define COM7                                   0x12 
	#define COM7_VALUE_VGA                         0x01   // Raw Bayer
	#define COM7_VALUE_VGA_COLOR_BAR               0x03   // Raw Bayer
	#define COM7_VALUE_VGA_PROCESSED_BAYER         0x05   // Processed Bayer
	#define COM7_VALUE_QVGA                        0x00
	#define COM7_VALUE_QVGA_COLOR_BAR              0x02
	#define COM7_VALUE_QVGA_PREDEFINED_COLOR_BAR   0x12
	#define COM7_VALUE_QQVGA                       0x00
	#define COM7_VALUE_QQVGA_COLOR_BAR             0x02   
	#define COM7_VALUE_QCIF                        0x08     // Predefined QCIF format
	#define COM7_VALUE_COLOR_BAR_QCIF              0x0A     // Predefined QCIF Format with ColorBar
	#define COM7_VALUE_RESET                       0x80


	#define COM3                            0x0C 
	#define COM3_VALUE_VGA                  0x00 // Raw Bayer
	#define COM3_VALUE_QVGA                 0x04
	#define COM3_VALUE_QQVGA                0x04  // From Docs
	#define COM3_VALUE_QQVGA_SCALE_ENABLED  0x0C  // Enable Scale and DCW
	#define COM3_VALUE_QCIF                 0x0C  // Enable Scaling and enable DCW

	#define COM14                            0x3E 
	#define COM14_VALUE_VGA                  0x00 // Raw Bayer
	#define COM14_VALUE_QVGA                 0x19
	#define COM14_VALUE_QQVGA                0x1A
	#define COM14_VALUE_MANUAL_SCALING       0x08   // Manual Scaling Enabled
	#define COM14_VALUE_NO_MANUAL_SCALING    0x00   // Manual Scaling DisEnabled

	#define SCALING_XSC                                  0x70
	#define SCALING_XSC_VALUE_VGA                        0x3A  // Raw Bayer
	#define SCALING_XSC_VALUE_QVGA                       0x3A
	#define SCALING_XSC_VALUE_QQVGA                      0x3A
	#define SCALING_XSC_VALUE_QQVGA_SHIFT1               0x3A
	#define SCALING_XSC_VALUE_COLOR_BAR                  0xBA  
	#define SCALING_XSC_VALUE_QCIF_COLOR_BAR_NO_SCALE    0x80 // Predefined QCIF with Color Bar and NO Scaling

	#define SCALING_YSC                                   0x71 
	#define SCALING_YSC_VALUE_VGA                         0x35 // Raw Bayer 
	#define SCALING_YSC_VALUE_QVGA                        0x35
	#define SCALING_YSC_VALUE_QQVGA                       0x35
	#define SCALING_YSC_VALUE_COLOR_BAR                   0x35  // 8 bar color bar
	#define SCALING_YSC_VALUE_COLOR_BAR_GREY              0xB5  // fade to grey color bar
	#define SCALING_YSC_VALUE_COLOR_BAR_SHIFT1            0xB5  // fade to grey color bar
	#define SCALING_YSC_VALUE_QCIF_COLOR_BAR_NO_SCALE     0x00  // Predefined QCIF with Color Bar and NO Scaling

	#define SCALING_DCWCTR               0x72 
	#define SCALING_DCWCTR_VALUE_VGA     0x11  // Raw Bayer
	#define SCALING_DCWCTR_VALUE_QVGA    0x11
	#define SCALING_DCWCTR_VALUE_QQVGA   0x22  

	#define SCALING_PCLK_DIV              0x73  
	#define SCALING_PCLK_DIV_VALUE_VGA    0xF0 // Raw Bayer
	#define SCALING_PCLK_DIV_VALUE_QVGA   0xF1
	#define SCALING_PCLK_DIV_VALUE_QQVGA  0xF2

	#define SCALING_PCLK_DELAY              0xA2
	#define SCALING_PCLK_DELAY_VALUE_VGA    0x02 // Raw Bayer
	#define SCALING_PCLK_DELAY_VALUE_QVGA   0x02
	#define SCALING_PCLK_DELAY_VALUE_QQVGA  0x02


	// Controls YUV order Used with COM13
	// Need YUYV format for Android Decoding- Default value is 0xD
	#define TSLB                                         0x3A
	#define TSLB_VALUE_YUYV_AUTO_OUTPUT_WINDOW_ENABLED   0x01 // No custom scaling
	#define TSLB_VALUE_YUYV_AUTO_OUTPUT_WINDOW_DISABLED  0x00 // For adjusting HSTART, etc. YUYV format
	#define TSLB_VALUE_UYVY_AUTO_OUTPUT_WINDOW_DISABLED  0x08 
	#define TSLB_VALUE_TESTVALUE                         0x04 // From YCbCr Reference 


	// Default value is 0x88
	// ok if you want YUYV order, no need to change
	#define COM13                      0x3D
	#define COM13_VALUE_DEFAULT        0x88
	#define COM13_VALUE_NOGAMMA_YUYV   0x00
	#define COM13_VALUE_GAMMA_YUYV     0x80
	#define COM13_VALUE_GAMMA_YVYU     0x82
	#define COM13_VALUE_YUYV_UVSATAUTOADJ_ON 0x40



	// Works with COM4
	#define COM17                                 0x42
	#define COM17_VALUE_AEC_NORMAL_NO_COLOR_BAR   0x00
	#define COM17_VALUE_AEC_NORMAL_COLOR_BAR      0x08 // Activate Color Bar for DSP

	#define COM4   0x0D

	// RGB Settings and Data format
	#define COM15    0x40


	// Night Mode
	#define COM11                             0x3B
	#define COM11_VALUE_NIGHTMODE_ON          0x80     // Night Mode
	#define COM11_VALUE_NIGHTMODE_OFF         0x00 
	#define COM11_VALUE_NIGHTMODE_ON_EIGHTH   0xE0     // Night Mode 1/8 frame rate minimum
	#define COM11_VALUE_NIGHTMODE_FIXED       0x0A 
	#define COM11_VALUE_NIGHTMODE_AUTO        0xEA     // Night Mode Auto Frame Rate Adjust


	// Color Matrix Control YUV
	#define MTX1	   	0x4f 
	#define MTX1_VALUE	0x80

	#define MTX2	   	0x50 
	#define MTX2_VALUE	0x80

	#define MTX3	   	0x51 
	#define MTX3_VALUE	0x00

	#define MTX4	   	0x52 
	#define MTX4_VALUE	0x22

	#define MTX5	   	0x53 
	#define MTX5_VALUE	0x5e

	#define MTX6	   	0x54 
	#define MTX6_VALUE	0x80

	#define CONTRAS	   	0x56 
	#define CONTRAS_VALUE	0x40

	#define MTXS	   	0x58 
	#define MTXS_VALUE	0x9e



	// COM8
	#define COM8                    0x13
	#define COM8_VALUE_AWB_OFF      0xE5
	#define COM8_VALUE_AWB_ON       0xE7




	// Automatic White Balance
	#define AWBC1	   	0x43 
	#define AWBC1_VALUE	0x14

	#define AWBC2	   	0x44 
	#define AWBC2_VALUE	0xf0

	#define AWBC3	   	0x45 
	#define AWBC3_VALUE  	0x34

	#define AWBC4	   	0x46 
	#define AWBC4_VALUE	0x58

	#define AWBC5	        0x47 
	#define AWBC5_VALUE	0x28

	#define AWBC6	   	0x48 
	#define AWBC6_VALUE	0x3a

	#define AWBC7           0x59
	#define AWBC7_VALUE     0x88

	#define AWBC8          0x5A
	#define AWBC8_VALUE    0x88

	#define AWBC9          0x5B
	#define AWBC9_VALUE    0x44

	#define AWBC10         0x5C
	#define AWBC10_VALUE   0x67

	#define AWBC11         0x5D
	#define AWBC11_VALUE   0x49

	#define AWBC12         0x5E
	#define AWBC12_VALUE   0x0E

	#define AWBCTR3        0x6C
	#define AWBCTR3_VALUE  0x0A

	#define AWBCTR2        0x6D
	#define AWBCTR2_VALUE  0x55

	#define AWBCTR1        0x6E
	#define AWBCTR1_VALUE  0x11

	#define AWBCTR0                0x6F
	#define AWBCTR0_VALUE_NORMAL   0x9F
	#define AWBCTR0_VALUE_ADVANCED 0x9E


	// Gain
	#define COM9                        0x14
	#define COM9_VALUE_MAX_GAIN_128X    0x6A
	#define COM9_VALUE_4XGAIN           0x10    // 0001 0000

	#define BLUE          0x01    // AWB Blue Channel Gain
	#define BLUE_VALUE    0x40

	#define RED            0x02    // AWB Red Channel Gain
	#define RED_VALUE      0x40

	#define GGAIN            0x6A   // AWB Green Channel Gain
	#define GGAIN_VALUE      0x40

	#define COM16	   	0x41 
	#define COM16_VALUE	0x08 // AWB Gain on

	#define GFIX	   	0x69 
	#define GFIX_VALUE	0x00

	// Edge Enhancement Adjustment
	#define EDGE	   	0x3f 
	#define EDGE_VALUE	0x00

	#define REG75	   	0x75 
	#define REG75_VALUE	0x03

	#define REG76	   	0x76 
	#define REG76_VALUE	0xe1

	// DeNoise 
	#define DNSTH	   	0x4c 
	#define DNSTH_VALUE	0x00

	#define REG77	   	0x77 
	#define REG77_VALUE	0x00

	// Denoise and Edge Enhancement
	#define COM16_VALUE_DENOISE_OFF_EDGE_ENHANCEMENT_OFF_AWBGAIN_ON     0x08 // Denoise off, AWB Gain on
	#define COM16_VALUE_DENOISE_ON__EDGE_ENHANCEMENT_OFF__AWBGAIN_ON    0x18
	#define COM16_VALUE_DENOISE_OFF__EDGE_ENHANCEMENT_ON__AWBGAIN_ON    0x28
	#define COM16_VALUE_DENOISE_ON__EDGE_ENHANCEMENT_ON__AWBGAIN_ON     0x38 // Denoise on,  Edge Enhancement on, AWB Gain on


	// 30FPS Frame Rate , PCLK = 24Mhz
	#define CLKRC_VALUE_30FPS  0x80

	#define DBLV               0x6b
	#define DBLV_VALUE_30FPS   0x0A

	#define EXHCH              0x2A
	#define EXHCH_VALUE_30FPS  0x00

	#define EXHCL              0x2B
	#define EXHCL_VALUE_30FPS  0x00

	#define DM_LNL               0x92
	#define DM_LNL_VALUE_30FPS   0x00

	#define DM_LNH               0x93
	#define DM_LNH_VALUE_30FPS   0x00

	#define COM11_VALUE_30FPS    0x0A   


	// Saturation Control
	#define SATCTR	   	0xc9 
	#define SATCTR_VALUE	0x60


	// AEC/AGC - Automatic Exposure/Gain Control
	#define GAIN		0x00 
	#define GAIN_VALUE	0x00

	#define AEW	   	0x24 
	#define AEW_VALUE	0x95

	#define AEB	   	0x25 
	#define AEB_VALUE	0x33

	#define VPT	   	0x26 
	#define VPT_VALUE	0xe3


	/*
	// Gamma
	#define SLOP	   	0x7a
	#define SLOP_VALUE	0x20

	#define GAM1	   	0x7b
	#define GAM1_VALUE	0x10

	#define GAM2	   	0x7c
	#define GAM2_VALUE      0x1e

	#define GAM3	   	0x7d
	#define GAM3_VALUE	0x35

	#define GAM4	   	0x7e
	#define GAM4_VALUE	0x5a

	#define GAM5	   	0x7f
	#define GAM5_VALUE	0x69

	#define GAM6	   	0x80
	#define GAM6_VALUE	0x76

	#define GAM7	   	0x81
	#define GAM7_VALUE	0x80

	#define GAM8	   	0x82
	#define GAM8_VALUE	0x88

	#define GAM9	   	0x83
	#define GAM9_VALUE	0x8f

	#define GAM10	   	0x84
	#define GAM10_VALUE	0x96

	#define GAM11	   	0x85
	#define GAM11_VALUE	0xa3

	#define GAM12	   	0x86
	#define GAM12_VALUE	0xaf

	#define GAM13	   	0x87
	#define GAM13_VALUE	0xc4

	#define GAM14	   	0x88
	#define GAM14_VALUE	0xd7

	#define GAM15	   	0x89
	#define GAM15_VALUE	0xe8
	*/



	// AEC/AGC Control- Histogram
	#define HAECC1	   	0x9f 
	#define HAECC1_VALUE	0x78

	#define HAECC2	   	0xa0 
	#define HAECC2_VALUE	0x68

	#define HAECC3	   	0xa6 
	#define HAECC3_VALUE	0xd8

	#define HAECC4	   	0xa7 
	#define HAECC4_VALUE	0xd8

	#define HAECC5	   	0xa8 
	#define HAECC5_VALUE	0xf0

	#define HAECC6	   	0xa9 
	#define HAECC6_VALUE	0x90

	#define HAECC7	                        0xaa  // AEC Algorithm selection
	#define HAECC7_VALUE_HISTOGRAM_AEC_ON	0x94 
	#define HAECC7_VALUE_AVERAGE_AEC_ON     0x00



	/*
	// Gamma
	#define SLOP	   	0x7a
	#define SLOP_VALUE	0x20

	#define GAM1	   	0x7b
	#define GAM1_VALUE	0x10

	#define GAM2	   	0x7c
	#define GAM2_VALUE      0x1e

	#define GAM3	   	0x7d
	#define GAM3_VALUE	0x35

	#define GAM4	   	0x7e
	#define GAM4_VALUE	0x5a

	#define GAM5	   	0x7f
	#define GAM5_VALUE	0x69

	#define GAM6	   	0x80
	#define GAM6_VALUE	0x76

	#define GAM7	   	0x81
	#define GAM7_VALUE	0x80

	#define GAM8	   	0x82
	#define GAM8_VALUE	0x88

	#define GAM9	   	0x83
	#define GAM9_VALUE	0x8f

	#define GAM10	   	0x84
	#define GAM10_VALUE	0x96

	#define GAM11	   	0x85
	#define GAM11_VALUE	0xa3

	#define GAM12	   	0x86
	#define GAM12_VALUE	0xaf

	#define GAM13	   	0x87
	#define GAM13_VALUE	0xc4

	#define GAM14	   	0x88
	#define GAM14_VALUE	0xd7

	#define GAM15	   	0x89
	#define GAM15_VALUE	0xe8

	*/

	// Array Control
	#define CHLF	   	0x33 
	#define CHLF_VALUE	0x0b

	#define ARBLM	   	0x34 
	#define ARBLM_VALUE	0x11



	// ADC Control
	#define ADCCTR1	   	0x21 
	#define ADCCTR1_VALUE	0x02

	#define ADCCTR2	   	0x22 
	#define ADCCTR2_VALUE	0x91

	#define ADC	   	0x37 
	#define ADC_VALUE       0x1d

	#define ACOM	   	0x38 
	#define ACOM_VALUE	0x71

	#define OFON	   	0x39 
	#define OFON_VALUE	0x2a


	// Black Level Calibration
	#define ABLC1	   	0xb1 
	#define ABLC1_VALUE	0x0c

	#define THL_ST		0xb3 
	#define THL_ST_VALUE	0x82


	// Window Output 
	#define HSTART               0x17
	#define HSTART_VALUE_DEFAULT 0x11
	#define HSTART_VALUE_VGA     0x13     
	#define HSTART_VALUE_QVGA    0x13   
	#define HSTART_VALUE_QQVGA   0x13   // Works

	#define HSTOP                0x18
	#define HSTOP_VALUE_DEFAULT  0x61
	#define HSTOP_VALUE_VGA      0x01   
	#define HSTOP_VALUE_QVGA     0x01  
	#define HSTOP_VALUE_QQVGA    0x01   // Works 

	#define HREF                  0x32
	#define HREF_VALUE_DEFAULT    0x80
	#define HREF_VALUE_VGA        0xB6   
	#define HREF_VALUE_QVGA       0x24
	#define HREF_VALUE_QQVGA      0xA4  

	#define VSTRT                0x19
	#define VSTRT_VALUE_DEFAULT  0x03
	#define VSTRT_VALUE_VGA      0x02
	#define VSTRT_VALUE_QVGA     0x02
	#define VSTRT_VALUE_QQVGA    0x02  

	#define VSTOP                0x1A
	#define VSTOP_VALUE_DEFAULT  0x7B
	#define VSTOP_VALUE_VGA      0x7A
	#define VSTOP_VALUE_QVGA     0x7A
	#define VSTOP_VALUE_QQVGA    0x7A  

	#define VREF                 0x03
	#define VREF_VALUE_DEFAULT   0x03
	#define VREF_VALUE_VGA       0x0A   
	#define VREF_VALUE_QVGA      0x0A
	#define VREF_VALUE_QQVGA     0x0A  


	// I2C 
	//was #define OV7670_I2C_ADDRESS                 0x21
	#define OV7670_I2C_WRITE_ADDRESS                 (0x42>>1)
	#define OV7670_I2C_READ_ADDRESS                  (0x43>>1)
	#define I2C_ERROR_WRITING_START_ADDRESS      11
	#define I2C_ERROR_WRITING_DATA               22

	#define DATA_TOO_LONG                  1      // data too long to fit in transmit buffer 
	#define NACK_ON_TRANSMIT_OF_ADDRESS    2      // received NACK on transmit of address 
	#define NACK_ON_TRANSMIT_OF_DATA       3      // received NACK on transmit of data 
	#define OTHER_ERROR                    4      // other error 

	#define I2C_READ_START_ADDRESS_ERROR        33
	#define I2C_READ_DATA_SIZE_MISMATCH_ERROR   44


	#pragma endregion

	#pragma region OV7670 structures & utils

		struct regval_list {
			uint8_t reg_num;
			uint8_t value;
		};


		const struct regval_list qvga_ov7670[] PROGMEM = {
		  { REG_COM14, 0x19 },
		  { 0x72, 0x11 },
		  { 0x73, 0xf1 },

		  { REG_HSTART, 0x16 },
		  { REG_HSTOP, 0x04 },
		  { REG_HREF, 0xF6 },
		  { REG_VSTART, 0x02 },
		  { REG_VSTOP, 0x7a },
		  { REG_VREF, 0x0a },

		  { 0xff, 0xff }, /* END MARKER */
		};
		const struct regval_list qvga_30fps_Pbayer[] PROGMEM = {
		  { REG_COM7, 0x11 },
		  { REG_COM3, 0x04 },
		  { REG_COM14, 0x1a },
		  { 0x70, 0x3a },
		  { 0x71, 0x35 },
		  { 0x72, 0x11 },
		  { 0x73, 0xf9 },
		  { 0xa2, 0x02 },

		  { REG_HSTART, 0x16 },
		  { REG_HSTOP, 0x04 },
		  { REG_HREF, 0xF6 },
		  { REG_VSTART, 0x02 },
		  { REG_VSTOP, 0x7a },
		  { REG_VREF, 0x0a },

		  { 0xff, 0xff }, /* END MARKER */
		};
		const struct regval_list qvga_30fps_rgb565[] PROGMEM = {
		  { REG_COM7, 0b00010100 },
		  { REG_COM3, 0x04 },//? tristate option
		  { REG_COM14,0b00000000  },//0x1a bit[2:0]=010=PCLK divided by 4; bit3=Manual scaling allowed
		  { REG_COM15, 0b11010000 },
		  { 0x70, 0x3a },
		  { 0x71, 0x35 },
		  { 0x72, 0x11 },
		  { 0x73, 0xf9 },
		  { 0xa2, 0x02 },

		  { REG_HSTART, 0x16 },
		  { REG_HSTOP, 0x04 },
		  { REG_HREF, 0xF6 },
		  { REG_VSTART, 0x02 },
		  { REG_VSTOP, 0x7a },
		  { REG_VREF, 0x0a },

		  { 0xff, 0xff }, /* END MARKER */
		};

		const struct regval_list yuv422_ov7670[] PROGMEM = {
		  { REG_COM7, 0x0 },  /* Selects YUV mode */
		  { REG_RGB444, 0 },  /* No RGB444 please */
		  { REG_COM1, 0 },
		  { REG_COM15, COM15_R00FF },
		  { REG_COM9, 0x6A }, /* 128x gain ceiling; 0x8 is reserved bit */
		  { 0x4f, 0x80 },   /* "matrix coefficient 1" */
		  { 0x50, 0x80 },   /* "matrix coefficient 2" */
		  { 0x51, 0 },    /* vb */
		  { 0x52, 0x22 },   /* "matrix coefficient 4" */
		  { 0x53, 0x5e },   /* "matrix coefficient 5" */
		  { 0x54, 0x80 },   /* "matrix coefficient 6" */
		  { REG_COM13, COM13_UVSAT },
		  { 0xff, 0xff },   /* END MARKER */
		};

		//include reset dei registri
		const struct regval_list ov7670_default_regs[] PROGMEM = {//from the linux driver
		  { REG_COM7, COM7_RESET },
		  { REG_TSLB, 0x04 }, /* OV */
		  { REG_COM7, 0 },  /* VGA */
		  /*
		  * Set the hardware window.  These values from OV don't entirely
		  * make sense - hstop is less than hstart.  But they work...
		  */
		  { REG_HSTART, 0x13 }, { REG_HSTOP, 0x01 },
		  { REG_HREF, 0xb6 }, { REG_VSTART, 0x02 },
		  { REG_VSTOP, 0x7a }, { REG_VREF, 0x0a },

		  { REG_COM3, 0 }, { REG_COM14, 0 },
		  /* Mystery scaling numbers */
		  { 0x70, 0x3a }, { 0x71, 0x35 },
		  { 0x72, 0x11 }, { 0x73, 0xf0 },
		  { 0xa2,/* 0x02 changed to 1*/1 }, { REG_COM10, 0x0 },
		  /* Gamma curve values */
		  { 0x7a, 0x20 }, { 0x7b, 0x10 },
		  { 0x7c, 0x1e }, { 0x7d, 0x35 },
		  { 0x7e, 0x5a }, { 0x7f, 0x69 },
		  { 0x80, 0x76 }, { 0x81, 0x80 },
		  { 0x82, 0x88 }, { 0x83, 0x8f },
		  { 0x84, 0x96 }, { 0x85, 0xa3 },
		  { 0x86, 0xaf }, { 0x87, 0xc4 },
		  { 0x88, 0xd7 }, { 0x89, 0xe8 },
		  /* AGC and AEC parameters.  Note we start by disabling those features,
		  then turn them only after tweaking the values. */
		  { REG_COM8, COM8_FASTAEC | COM8_AECSTEP },
		  { REG_GAIN, 0 }, { REG_AECH, 0 },
		  { REG_COM4, 0x40 }, /* magic reserved bit */
		  { REG_COM9, 0x18 }, /* 4x gain + magic rsvd bit */
		  { REG_BD50MAX, 0x05 }, { REG_BD60MAX, 0x07 },
		  { REG_AEW, 0x95 }, { REG_AEB, 0x33 },
		  { REG_VPT, 0xe3 }, { REG_HAECC1, 0x78 },
		  { REG_HAECC2, 0x68 }, { 0xa1, 0x03 }, /* magic */
		  { REG_HAECC3, 0xd8 }, { REG_HAECC4, 0xd8 },
		  { REG_HAECC5, 0xf0 }, { REG_HAECC6, 0x90 },
		  { REG_HAECC7, 0x94 },
		  { REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_AGC | COM8_AEC },
		  { 0x30, 0 }, { 0x31, 0 },//disable some delays
		  /* Almost all of these are magic "reserved" values.  */
		  { REG_COM5, 0x61 }, { REG_COM6, 0x4b },
		  { 0x16, 0x02 }, { REG_MVFP, 0x07 },
		  { 0x21, 0x02 }, { 0x22, 0x91 },
		  { 0x29, 0x07 }, { 0x33, 0x0b },
		  { 0x35, 0x0b }, { 0x37, 0x1d },
		  { 0x38, 0x71 }, { 0x39, 0x2a },
		  { REG_COM12, 0x78 }, { 0x4d, 0x40 },
		  { 0x4e, 0x20 }, { REG_GFIX, 0 },
		  /*{0x6b, 0x4a},*/{ 0x74, 0x10 },
		  { 0x8d, 0x4f }, { 0x8e, 0 },
		  { 0x8f, 0 }, { 0x90, 0 },
		  { 0x91, 0 }, { 0x96, 0 },
		  { 0x9a, 0 }, { 0xb0, 0x84 },
		  { 0xb1, 0x0c }, { 0xb2, 0x0e },
		  { 0xb3, 0x82 }, { 0xb8, 0x0a },

		  /* More reserved magic, some of which tweaks white balance */
		  { 0x43, 0x0a }, { 0x44, 0xf0 },
		  { 0x45, 0x34 }, { 0x46, 0x58 },
		  { 0x47, 0x28 }, { 0x48, 0x3a },
		  { 0x59, 0x88 }, { 0x5a, 0x88 },
		  { 0x5b, 0x44 }, { 0x5c, 0x67 },
		  { 0x5d, 0x49 }, { 0x5e, 0x0e },
		  { 0x6c, 0x0a }, { 0x6d, 0x55 },
		  { 0x6e, 0x11 }, { 0x6f, 0x9e }, /* it was 0x9F "9e for advance AWB" */
		  { 0x6a, 0x40 }, { REG_BLUE, 0x40 },
		  { REG_RED, 0x60 },
		  { REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_AGC | COM8_AEC | COM8_AWB },

		  /* Matrix coefficients */
		  { 0x4f, 0x80 }, { 0x50, 0x80 },
		  { 0x51, 0 },    { 0x52, 0x22 },
		  { 0x53, 0x5e }, { 0x54, 0x80 },
		  { 0x58, 0x9e },

		  { REG_COM16, COM16_AWBGAIN }, { REG_EDGE, 0 },
		  { 0x75, 0x05 }, { REG_REG76, 0xe1 },
		  { 0x4c, 0 },     { 0x77, 0x01 },
		  { REG_COM13, /*0xc3*/0x48 }, { 0x4b, 0x09 },
		  { 0xc9, 0x60 },   /*{REG_COM16, 0x38},*/
		  { 0x56, 0x40 },

		  { 0x34, 0x11 }, { REG_COM11, COM11_EXP | COM11_HZAUTO },
		  { 0xa4, 0x82/*Was 0x88*/ }, { 0x96, 0 },
		  { 0x97, 0x30 }, { 0x98, 0x20 },
		  { 0x99, 0x30 }, { 0x9a, 0x84 },
		  { 0x9b, 0x29 }, { 0x9c, 0x03 },
		  { 0x9d, 0x4c }, { 0x9e, 0x3f },
		  { 0x78, 0x04 },

		  /* Extra-weird stuff.  Some sort of multiplexor register */
		  { 0x79, 0x01 }, { 0xc8, 0xf0 },
		  { 0x79, 0x0f }, { 0xc8, 0x00 },
		  { 0x79, 0x10 }, { 0xc8, 0x7e },
		  { 0x79, 0x0a }, { 0xc8, 0x80 },
		  { 0x79, 0x0b }, { 0xc8, 0x01 },
		  { 0x79, 0x0c }, { 0xc8, 0x0f },
		  { 0x79, 0x0d }, { 0xc8, 0x20 },
		  { 0x79, 0x09 }, { 0xc8, 0x80 },
		  { 0x79, 0x02 }, { 0xc8, 0xc0 },
		  { 0x79, 0x03 }, { 0xc8, 0x40 },
		  { 0x79, 0x05 }, { 0xc8, 0x30 },
		  { 0x79, 0x26 },

		  { 0xff, 0xff }, /* END MARKER */
		};


		void wrSensorRegs8_8(const struct regval_list reglist[]) {
			int index = 0;
			regval_list regpaar = reglist[index];

			do {
				write(regpaar.reg_num, regpaar.value);
				index++;
				regpaar = reglist[index];
			} while (regpaar.reg_num != 0xFF);
		}
		// imposta 
		void setColor(void) {
			wrSensorRegs8_8(yuv422_ov7670);
		}
		// esegue i settaggi: qvga_ov7670 e qvga_30fps_rgb565
		void setRes(void) {
			write(REG_COM3, 4); // REG_COM3 enable scaling
			 wrSensorRegs8_8(qvga_ov7670);
			 wrSensorRegs8_8(qvga_30fps_rgb565);
		}
		//	Esegue il reset ed imposta le porte in Input/output
		void camInit(void) {
			pinMode(RESET, OUTPUT);
			digitalWrite(RESET, HIGH);

			////solo per Arduino UNO o MEGA
			//  pinMode(pullup1, OUTPUT);
			//  digitalWrite(pullup1, HIGH);//solo per Arduino UNO o MEGA

			//  pinMode(pullup2, OUTPUT);
			//  digitalWrite(pullup2, HIGH);//solo per Arduino UNO o MEGA


			write(REG_COM7, 0x80); //RESET ALL REGISTERS TO DEFAUL VALUE

			delayMicroseconds(100);
			wrSensorRegs8_8(ov7670_default_regs);
			write(REG_COM10, 32);//PCLK does not toggle on HBLANK.

			pinMode(13, OUTPUT);


			pinMode(DO0, INPUT);
			pinMode(DO1, INPUT);
			pinMode(DO2, INPUT);
			pinMode(DO3, INPUT);
			pinMode(DO4, INPUT);
			pinMode(DO5, INPUT);
			pinMode(DO6, INPUT);
			pinMode(DO7, INPUT);

			pinMode(VSYNC, INPUT);
			pinMode(PCLK, INPUT);
		}
		void camInitNew(void) {
			pinMode(RESET, OUTPUT);
			digitalWrite(RESET, HIGH);

			////solo per Arduino UNO o MEGA
			//  pinMode(pullup1, OUTPUT);
			//  digitalWrite(pullup1, HIGH);//solo per Arduino UNO o MEGA

			//  pinMode(pullup2, OUTPUT);
			//  digitalWrite(pullup2, HIGH);//solo per Arduino UNO o MEGA


			pinMode(13, OUTPUT);


			pinMode(DO0, INPUT);
			pinMode(DO1, INPUT);
			pinMode(DO2, INPUT);
			pinMode(DO3, INPUT);
			pinMode(DO4, INPUT);
			pinMode(DO5, INPUT);
			pinMode(DO6, INPUT);
			pinMode(DO7, INPUT);

			pinMode(VSYNC, INPUT);
			pinMode(PCLK, INPUT);


			write(REG_COM7, 0x80); //RESET ALL REGISTERS TO DEFAUL VALUE

			delayMicroseconds(100);
//			wrSensorRegs8_8(qvga_30fps_rgb565);// alternative: qvga_30fps_Pbayer

			write(REG_COM10, 32);//bit5=1:PCLK does not toggle on HBLANK., bit4=1: PCLK reverse ; bit1=1:VSYNC negative

		}
		void ReadRegisters()
		{
			byte data = 0;

			data = ReadRegisterValue(CLKRC);
			Serial.print(F("CLKRC = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(COM7);
			Serial.print(F("COM7 = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(COM3);
			Serial.print(F("COM3 = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(COM14);
			Serial.print(F("COM14 = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(SCALING_XSC);
			Serial.print(F("SCALING_XSC = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(SCALING_YSC);
			Serial.print(F("SCALING_YSC = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(SCALING_DCWCTR);
			Serial.print(F("SCALING_DCWCTR = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(SCALING_PCLK_DIV);
			Serial.print(F("SCALING_PCLK_DIV = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(SCALING_PCLK_DELAY);
			Serial.print(F("SCALING_PCLK_DELAY = "));
			Serial.println(data, HEX);

			//data = ReadRegisterValue(COM10);
			//Serial.print(F("COM10 (Vsync Polarity) = "));
			//Serial.println(data,HEX);

			// default value D
			data = ReadRegisterValue(TSLB);
			Serial.print(F("TSLB (YUV Order- Higher Bit, Bit[3]) = "));
			Serial.println(data, HEX);

			// default value 88
			data = ReadRegisterValue(COM13);
			Serial.print(F("COM13 (YUV Order - Lower Bit, Bit[1]) = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(COM17);
			Serial.print(F("COM17 (DSP Color Bar Selection) = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(COM4);
			Serial.print(F("COM4 (works with COM 17) = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(COM15);
			Serial.print(F("COM15 (COLOR FORMAT SELECTION) = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(COM11);
			Serial.print(F("COM11 (Night Mode) = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(COM8);
			Serial.print(F("COM8 (Color Control, AWB) = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(HAECC7);
			Serial.print(F("HAECC7 (AEC Algorithm Selection) = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(GFIX);
			Serial.print(F("GFIX = "));
			Serial.println(data, HEX);


			// Window Output
			data = ReadRegisterValue(HSTART);
			Serial.print(F("HSTART = "));
			Serial.println(data, HEX);
			//Serial.print(F(", "));
			//Serial.println(data, DEC);

			data = ReadRegisterValue(HSTOP);
			Serial.print(F("HSTOP = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(HREF);
			Serial.print(F("HREF = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(VSTRT);
			Serial.print(F("VSTRT = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(VSTOP);
			Serial.print(F("VSTOP = "));
			Serial.println(data, HEX);

			data = ReadRegisterValue(VREF);
			Serial.print(F("VREF = "));
			Serial.println(data, HEX);
		}

	#pragma endregion

	#pragma region OV7670 Setup Procedures

			void SetupOV7670ForQVGArgb565()// mio adattamanto
			{
				ResetCameraRegisters();
				int result = 0;
				String sresult = "";

				Serial.println(F("------------Setting Camera for QVGA (RGB565) ------------"));

				PHOTO_WIDTH = 320;
				PHOTO_HEIGHT = 240;
				PHOTO_BYTES_PER_PIXEL = 2;

				Serial.print(F("Photo Width = "));
				Serial.println(PHOTO_WIDTH);

				Serial.print(F("Photo Height = "));
				Serial.println(PHOTO_HEIGHT);

				Serial.print(F("Bytes Per Pixel = "));
				Serial.println(PHOTO_BYTES_PER_PIXEL);


				// Basic Registers
				//was result = OV7670WriteReg(CLKRC, CLKRC_VALUE_QVGA);
				result = OV7670WriteReg(CLKRC, 0x40);//clock esterno
				sresult = ParseI2CResult(result);
				Serial.print(F("CLKRC: "));
				Serial.println(sresult);

				//was result = OV7670WriteReg(COM7,COM7_RGB COM7_VALUE_QVGA);
				//result = OV7670WriteReg(COM7, COM7_VALUE_QVGA_COLOR_BAR );
				result = OV7670WriteReg(COM7, 0b00010110);// bit 1 = Color bar enable

				sresult = ParseI2CResult(result);
				Serial.print(F("COM7: "));
				Serial.println(sresult);
				result = OV7670WriteReg(COM15, 0xC1);
				result = OV7670WriteReg(REG_RGB444, 0x0);

				////was result = OV7670WriteReg(SCALING_XSC, SCALING_XSC_VALUE_QVGA);
				//result = OV7670WriteReg(SCALING_XSC, 0b10000000);
				//sresult = ParseI2CResult(result);
				//Serial.print(F("SCALING_XSC: "));
				//Serial.println(sresult);

				////wasresult = OV7670WriteReg(SCALING_YSC, SCALING_YSC_VALUE_QVGA);
				//result = OV7670WriteReg(SCALING_YSC, 0b10000000);
				//sresult = ParseI2CResult(result);
				//Serial.print(F("SCALING_YSC: "));
				//Serial.println(sresult);


				result = OV7670WriteReg(COM3, COM3_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM3: "));
				Serial.println(sresult);
#if 0
				result = OV7670WriteReg(COM14, COM14_VALUE_QVGA);//PCLK diviso 2, scaling manuale
				sresult = ParseI2CResult(result);
				Serial.print(F("COM14: "));
				Serial.println(sresult);


				result = OV7670WriteReg(SCALING_DCWCTR, SCALING_DCWCTR_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_DCWCTR: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_PCLK_DIV, SCALING_PCLK_DIV_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_PCLK_DIV: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_PCLK_DELAY, SCALING_PCLK_DELAY_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_PCLK_DELAY: "));
				Serial.println(sresult);

				// YUV order control change from default use with COM13
				//result = OV7670WriteReg(TSLB, TSLB_VALUE_YUYV_AUTO_OUTPUT_WINDOW_ENABLED);
				//result = OV7670WriteReg(TSLB, TSLB_VALUE_YUYV_AUTO_OUTPUT_WINDOW_DISABLED); // Works
				//result = OV7670WriteReg(TSLB, TSLB_VALUE_UYVY_AUTO_OUTPUT_WINDOW_DISABLED);
				//result = OV7670WriteReg(TSLB, TSLB_VALUE_TESTVALUE); 
				result = OV7670WriteReg(TSLB, 0x04);
				sresult = ParseI2CResult(result);
				Serial.print(F("TSLB: "));
				Serial.println(sresult);


				//COM13
				//result = OV7670WriteReg(COM13, COM13_VALUE_GAMMA_YUYV);
				//result = OV7670WriteReg(COM13, COM13_VALUE_DEFAULT);
				//result = OV7670WriteReg(COM13, COM13_VALUE_GAMMA_YVYU);
				//result = OV7670WriteReg(COM13, COM13_VALUE_NOGAMMA_YUYV);
				//result = OV7670WriteReg(COM13, COM13_VALUE_YUYV_UVSATAUTOADJ_ON); 
				// { REG_COM13, /*0xc3*/0x48 } // error in datasheet bit 3 controls YUV order
				//result = OV7670WriteReg(COM13, 0x48);
				//result = OV7670WriteReg(COM13, 0xC8); // needed for correct color bar
				result = OV7670WriteReg(COM13, 0xC2);   // from YCbCr reference specs 
														//result = OV7670WriteReg(COM13, 0x82);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM13: "));
				Serial.println(sresult);

				// COM17 - DSP Color Bar Enable/Disable
				// 0000 1000 => to Enable
				// 0x08  
				// COM17_VALUE  0x08 // Activate Color Bar for DSP
				//result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_COLOR_BAR);
				result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_NO_COLOR_BAR);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM17: "));
				Serial.println(sresult);




				// Set Additional Parameters

				// Set Camera Frames per second
				SetCameraFPSMode();

				// Set Camera Automatic Exposure Control
				SetCameraAEC();

				// Set Camera Automatic White Balance
				SetupCameraAWB();

				// Setup Undocumented Registers - Needed Minimum
				//SetupCameraUndocumentedRegisters();

				//// Set Color Matrix for YUV
				//if (YUVMatrixParam == "YUVMatrixOn")
				//{
				//	SetCameraColorMatrixYUV();
				//}

				// Set Camera Saturation
				SetCameraSaturationControl();

				// Denoise and Edge Enhancement
				SetupCameraDenoiseEdgeEnhancement();


				// Set up Camera Array Control
				SetupCameraArrayControl();


				// Set ADC Control
				SetupCameraADCControl();


				// Set Automatic Black Level Calibration
				SetupCameraABLC();
#endif // 0

				Serial.println(F("........... Setting Camera Window Output Parameters  ........"));

				// Change Window Output parameters after custom scaling
				result = OV7670WriteReg(HSTART, HSTART_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("HSTART: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HSTOP, HSTOP_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("HSTOP: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HREF, HREF_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("HREF: "));
				Serial.println(sresult);

				result = OV7670WriteReg(VSTRT, VSTRT_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("VSTRT: "));
				Serial.println(sresult);

				result = OV7670WriteReg(VSTOP, VSTOP_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("VSTOP: "));
				Serial.println(sresult);

				result = OV7670WriteReg(VREF, VREF_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("VREF: "));
				Serial.println(sresult);
			}

#pragma region altri settaggi disabilitati
#if 0


			void SetupOV7670ForQVGAYUV()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("--------------------------- Setting Camera for QVGA (YUV) ---------------------------"));

				PHOTO_WIDTH = 320;
				PHOTO_HEIGHT = 240;
				PHOTO_BYTES_PER_PIXEL = 2;

				Serial.print(F("Photo Width = "));
				Serial.println(PHOTO_WIDTH);

				Serial.print(F("Photo Height = "));
				Serial.println(PHOTO_HEIGHT);

				Serial.print(F("Bytes Per Pixel = "));
				Serial.println(PHOTO_BYTES_PER_PIXEL);


				// Basic Registers
				result = OV7670WriteReg(CLKRC, CLKRC_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("CLKRC: "));
				Serial.println(sresult);

				result = OV7670WriteReg(COM7, COM7_VALUE_QVGA);
				//result = OV7670WriteReg(COM7, COM7_VALUE_QVGA_COLOR_BAR );
				sresult = ParseI2CResult(result);
				Serial.print(F("COM7: "));
				Serial.println(sresult);

				result = OV7670WriteReg(COM3, COM3_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM3: "));
				Serial.println(sresult);

				result = OV7670WriteReg(COM14, COM14_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM14: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_XSC, SCALING_XSC_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_XSC: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_YSC, SCALING_YSC_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_YSC: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_DCWCTR, SCALING_DCWCTR_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_DCWCTR: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_PCLK_DIV, SCALING_PCLK_DIV_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_PCLK_DIV: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_PCLK_DELAY, SCALING_PCLK_DELAY_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_PCLK_DELAY: "));
				Serial.println(sresult);

				// YUV order control change from default use with COM13
				//result = OV7670WriteReg(TSLB, TSLB_VALUE_YUYV_AUTO_OUTPUT_WINDOW_ENABLED);
				//result = OV7670WriteReg(TSLB, TSLB_VALUE_YUYV_AUTO_OUTPUT_WINDOW_DISABLED); // Works
				//result = OV7670WriteReg(TSLB, TSLB_VALUE_UYVY_AUTO_OUTPUT_WINDOW_DISABLED);
				//result = OV7670WriteReg(TSLB, TSLB_VALUE_TESTVALUE); 
				result = OV7670WriteReg(TSLB, 0x04);
				sresult = ParseI2CResult(result);
				Serial.print(F("TSLB: "));
				Serial.println(sresult);

				//COM13
				//result = OV7670WriteReg(COM13, COM13_VALUE_GAMMA_YUYV);
				//result = OV7670WriteReg(COM13, COM13_VALUE_DEFAULT);
				//result = OV7670WriteReg(COM13, COM13_VALUE_GAMMA_YVYU);
				//result = OV7670WriteReg(COM13, COM13_VALUE_NOGAMMA_YUYV);
				//result = OV7670WriteReg(COM13, COM13_VALUE_YUYV_UVSATAUTOADJ_ON); 
				// { REG_COM13, /*0xc3*/0x48 } // error in datasheet bit 3 controls YUV order
				//result = OV7670WriteReg(COM13, 0x48);
				//result = OV7670WriteReg(COM13, 0xC8); // needed for correct color bar
				result = OV7670WriteReg(COM13, 0xC2);   // from YCbCr reference specs 
														//result = OV7670WriteReg(COM13, 0x82);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM13: "));
				Serial.println(sresult);

				// COM17 - DSP Color Bar Enable/Disable
				// 0000 1000 => to Enable
				// 0x08  
				// COM17_VALUE  0x08 // Activate Color Bar for DSP
				//result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_COLOR_BAR);
				result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_NO_COLOR_BAR);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM17: "));
				Serial.println(sresult);



				// Set Additional Parameters

				// Set Camera Frames per second
				SetCameraFPSMode();

				// Set Camera Automatic Exposure Control
				SetCameraAEC();

				// Set Camera Automatic White Balance
				SetupCameraAWB();

				// Setup Undocumented Registers - Needed Minimum
				SetupCameraUndocumentedRegisters();

				// Set Color Matrix for YUV
				if (YUVMatrixParam == "YUVMatrixOn")
				{
					SetCameraColorMatrixYUV();
				}

				// Set Camera Saturation
				SetCameraSaturationControl();

				// Denoise and Edge Enhancement
				SetupCameraDenoiseEdgeEnhancement();


				// Set up Camera Array Control
				SetupCameraArrayControl();


				// Set ADC Control
				SetupCameraADCControl();


				// Set Automatic Black Level Calibration
				SetupCameraABLC();


				Serial.println(F("........... Setting Camera Window Output Parameters  ........"));

				// Change Window Output parameters after custom scaling
				result = OV7670WriteReg(HSTART, HSTART_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("HSTART: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HSTOP, HSTOP_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("HSTOP: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HREF, HREF_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("HREF: "));
				Serial.println(sresult);

				result = OV7670WriteReg(VSTRT, VSTRT_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("VSTRT: "));
				Serial.println(sresult);

				result = OV7670WriteReg(VSTOP, VSTOP_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("VSTOP: "));
				Serial.println(sresult);

				result = OV7670WriteReg(VREF, VREF_VALUE_QVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("VREF: "));
				Serial.println(sresult);
			}
			// include il reset, output su seriale

			void SetupOV7670ForQQVGAYUV()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("--------------------------- Setting Camera for QQVGA YUV ---------------------------"));

				PHOTO_WIDTH = 160;
				PHOTO_HEIGHT = 120;
				PHOTO_BYTES_PER_PIXEL = 2;

				Serial.print(F("Photo Width = "));
				Serial.println(PHOTO_WIDTH);

				Serial.print(F("Photo Height = "));
				Serial.println(PHOTO_HEIGHT);

				Serial.print(F("Bytes Per Pixel = "));
				Serial.println(PHOTO_BYTES_PER_PIXEL);


				Serial.println(F("........... Setting Basic QQVGA Parameters  ........"));

				result = OV7670WriteReg(CLKRC, CLKRC_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("CLKRC: "));
				Serial.println(sresult);

				result = OV7670WriteReg(COM7, COM7_VALUE_QQVGA);
				//result = OV7670WriteReg(COM7, COM7_VALUE_QQVGA_COLOR_BAR );
				sresult = ParseI2CResult(result);
				Serial.print(F("COM7: "));
				Serial.println(sresult);

				result = OV7670WriteReg(COM3, COM3_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM3: "));
				Serial.println(sresult);

				result = OV7670WriteReg(COM14, COM14_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM14: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_XSC, SCALING_XSC_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_XSC: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_YSC, SCALING_YSC_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_YSC: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_DCWCTR, SCALING_DCWCTR_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_DCWCTR: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_PCLK_DIV, SCALING_PCLK_DIV_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_PCLK_DIV: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_PCLK_DELAY, SCALING_PCLK_DELAY_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_PCLK_DELAY: "));
				Serial.println(sresult);

				// YUV order control change from default use with COM13
				result = OV7670WriteReg(TSLB, TSLB_VALUE_YUYV_AUTO_OUTPUT_WINDOW_DISABLED); // Works
				sresult = ParseI2CResult(result);
				Serial.print(F("TSLB: "));
				Serial.println(sresult);

				//COM13
				//result = OV7670WriteReg(COM13, COM13_VALUE_GAMMA_YUYV);
				//result = OV7670WriteReg(COM13, COM13_VALUE_DEFAULT);
				//result = OV7670WriteReg(COM13, COM13_VALUE_GAMMA_YVYU);
				//result = OV7670WriteReg(COM13, COM13_VALUE_NOGAMMA_YUYV);
				//result = OV7670WriteReg(COM13, COM13_VALUE_YUYV_UVSATAUTOADJ_ON); 
				// { REG_COM13, /*0xc3*/0x48 } // error in datasheet bit 3 controls YUV order
				//result = OV7670WriteReg(COM13, 0x48);
				result = OV7670WriteReg(COM13, 0xC8);  // Gamma Enabled, UV Auto Adj On
				sresult = ParseI2CResult(result);
				Serial.print(F("COM13: "));
				Serial.println(sresult);

				// COM17 - DSP Color Bar Enable/Disable
				// 0000 1000 => to Enable
				// 0x08  
				// COM17_VALUE  0x08 // Activate Color Bar for DSP
				//result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_COLOR_BAR);
				result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_NO_COLOR_BAR);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM17: "));
				Serial.println(sresult);


				// Set Additional Parameters
				// Set Camera Frames per second
				SetCameraFPSMode();

				// Set Camera Automatic Exposure Control
				SetCameraAEC();

				// Set Camera Automatic White Balance
				SetupCameraAWB();

				// Setup Undocumented Registers - Needed Minimum
				SetupCameraUndocumentedRegisters();


				// Set Color Matrix for YUV
				if (YUVMatrixParam == "YUVMatrixOn")
				{
					SetCameraColorMatrixYUV();
				}

				// Set Camera Saturation
				SetCameraSaturationControl();

				// Denoise and Edge Enhancement
				SetupCameraDenoiseEdgeEnhancement();

				// Set New Gamma Values
				//SetCameraGamma();

				// Set Array Control
				SetupCameraArrayControl();

				// Set ADC Control
				SetupCameraADCControl();

				// Set Automatic Black Level Calibration
				SetupCameraABLC();


				Serial.println(F("........... Setting Camera Window Output Parameters  ........"));

				// Change Window Output parameters after custom scaling
				result = OV7670WriteReg(HSTART, HSTART_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("HSTART: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HSTOP, HSTOP_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("HSTOP: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HREF, HREF_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("HREF: "));
				Serial.println(sresult);

				result = OV7670WriteReg(VSTRT, VSTRT_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("VSTRT: "));
				Serial.println(sresult);

				result = OV7670WriteReg(VSTOP, VSTOP_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("VSTOP: "));
				Serial.println(sresult);

				result = OV7670WriteReg(VREF, VREF_VALUE_QQVGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("VREF: "));
				Serial.println(sresult);
			}

			void SetupCameraAdvancedAutoWhiteBalanceConfig()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("........... Setting Camera Advanced Auto White Balance Configs ........"));

				result = OV7670WriteReg(AWBC1, AWBC1_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBC1: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBC2, AWBC2_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBC2: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBC3, AWBC3_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBC3: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBC4, AWBC4_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBC4: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBC5, AWBC5_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBC5: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBC6, AWBC6_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBC6: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBC7, AWBC7_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBC7: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBC8, AWBC8_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBC8: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBC9, AWBC9_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBC9: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBC10, AWBC10_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBC10: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBC11, AWBC11_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBC11: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBC12, AWBC12_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBC12: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBCTR3, AWBCTR3_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBCTR3: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBCTR2, AWBCTR2_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBCTR2: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AWBCTR1, AWBCTR1_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBCTR1: "));
				Serial.println(sresult);
			}
			void SetupCameraUndocumentedRegisters()
			{
				// Write(0xb0,0x84); //adding this improve the color a little bit
				int result = 0;
				String sresult = "";

				Serial.println(F("........... Setting Camera Undocumented Registers ........"));
				result = OV7670WriteReg(0xB0, 0x84);
				sresult = ParseI2CResult(result);
				Serial.print(F("Setting B0 UNDOCUMENTED register to 0x84:= "));
				Serial.println(sresult);
			}
			void SetupCameraFor30FPS()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("........... Setting Camera to 30 FPS ........"));
				result = OV7670WriteReg(CLKRC, CLKRC_VALUE_30FPS);
				sresult = ParseI2CResult(result);
				Serial.print(F("CLKRC: "));
				Serial.println(sresult);

				result = OV7670WriteReg(DBLV, DBLV_VALUE_30FPS);
				sresult = ParseI2CResult(result);
				Serial.print(F("DBLV: "));
				Serial.println(sresult);

				result = OV7670WriteReg(EXHCH, EXHCH_VALUE_30FPS);
				sresult = ParseI2CResult(result);
				Serial.print(F("EXHCH: "));
				Serial.println(sresult);

				result = OV7670WriteReg(EXHCL, EXHCL_VALUE_30FPS);
				sresult = ParseI2CResult(result);
				Serial.print(F("EXHCL: "));
				Serial.println(sresult);

				result = OV7670WriteReg(DM_LNL, DM_LNL_VALUE_30FPS);
				sresult = ParseI2CResult(result);
				Serial.print(F("DM_LNL: "));
				Serial.println(sresult);

				result = OV7670WriteReg(DM_LNH, DM_LNH_VALUE_30FPS);
				sresult = ParseI2CResult(result);
				Serial.print(F("DM_LNH: "));
				Serial.println(sresult);

				result = OV7670WriteReg(COM11, COM11_VALUE_30FPS);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM11: "));
				Serial.println(sresult);

			}
			void SetupCameraABLC()
			{
				int result = 0;
				String sresult = "";

				// If ABLC is off then return otherwise
				// turn on ABLC.
				if (ABLCParam == "AblcOFF")
				{
					return;
				}

				Serial.println(F("........ Setting Camera ABLC ......."));

				result = OV7670WriteReg(ABLC1, ABLC1_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("ABLC1: "));
				Serial.println(sresult);

				result = OV7670WriteReg(THL_ST, THL_ST_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("THL_ST: "));
				Serial.println(sresult);
			}
			void SetupOV7670ForVGARawRGB()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("--------------------------- Setting Camera for VGA (Raw RGB) ---------------------------"));

				PHOTO_WIDTH = 640;
				PHOTO_HEIGHT = 480;
				PHOTO_BYTES_PER_PIXEL = 1;

				Serial.print(F("Photo Width = "));
				Serial.println(PHOTO_WIDTH);

				Serial.print(F("Photo Height = "));
				Serial.println(PHOTO_HEIGHT);

				Serial.print(F("Bytes Per Pixel = "));
				Serial.println(PHOTO_BYTES_PER_PIXEL);


				// Basic Registers
				result = OV7670WriteReg(CLKRC, CLKRC_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("CLKRC: "));
				Serial.println(sresult);

				result = OV7670WriteReg(COM7, COM7_VALUE_VGA);
				//result = OV7670WriteReg(COM7, COM7_VALUE_VGA_COLOR_BAR );
				sresult = ParseI2CResult(result);
				Serial.print(F("COM7: "));
				Serial.println(sresult);

				result = OV7670WriteReg(COM3, COM3_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM3: "));
				Serial.println(sresult);

				result = OV7670WriteReg(COM14, COM14_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM14: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_XSC, SCALING_XSC_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_XSC: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_YSC, SCALING_YSC_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_YSC: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_DCWCTR, SCALING_DCWCTR_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_DCWCTR: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_PCLK_DIV, SCALING_PCLK_DIV_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_PCLK_DIV: "));
				Serial.println(sresult);

				result = OV7670WriteReg(SCALING_PCLK_DELAY, SCALING_PCLK_DELAY_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("SCALING_PCLK_DELAY: "));
				Serial.println(sresult);

				// COM17 - DSP Color Bar Enable/Disable
				// 0000 1000 => to Enable
				// 0x08  
				// COM17_VALUE  0x08 // Activate Color Bar for DSP
				//result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_COLOR_BAR);
				result = OV7670WriteReg(COM17, COM17_VALUE_AEC_NORMAL_NO_COLOR_BAR);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM17: "));
				Serial.println(sresult);


				// Set Additional Parameters

				// Set Camera Frames per second
				SetCameraFPSMode();

				// Set Camera Automatic Exposure Control
				SetCameraAEC();

				// Needed Color Correction, green to red
				result = OV7670WriteReg(0xB0, 0x8c);
				sresult = ParseI2CResult(result);
				Serial.print(F("Setting B0 UNDOCUMENTED register to 0x84:= "));
				Serial.println(sresult);

				// Set Camera Saturation
				SetCameraSaturationControl();

				// Setup Camera Array Control
				SetupCameraArrayControl();

				// Set ADC Control
				SetupCameraADCControl();

				// Set Automatic Black Level Calibration
				SetupCameraABLC();



				Serial.println(F("........... Setting Camera Window Output Parameters  ........"));

				// Change Window Output parameters after custom scaling
				result = OV7670WriteReg(HSTART, HSTART_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("HSTART: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HSTOP, HSTOP_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("HSTOP: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HREF, HREF_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("HREF: "));
				Serial.println(sresult);

				result = OV7670WriteReg(VSTRT, VSTRT_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("VSTRT: "));
				Serial.println(sresult);

				result = OV7670WriteReg(VSTOP, VSTOP_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("VSTOP: "));
				Serial.println(sresult);

				result = OV7670WriteReg(VREF, VREF_VALUE_VGA);
				sresult = ParseI2CResult(result);
				Serial.print(F("VREF: "));
				Serial.println(sresult);
			}
			void SetupOV7670ForVGAProcessedBayerRGB()
			{
				int result = 0;
				String sresult = "";

				// Call Base for VGA Raw Bayer RGB Mode
				SetupOV7670ForVGARawRGB();

				Serial.println(F("------------- Setting Camera for VGA (Processed Bayer RGB) ----------------"));

				// Set key register for selecting processed bayer rgb output
				result = OV7670WriteReg(COM7, COM7_VALUE_VGA_PROCESSED_BAYER);
				//result = OV7670WriteReg(COM7, COM7_VALUE_VGA_COLOR_BAR );
				sresult = ParseI2CResult(result);
				Serial.print(F("COM7: "));
				Serial.println(sresult);

				result = OV7670WriteReg(TSLB, 0x04);
				sresult = ParseI2CResult(result);
				Serial.print(F("Initializing TSLB register result = "));
				Serial.println(sresult);

				// Needed Color Correction, green to red
				result = OV7670WriteReg(0xB0, 0x8c);
				sresult = ParseI2CResult(result);
				Serial.print(F("Setting B0 UNDOCUMENTED register to 0x84:= "));
				Serial.println(sresult);

				// Set Camera Automatic White Balance
				SetupCameraAWB();

				// Denoise and Edge Enhancement
				SetupCameraDenoiseEdgeEnhancement();
			}
			void SetupCameraAverageBasedAECAGC()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("-------------- Setting Camera Average Based AEC/AGC Registers ---------------"));

				result = OV7670WriteReg(AEW, AEW_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AEW: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AEB, AEB_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AEB: "));
				Serial.println(sresult);

				result = OV7670WriteReg(VPT, VPT_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("VPT: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HAECC7, HAECC7_VALUE_AVERAGE_AEC_ON);
				sresult = ParseI2CResult(result);
				Serial.print(F("HAECC7: "));
				Serial.println(sresult);
			}
			void SetCameraHistogramBasedAECAGC()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("-------------- Setting Camera Histogram Based AEC/AGC Registers ---------------"));

				result = OV7670WriteReg(AEW, AEW_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AEW: "));
				Serial.println(sresult);

				result = OV7670WriteReg(AEB, AEB_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("AEB: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HAECC1, HAECC1_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("HAECC1: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HAECC2, HAECC2_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("HAECC2: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HAECC3, HAECC3_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("HAECC3: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HAECC4, HAECC4_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("HAECC4: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HAECC5, HAECC5_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("HAECC5: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HAECC6, HAECC6_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("HAECC6: "));
				Serial.println(sresult);

				result = OV7670WriteReg(HAECC7, HAECC7_VALUE_HISTOGRAM_AEC_ON);
				sresult = ParseI2CResult(result);
				Serial.print(F("HAECC7: "));
				Serial.println(sresult);
			}
			void SetupCameraNightMode()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("......... Turning NIGHT MODE ON ........"));
				result = OV7670WriteReg(CLKRC, CLKRC_VALUE_NIGHTMODE_AUTO);
				sresult = ParseI2CResult(result);
				Serial.print(F("CLKRC: "));
				Serial.println(sresult);

				result = OV7670WriteReg(COM11, COM11_VALUE_NIGHTMODE_AUTO);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM11: "));
				Serial.println(sresult);
			}
			void SetupCameraSimpleAutomaticWhiteBalance()
			{
				/*
				i2c_salve_Address = 0x42;
				write_i2c(0x13, 0xe7); //AWB on
				write_i2c(0x6f, 0x9f); // Simple AWB
				*/

				int result = 0;
				String sresult = "";

				Serial.println(F("........... Setting Camera to Simple AWB ........"));

				// COM8
				//result = OV7670WriteReg(0x13, 0xE7);
				result = OV7670WriteReg(COM8, COM8_VALUE_AWB_ON);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM8(0x13): "));
				Serial.println(sresult);

				// AWBCTR0
				//result = OV7670WriteReg(0x6f, 0x9f);
				result = OV7670WriteReg(AWBCTR0, AWBCTR0_VALUE_NORMAL);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWBCTR0 Control Register 0(0x6F): "));
				Serial.println(sresult);
			}
			void SetupCameraAdvancedAutomaticWhiteBalance()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("........... Setting Camera to Advanced AWB ........"));

				// AGC, AWB, and AEC Enable
				result = OV7670WriteReg(0x13, 0xE7);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM8(0x13): "));
				Serial.println(sresult);

				// AWBCTR0 
				result = OV7670WriteReg(0x6f, 0x9E);
				sresult = ParseI2CResult(result);
				Serial.print(F("AWB Control Register 0(0x6F): "));
				Serial.println(sresult);
			}
			void SetupCameraGain()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("........... Setting Camera Gain ........"));

				// Set Maximum Gain
				//result = OV7670WriteReg(COM9, COM9_VALUE_MAX_GAIN_128X);
				result = OV7670WriteReg(COM9, COM9_VALUE_4XGAIN);
				//result = OV7670WriteReg(COM9, 0x18);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM9: "));
				Serial.println(sresult);

				// Set Blue Gain
				//{ REG_BLUE, 0x40 },
				result = OV7670WriteReg(BLUE, BLUE_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("BLUE GAIN: "));
				Serial.println(sresult);

				// Set Red Gain
				//{ REG_RED, 0x60 },
				result = OV7670WriteReg(RED, RED_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("RED GAIN: "));
				Serial.println(sresult);


				// Set Green Gain
				//{ 0x6a, 0x40 }, 
				result = OV7670WriteReg(GGAIN, GGAIN_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("GREEN GAIN: "));
				Serial.println(sresult);


				// Enable AWB Gain
				// REG_COM16	0x41	/* Control 16 */
				// COM16_AWBGAIN   0x08	  /* AWB gain enable */
				// { REG_COM16, COM16_AWBGAIN }, 
				result = OV7670WriteReg(COM16, COM16_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("COM16(ENABLE GAIN): "));
				Serial.println(sresult);

			}
			void SetCameraSaturationControl()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("........... Setting Camera Saturation Level ........"));
				result = OV7670WriteReg(SATCTR, SATCTR_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("SATCTR: "));
				Serial.println(sresult);
			}
			void SetCameraColorMatrixYUV()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("........... Setting Camera Color Matrix for YUV ........"));

				result = OV7670WriteReg(MTX1, MTX1_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("MTX1: "));
				Serial.println(sresult);

				result = OV7670WriteReg(MTX2, MTX2_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("MTX2: "));
				Serial.println(sresult);

				result = OV7670WriteReg(MTX3, MTX3_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("MTX3: "));
				Serial.println(sresult);

				result = OV7670WriteReg(MTX4, MTX4_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("MTX4: "));
				Serial.println(sresult);

				result = OV7670WriteReg(MTX5, MTX5_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("MTX5: "));
				Serial.println(sresult);

				result = OV7670WriteReg(MTX6, MTX6_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("MTX6: "));
				Serial.println(sresult);

				result = OV7670WriteReg(CONTRAS, CONTRAS_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("CONTRAS: "));
				Serial.println(sresult);

				result = OV7670WriteReg(MTXS, MTXS_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("MTXS: "));
				Serial.println(sresult);
			}
			void SetCameraFPSMode()
			{
				// Set FPS for Camera
				if (FPSParam == "ThirtyFPS")
				{
					SetupCameraFor30FPS();
				}
				else
					if (FPSParam == "NightMode")
					{
						SetupCameraNightMode();
					}
			}
			void SetCameraAEC()
			{
				// Process AEC
				if (AECParam == "AveAEC")
				{
					// Set Camera's Average AEC/AGC Parameters  
					SetupCameraAverageBasedAECAGC();
				}
				else
					if (AECParam == "HistAEC")
					{
						// Set Camera AEC algorithim to Histogram
						SetCameraHistogramBasedAECAGC();
					}
			}
			void SetupCameraAWB()
			{
				// Set AWB Mode
				if (AWBParam == "SAWB")
				{
					// Set Simple Automatic White Balance
					SetupCameraSimpleAutomaticWhiteBalance(); // OK

															  // Set Gain Config
					SetupCameraGain();
				}
				else
					if (AWBParam == "AAWB")
					{
						// Set Advanced Automatic White Balance
						SetupCameraAdvancedAutomaticWhiteBalance(); // ok

																	// Set Camera Automatic White Balance Configuration
						SetupCameraAdvancedAutoWhiteBalanceConfig(); // ok

																	 // Set Gain Config
						SetupCameraGain();
					}
			}
			void SetupCameraDenoise()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("........... Setting Camera Denoise  ........"));

				result = OV7670WriteReg(DNSTH, DNSTH_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("DNSTH: "));
				Serial.println(sresult);

				result = OV7670WriteReg(REG77, REG77_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("REG77: "));
				Serial.println(sresult);
			}
			void SetupCameraEdgeEnhancement()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("........... Setting Camera Edge Enhancement  ........"));

				result = OV7670WriteReg(EDGE, EDGE_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("EDGE: "));
				Serial.println(sresult);

				result = OV7670WriteReg(REG75, REG75_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("REG75: "));
				Serial.println(sresult);

				result = OV7670WriteReg(REG76, REG76_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("REG76: "));
				Serial.println(sresult);
			}
			void SetupCameraDenoiseEdgeEnhancement()
			{
				int result = 0;
				String sresult = "";

				if ((DenoiseParam == "DenoiseYes") &&
					(EdgeParam == "EdgeYes"))
				{
					SetupCameraDenoise();
					SetupCameraEdgeEnhancement();
					result = OV7670WriteReg(COM16, COM16_VALUE_DENOISE_ON__EDGE_ENHANCEMENT_ON__AWBGAIN_ON);
					sresult = ParseI2CResult(result);
					Serial.print(F("COM16: "));
					Serial.println(sresult);
				}
				else
					if ((DenoiseParam == "DenoiseYes") &&
						(EdgeParam == "EdgeNo"))
					{
						SetupCameraDenoise();
						result = OV7670WriteReg(COM16, COM16_VALUE_DENOISE_ON__EDGE_ENHANCEMENT_OFF__AWBGAIN_ON);
						sresult = ParseI2CResult(result);
						Serial.print(F("COM16: "));
						Serial.println(sresult);
					}
					else
						if ((DenoiseParam == "DenoiseNo") &&
							(EdgeParam == "EdgeYes"))
						{
							SetupCameraEdgeEnhancement();
							result = OV7670WriteReg(COM16, COM16_VALUE_DENOISE_OFF__EDGE_ENHANCEMENT_ON__AWBGAIN_ON);
							sresult = ParseI2CResult(result);
							Serial.print(F("COM16: "));
							Serial.println(sresult);
						}
			}
			void SetupCameraArrayControl()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("........... Setting Camera Array Control  ........"));

				result = OV7670WriteReg(CHLF, CHLF_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("CHLF: "));
				Serial.println(sresult);

				result = OV7670WriteReg(ARBLM, ARBLM_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("ARBLM: "));
				Serial.println(sresult);
			}
			void SetupCameraADCControl()
			{
				int result = 0;
				String sresult = "";

				Serial.println(F("........... Setting Camera ADC Control  ........"));

				result = OV7670WriteReg(ADCCTR1, ADCCTR1_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("ADCCTR1: "));
				Serial.println(sresult);

				result = OV7670WriteReg(ADCCTR2, ADCCTR2_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("ADCCTR2: "));
				Serial.println(sresult);

				result = OV7670WriteReg(ADC, ADC_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("ADC: "));
				Serial.println(sresult);

				result = OV7670WriteReg(ACOM, ACOM_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("ACOM: "));
				Serial.println(sresult);

				result = OV7670WriteReg(OFON, OFON_VALUE);
				sresult = ParseI2CResult(result);
				Serial.print(F("OFON: "));
				Serial.println(sresult);
			}

#endif // 0

#pragma endregion


	#pragma endregion
	
			
	//OV7670 cam = OV7670(11, 2, 12, 44, 45, 41, 3, 13);
	//OV7670 cam = OV7670(VSYNC, HREF, WEN, RRST, OE, XCLK, RESET, WRST);
	static void captureImg(uint16_t wg, uint16_t hg) {
		uint16_t y, x;

		Serial.println("*capturing...*");

		while (!(REG_PIOB_PDSR & (1 << 21)));//wait for high
		while ((REG_PIOB_PDSR & (1 << 21)));//wait for low

		y = hg;
		while (y--) {
			x = wg;
			while (x--) {
				while ((REG_PIOD_PDSR & (1 << 10)));//wait for low
				myImage[y][x] = (REG_PIOC_PDSR & 0xFF000) >> 6;
				while (!(REG_PIOD_PDSR & (1 << 10)));//wait for high
				while ((REG_PIOD_PDSR & (1 << 10)));//wait for low
				while (!(REG_PIOD_PDSR & (1 << 10)));//wait for high

				while ((REG_PIOD_PDSR & (1 << 10)));//wait for low
				myImage[y][x] |= (REG_PIOC_PDSR & 0xFF000) >> 12;
				while (!(REG_PIOD_PDSR & (1 << 10)));//wait for high
				while ((REG_PIOD_PDSR & (1 << 10)));//wait for low
				while (!(REG_PIOD_PDSR & (1 << 10)));//wait for high 	
			}
		}
		displayMyImage(240,320);
		Serial.println("*...end captureImg.");

	}
	static void captureImgTFT(uint16_t wg, uint16_t hg){
	  uint16_t y, x, px;

	  //Serial.println("*RDY*");

	  while (!(REG_PIOB_PDSR & (1 << 21)));//wait for high
	  while ((REG_PIOB_PDSR & (1 << 21)));//wait for low

	  y = hg;
	  while (y--){
		x = wg;
		while (x--){
		  while ((REG_PIOD_PDSR & (1 << 10)));//wait for low
	//      myImage[y][x] = (REG_PIOC_PDSR & 0xFF000) >> 12; 
		  px = (REG_PIOC_PDSR & 0xFF000) >> 12; 
	//	  uart_putchar(px);
		  tft.drawPixel(x, y, px);

		  while (!(REG_PIOD_PDSR & (1 << 10)));//wait for high
		  while ((REG_PIOD_PDSR & (1 << 10)));//wait for low
		  while (!(REG_PIOD_PDSR & (1 << 10)));//wait for high      
		}
	  }
  
	  //for (y = 0; y < hg; y++) {
	  //  for (x = 0; x < wg; x++) {
	  //    uart_putchar(myImage[y][x]);      
	  //  }
	  //}
	  Serial.print(F("Snap_done\r\n"));
	}

	static void captureImgAndDisplay(uint16_t wg, uint16_t hg) {
		uint16_t y, x, pixelColor;

		Serial.println("*RDY*");

		while (!(REG_PIOB_PDSR & (1 << 21)));//wait for high
		while ((REG_PIOB_PDSR & (1 << 21)));//wait for low

		y = hg;
		while (y--) {
			x = wg;
			while (x--) {
				while ((REG_PIOD_PDSR & (1 << 10)));//wait for low
				myImage[y][x] = (REG_PIOC_PDSR & 0xFF000) >> 12;

				//was myImage[y][x] = (REG_PIOC_PDSR & 0xFF000) >> 12;
				//is
				int temp = (REG_PIOC_PDSR & 0xFF000);
				pixelColor = ((temp & 0xFF) << 16) | ((temp & 0xFF) << 8) | (temp & 0xFF);
				//pixelColor =  (REG_PIOC_PDSR & 0xFF000) >> 12;
				tft.drawPixel(x, y, pixelColor);

				while (!(REG_PIOD_PDSR & (1 << 10)));//wait for high
				while ((REG_PIOD_PDSR & (1 << 10)));//wait for low
				while (!(REG_PIOD_PDSR & (1 << 10)));//wait for high      
			}
		}

		//for (y = 0; y < hg; y++) {
		//	for (x = 0; x < wg; x++) {
		//		pixelColor = (uint16_t)myImage[y][x];
		//		 
		//		 
		//		tft.drawPixel(x, y, pixelColor);
		//	}
		//}
	}

#pragma endregion

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // set the slaveSelectPin as an output:
  // Init cam
 #pragma region ILI9341_due tft  setup
  pinMode(TFT_DC, OUTPUT);

  // initialize SPI:
  SPI.begin();

  tft.begin();

  tft.setRotation(iliRotation90);
  tft.fillScreen(ILI9341_ORANGE);

  tft.setFont(Arial_bold_14);
  tft.setTextLetterSpacing(5);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
  tft.printAligned(F("TestOV7670due.ino "), gTextAlignMiddleCenter);

  //for (size_t x = PICT_OFFSET_X; x < PICT_OFFSET_X + PICT_SIZE_X; x++)
  //{
  //	for (size_t y = PICT_OFFSET_Y; y < PICT_OFFSET_Y + PICT_SIZE_Y; y++)
  //	{
  //		tft.drawPixel(x, y, ILI9341_RED);

  //	}
  //}


#pragma endregion
 //cam.Init();
 // cam.InitQQVGA();

#if 1

#pragma region 10.5MHz clock
  int32_t mask_PWM_pin = digitalPinToBitMask(7);
  REG_PMC_PCER1 = 1 << 4;				// activate clock for PWM controller
  REG_PIOC_PDR |= mask_PWM_pin;		// activate peripheral functions for pin (disables all PIO functionality)
  REG_PIOC_ABSR |= mask_PWM_pin;	// choose peripheral option B    
  REG_PWM_CLK = 0;					// choose clock rate, 0 -> full MCLK as reference 84MHz
  REG_PWM_CMR6 = 0 << 9;				// select clock and polarity for PWM channel (pin7) -> (CPOL = 0)
  REG_PWM_CPRD6 = 8; //was 8               // initialize PWM period -> T = value/84MHz (value: up to 16bit), value=8 -> 10.5MHz
  REG_PWM_CDTY6 = 4;                // initialize duty cycle, REG_PWM_CPRD6 / value = duty cycle, for 8/4 = 50%
  REG_PWM_ENA = 1 << 6;               // enable PWM on PWM channel (pin 7 = PWML6)
#pragma endregion
#endif

#if 0
#pragma region  8.4MHz clock
//#pragma region 21 MHz clock
//  int32_t mask_PWM_pin = digitalPinToBitMask(7);
//  REG_PMC_PCER1 = 1 << 4;				// activate clock for PWM controller
//  REG_PIOC_PDR |= mask_PWM_pin;		// activate peripheral functions for pin (disables all PIO functionality)
//  REG_PIOC_ABSR |= mask_PWM_pin;	// choose peripheral option B    
//  REG_PWM_CLK = 0;					// choose clock rate, 0 -> full MCLK as reference 84MHz
//  REG_PWM_CMR6 = 0 << 9;				// select clock and polarity for PWM channel (pin7) -> (CPOL = 0)
//  REG_PWM_CPRD6 = 6;                // initialize PWM period -> T = value/84MHz (value: up to 16bit), value=8 -> 10.5MHz
//  REG_PWM_CDTY6 = 3;                // initialize duty cycle, REG_PWM_CPRD6 / value = duty cycle, for 8/4 = 50%
//  REG_PWM_ENA = 1 << 6;               // enable PWM on PWM channel (pin 7 = PWML6)
//#pragma endregion

  int32_t mask_PWM_pin = digitalPinToBitMask(7);
  REG_PMC_PCER1 = 1 << 4;                         // activate clock for PWM controller
  REG_PIOC_PDR |= mask_PWM_pin;                   // activate peripheral functions for pin (disables all PIO functionality)
  REG_PIOC_ABSR |= mask_PWM_pin;                  // choose peripheral option B
  REG_PWM_CLK = 0;                                // choose clock rate, 0 -> full MCLK as reference 84MHz
  REG_PWM_CMR6 = 0 << 9;                          // select clock and polarity for PWM channel (pin7) -> (CPOL = 0)
  REG_PWM_CPRD6 = 10;                             // initialize PWM period -> T = value/84MHz (value: up to 16bit), value=10 -> 8.4MHz
  REG_PWM_CDTY6 = 5;                              // initialize duty cycle, REG_PWM_CPRD6 / value = duty cycle, for 10/5 = 50%
  REG_PWM_ENA = 1 << 6;                           // enable PWM on PWM channel (pin 7 = PWML6)

#pragma endregion

#endif // 0


  camInit();//was camInit();
  //setRes();
  //setColor();
  SetupOV7670ForQVGArgb565();
  //captureImg(320, 240);

//  write(0x11, 6);//CLKRC 110= set internal clock prescaler divided by 7
}

#define STRIDE  32

#if 0
void loop1() {


	int i, j, k;
	uint16_t pix1, pix2, pix, npix;
	uint16_t p[STRIDE];
	int time;

	uint8_t rotback = tft.getRotation();

	cam.ReadStart();
	for (i = 0; i < 120; i++) {
		for (j = 0; j < 160; j += STRIDE) {
			for (k = 0; k < STRIDE; k++) {
				pix1 = cam.ReadOneByte();
				pix2 = cam.ReadOneByte();
				p[k] = (pix2 << 8) | pix1;
				//if ((i == mouse_y) && ((j + k) == mouse_x)) {
				//	mouse_pix = p[k];
				//}
			}
			tft.pushColors(p, 0, (int)STRIDE);
			//tft.drawPixel(i, j, npix);
			//Serial.print(cam.ReadOneByte(),HEX);
		}
	}
	cam.ReadStop();


}

#endif // 0

// visualizza la matrice myImage[y][x] sul display
void displayMyImage(uint16_t wg, uint16_t hg) {
	Serial.println("Displaying to TFT...");
	for (int y = 0; y < hg; y++) {
		for (int x = 0; x < wg; x++) {
			tft.drawPixel(x, y, myImage[y][x]);//uart_putchar(myImage[y][x]);
		}
	}

}
void loop() {
	digitalWrite(13, !digitalRead(13)); //toggle LED
	
	ReadTransmitTFTCapturedFrame();//was captureImg(320, 240);
}								   //Camera
 
void loop0() {     
	// Wait for Command
     Serial.println(F("Ready to Accept new Command => "));
     while (1)
     {
       if (Serial.available() > 0) 
       {
         int NumberCharsRead = Serial.readBytesUntil('\n', IncomingByte, BUFFERLENGTH);  
         for (int i = 0; i < NumberCharsRead; i++)
         {
           RawCommandLine += IncomingByte[i];
         } 
         break;
       }
     }  

	 // command parsing-------------------
	 if (RawCommandLine == "snap") 
	 {
		 captureImg(320,240);
		 memset(&RawCommandLine, 0, sizeof(RawCommandLine));

	 }
	 else
 
			if (RawCommandLine == "init_bw_VGA")
			{
				format = 'b';
				resolution = VGA;
 
				Serial.print(F("NOT AVAILABLE\r\n"));
				  

				memset(&RawCommandLine, 0, sizeof(RawCommandLine));
			}
		 else
			 if (RawCommandLine == "init_yuv_QVGA")
			 {
				format = 'y';
				resolution = QVGA;
				SetupOV7670ForQVGAYUV();
				//if (camera.Init('b', QVGA) != 1)
				//{
				//	Serial.print(F("Init Fail\r\n"));
				//}
				Serial.print(F("Initializing done\r\n"));
				memset(&RawCommandLine, 0, sizeof(RawCommandLine));
			 }
		 else
			 if (RawCommandLine == "init_rgb_QVGA")
			 {
					format = 'r';
					resolution = QVGA;
					SetupOV7670ForQVGArgb565();
					//if (camera.Init('r', QVGA) != 1)
					//{
					//	Serial.print(F("Init Fail\r\n"));
					//}
					Serial.print(F("Initializing done\r\n"));
					memset(&RawCommandLine, 0, sizeof(RawCommandLine));
			 }
		 else
			 if (RawCommandLine == "init_bw_QVGA")
			 {
						format = 'b';
						resolution = QVGA;

							Serial.print(F("Init Fail\r\n"));

						//Serial.print(F("Initializing done\r\n"));
						memset(&RawCommandLine, 0, sizeof(RawCommandLine));
			 }
		 else
			 if (RawCommandLine == "init_yuv_QQVGA")
			 {
							format = 'y';
							resolution = QQVGA;
							SetupOV7670ForQQVGAYUV();
							//if (camera.Init('b', QQVGA) != 1)
							//{
							//	Serial.print(F("Init Fail\r\n"));
							//}
							Serial.print(F("Initializing done\r\n"));
							memset(&RawCommandLine, 0, sizeof(RawCommandLine));
			 }
		 else
			 if (RawCommandLine == "init_rgb_QQVGA")
			 {
				format = 'r';
				resolution = QQVGA;

				Serial.print(F("NOT AVAILABLE\r\n"));
				memset(&RawCommandLine, 0, sizeof(RawCommandLine));
			 }
		 else
			 if (RawCommandLine == "init_bw_QQVGA")
			 {
				format = 'b';
				resolution = QQVGA;
				Serial.print(F("Init Fail\r\n"));
				//Serial.print(F("Initializing done\r\n"));
				memset(&RawCommandLine, 0, sizeof(RawCommandLine));
			 }
		 else
			 if (RawCommandLine == "reset")
			 {
				//mbed_reset();
				 ResetCameraRegisters();

			 }
		 else
			 if (RawCommandLine == "time")
			 {
				 Serial.print(F("not avail.: "));

				memset(&RawCommandLine, 0, sizeof(RawCommandLine));
			 }
		 else
			 if (RawCommandLine == "reg_status")
			 {
				 ReadRegisters();

					//int i = 0;
					//Serial.print(F("AD : +0 +1 +2 +3 +4 +5 +6 +7 +8 +9 +A +B +C +D +E +F");
					//for (i = 0; i<OV7670_REGMAX; i++)
					//{
					//	int data;
					//	data = camera.ReadReg(i); // READ REG
					//	if ((i & 0x0F) == 0)
					//	{
					//		Serial.print(F("\r\n%02X : ", i);
					//	}
					//	Serial.print(F("%02X ", data);
					//}
					//Serial.print(F("\r\n"));
			 }

			 memset(&RawCommandLine, 0, sizeof(RawCommandLine));

}

 



void write(uint8_t regID, byte regDat){
	OV7670Write(regID, &regDat, 1);
}

void write_Orig(uint8_t regID, byte regDat){
  Wire.beginTransmission(address);
  Wire.write(regID & 0x00FF);    
  Wire.write(regDat & 0x00FF);    
    
  if(Wire.endTransmission(true)){
    Serial.print("error write reg ");
    Serial.println(regID);
  }
  delay(20);
}
#pragma region adattati da OV7670 FIFO

#define OV7670_I2C_WRITE_ADDRESS                 (0x42>>1)
#define OV7670_I2C_READ_ADDRESS                  (0x43>>1)
#define I2C_ERROR_WRITING_START_ADDRESS      11
#define I2C_ERROR_WRITING_DATA               22

int OV7670Write(int start, const byte *pData, int size)
{
	int n, error;

	Wire.beginTransmission(OV7670_I2C_WRITE_ADDRESS);
	n = Wire.write(start);        // write the start address
	if (n != 1)
	{
		return (I2C_ERROR_WRITING_START_ADDRESS);
	}

	n = Wire.write(pData, size);  // write data bytes
	if (n != size)
	{
		return (I2C_ERROR_WRITING_DATA);
	}

	error = Wire.endTransmission(true); // release the I2C-bus
	if (error != 0)
	{
		return (error);
	}

	return 0;         // return : no error
}
//
// A function to write a single register
//
int OV7670WriteReg(int reg, byte data)
{
	int error;

	error = OV7670Write(reg, &data, 1);

	return (error);
}
void PulseLowEnabledPin(int PinNumber, int DurationMicroSecs)
{
	// For Low Enabled Pins , 0 = on and 1 = off
	digitalWrite(PinNumber, LOW);            // Sets the pin on
	delayMicroseconds(DurationMicroSecs);    // Pauses for DurationMicroSecs microseconds      

	digitalWrite(PinNumber, HIGH);            // Sets the pin off
	delayMicroseconds(DurationMicroSecs);     // Pauses for DurationMicroSecs microseconds  
}
void PulsePin(int PinNumber, int DurationMicroSecs)
{
	digitalWrite(PinNumber, HIGH);           // Sets the pin on
	delayMicroseconds(DurationMicroSecs);    // Pauses for DurationMicroSecs microseconds      

	digitalWrite(PinNumber, LOW);            // Sets the pin off
	delayMicroseconds(DurationMicroSecs);    // Pauses for DurationMicroSecs microseconds  
}
// Converts pin HIGH/LOW values on pins at positions 0-7 to a corresponding byte value
byte ConvertPinValueToByteValue(int PinValue, int PinPosition)
{
	byte ByteValue = 0;
	if (PinValue == HIGH)
	{
		ByteValue = 1 << PinPosition;
	}

	return ByteValue;
}
uint16_t ConvertPinValueTo16bitValue(int PinValue, int PinPosition)
{
	uint16_t value = 0;
	if (PinValue == HIGH)
	{
		value = 1 << PinPosition;
	}

	return value;
}
//void CaptureOV7670Frame()
//{
//	unsigned long DurationStart = 0;
//	unsigned long DurationStop = 0;
//	unsigned long TimeForCaptureStart = 0;
//	unsigned long TimeForCaptureEnd = 0;
//	unsigned long ElapsedTime = 0;
//
//	//Capture one frame into FIFO memory
//
//	// 0. Initialization. 
//	Serial.println();
//	Serial.println(F("Starting Capture of Photo ..."));
//	TimeForCaptureStart = millis();
//
//	// 1. Wait for VSync to pulse to indicate the start of the image
//	DurationStart = pulseIn(VSYNC, LOW);
//
//	// 2. Reset Write Pointer to 0. Which is the beginning of frame
//	PulseLowEnabledPin(WRST, 6); // 3 microseconds + 3 microseconds for error factor on Arduino
//
//								 // 3. Set FIFO Write Enable to active (high) so that image can be written to ram
//	digitalWrite(WEN, HIGH);
//
//	// 4. Wait for VSync to pulse again to indicate the end of the frame capture
//	DurationStop = pulseIn(VSYNC, LOW);
//
//	// 5. Set FIFO Write Enable to nonactive (low) so that no more images can be written to the ram
//	digitalWrite(WEN, LOW);
//
//	// 6. Print out Stats
//	TimeForCaptureEnd = millis();
//	ElapsedTime = TimeForCaptureEnd - TimeForCaptureStart;
//
//	Serial.print(F("Time for Frame Capture (milliseconds) = "));
//	Serial.println(ElapsedTime);
//
//	Serial.print(F("VSync beginning duration (microseconds) = "));
//	Serial.println(DurationStart);
//
//	Serial.print(F("VSync end duration (microseconds) = "));
//	Serial.println(DurationStop);
//
//	// 7. WAIT so that new data can appear on output pins Read new data.
//	delay(2);
//}
void ReadTransmitCapturedFrame()
{
	// Read captured frame from FIFO memory and send each byte as it is read to the Android controller
	// via bluetooth.

	// Set Output Enable OE to active (low).
	// * Make sure to connect the OE output to ground.

	// Reset the FIFO read pointer by setting RRST to active (low) then bringing it back to high.
	// *Done from previous CaptureOV7670Frame() assuming WRST and RRST are tied together.

	// Read in the QQVGA image that is captured in the camera buffer by reading in the 38400 bytes that make up the 
	//   YUV photo 

	byte PixelData = 0;
	byte PinVal7 = 0;
	byte PinVal6 = 0;
	byte PinVal5 = 0;
	byte PinVal4 = 0;
	byte PinVal3 = 0;
	byte PinVal2 = 0;
	byte PinVal1 = 0;
	byte PinVal0 = 0;

	Serial.println(F("Starting Transmission of Photo ..."));



	// Set Read Buffer Pointer to start of frame
	digitalWrite(RRST, LOW);
	PulsePin(PCLK, 1);
	PulsePin(PCLK, 1);
	PulsePin(PCLK, 1);
	digitalWrite(RRST, HIGH);

	unsigned long  ByteCounter = 0;
	for (int height = 0; height < PHOTO_HEIGHT; height++)
	{
		for (int width = 0; width < PHOTO_WIDTH; width++)
		{
			for (int bytenumber = 0; bytenumber < PHOTO_BYTES_PER_PIXEL; bytenumber++)
			{
				// Pulse the read clock RCLK to bring in new byte of data.
				// 7ns for RCLK High Pulse Width and Low Pulse Width .007 micro secs
				PulsePin(PCLK, 1);

				// Convert Pin values to byte values for pins 0-7 of incoming pixel byte
				PinVal7 = ConvertPinValueToByteValue(digitalRead(DO7), 7);
				PinVal6 = ConvertPinValueToByteValue(digitalRead(DO6), 6);
				PinVal5 = ConvertPinValueToByteValue(digitalRead(DO5), 5);
				PinVal4 = ConvertPinValueToByteValue(digitalRead(DO4), 4);
				PinVal3 = ConvertPinValueToByteValue(digitalRead(DO3), 3);
				PinVal2 = ConvertPinValueToByteValue(digitalRead(DO2), 2);
				PinVal1 = ConvertPinValueToByteValue(digitalRead(DO1), 1);
				PinVal0 = ConvertPinValueToByteValue(digitalRead(DO0), 0);

				// Combine individual data from each pin into composite data in the form of a single byte
				PixelData = PinVal7 | PinVal6 | PinVal5 | PinVal4 | PinVal3 | PinVal2 | PinVal1 | PinVal0;
				uart_putchar(PixelData);
				/////////////////////////////  SD Card ////////////////////////////////
				//ByteCounter = ByteCounter + ImageOutputFile.write(PixelData);
				///////////////////////////////////////////////////////////////////////
			}
		}
	}

}
void ReadTransmitTFTCapturedFrame()
{
	// Read captured frame from FIFO memory and send each byte as it is read to the Android controller
	// via bluetooth.

	// Set Output Enable OE to active (low).
	// * Make sure to connect the OE output to ground.

	// Reset the FIFO read pointer by setting RRST to active (low) then bringing it back to high.
	// *Done from previous CaptureOV7670Frame() assuming WRST and RRST are tied together.

	// Read in the QQVGA image that is captured in the camera buffer by reading in the 38400 bytes that make up the 
	//   YUV photo 

	byte PixelData = 0;
	byte PinVal7 = 0;
	byte PinVal6 = 0;
	byte PinVal5 = 0;
	byte PinVal4 = 0;
	byte PinVal3 = 0;
	byte PinVal2 = 0;
	byte PinVal1 = 0;
	byte PinVal0 = 0;

	Serial.println(F("Acquiring from camera ..."));



	// Set Read Buffer Pointer to start of frame
	digitalWrite(RRST, LOW);
	PulsePin(XCLK, 1); 
	PulsePin(XCLK, 1); 
	PulsePin(XCLK, 1); 
	digitalWrite(RRST, HIGH); 
	uint16_t color = 0;

	unsigned long  ByteCounter = 0;
	for (int y = 0; y < 240; y++)
	{
		//Serial.println(y);
		for (int x = 0; x < 320; x++)
		{


				// Pulse the read clock RCLK to bring in new byte of data.
				// 7ns for RCLK High Pulse Width and Low Pulse Width .007 micro secs
				PulsePin(XCLK, 1);

				// Convert Pin values to byte values for pins 0-7 of incoming pixel byte
				color = ConvertPinValueTo16bitValue(digitalRead(DO7), 15) | ConvertPinValueTo16bitValue(digitalRead(DO6), 14) | ConvertPinValueTo16bitValue(digitalRead(DO5), 13) | ConvertPinValueTo16bitValue(digitalRead(DO4), 12) | ConvertPinValueTo16bitValue(digitalRead(DO3), 11) | ConvertPinValueTo16bitValue(digitalRead(DO2), 10) | ConvertPinValueTo16bitValue(digitalRead(DO1), 9) | ConvertPinValueTo16bitValue(digitalRead(DO0), 8);

 				PulsePin(XCLK, 1);

				// Convert Pin values to byte values for pins 0-7 of incoming pixel byte
				color |= ConvertPinValueTo16bitValue(digitalRead(DO7), 7) | ConvertPinValueTo16bitValue(digitalRead(DO6), 6) | ConvertPinValueTo16bitValue(digitalRead(DO5), 5) | ConvertPinValueTo16bitValue(digitalRead(DO4), 4) | ConvertPinValueTo16bitValue(digitalRead(DO3), 3) | ConvertPinValueTo16bitValue(digitalRead(DO2), 2) | ConvertPinValueTo16bitValue(digitalRead(DO1), 1) | ConvertPinValueTo16bitValue(digitalRead(DO0), 0);


				//uart_putchar(PixelData);
				//myImage[y][x] = PixelData;
				tft.drawPixel(x,y,color);
 
		}
	}
	//displayMyImage(240, 320);

}

#pragma endregion

static inline int uart_putchar(const uint8_t c) {
    while(!(UART->UART_SR & UART_SR_TXRDY));
    UART->UART_THR = c;
    return 0;
}
byte ReadRegisterValue(int RegisterAddress)
{
	byte data = 0;

	Wire.beginTransmission(OV7670_I2C_READ_ADDRESS);
	Wire.write(RegisterAddress);
	Wire.endTransmission();
	Wire.requestFrom(OV7670_I2C_READ_ADDRESS, 1);
	while (Wire.available() < 1);
	data = Wire.read();

	return data;
}
String ParseI2CResult(int result)
{
	String sresult = "";
	switch (result)
	{
	case 0:
		sresult = "I2C Operation OK ...";
		break;

	case  I2C_ERROR_WRITING_START_ADDRESS:
		sresult = "I2C_ERROR_WRITING_START_ADDRESS";
		break;

	case I2C_ERROR_WRITING_DATA:
		sresult = "I2C_ERROR_WRITING_DATA";
		break;

	case DATA_TOO_LONG:
		sresult = "DATA_TOO_LONG";
		break;

	case NACK_ON_TRANSMIT_OF_ADDRESS:
		sresult = "NACK_ON_TRANSMIT_OF_ADDRESS";
		break;

	case NACK_ON_TRANSMIT_OF_DATA:
		sresult = "NACK_ON_TRANSMIT_OF_DATA";
		break;

	case OTHER_ERROR:
		sresult = "OTHER_ERROR";
		break;

	default:
		sresult = "I2C ERROR TYPE NOT FOUND...";
		break;
	}

	return sresult;
}

void ResetCameraRegisters()
{
	// Reset Camera Registers
	// Reading needed to prevent error
	byte data = ReadRegisterValue(COM7);

	int result = OV7670WriteReg(COM7, COM7_VALUE_RESET);
	String sresult = ParseI2CResult(result);
	Serial.println("RESETTING ALL REGISTERS BY SETTING COM7 REGISTER to 0x80: " + sresult);

	// Delay at least 500ms 
	delay(500);
}
 
