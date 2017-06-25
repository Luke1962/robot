#include "ov7670reg.h"
#include <Wire.h>

#define OV7670_WRITE (0x42)
#define OV7670_READ  (0x43)
#define OV7670_WRITEWAIT (20)
#define OV7670_NOACK (0)
#define OV7670_REGMAX (201)
#define OV7670_I2CFREQ (50000)

#define VSYNC_BIT	(1<<7)
#define PIOD_PDSR	0x400E143C
#define PIOC_PDSR	0x400E123C

#define WHILE_VSYNC_H	while( (*(int *)PIOD_PDSR)&VSYNC_BIT);
#define WHILE_VSYNC_L	while(!((*(int *)PIOD_PDSR)&VSYNC_BIT));

#define CLOCK_HIGH *(int *)0x400E1230=1<<9
#define CLOCK_LOW   *(int *)0x400E1234=1<<9


//
// OV7670 + FIFO AL422B camera board test
//
class OV7670 /*: public Base*/
{
public:
    int vsync,href;
    int wen ;
    //BusIn data ;
   // PortIn data;
    int rrst,oe,rclk,reset,vsc,wrst ;
    volatile int LineCounter ;
    volatile int LastLines ;
    volatile bool CaptureReq ;
    volatile bool Busy ;
    volatile bool Done ;    
        
    OV7670(
        int vs, // VSYNC
        int hr, // HREF
        int we, // WEN
        int rt, // /RRST
        int o,  // /OE
        int rc,  // RCLK 
		int rs, // RESET
		int wt // WRST
        ) 
    {
		vsc = vs;
		href =  hr;
		wen = we;
		rrst = rt;
		oe = o;
		rclk = rc;;
		reset = rs;
		wrst = wt;
	}
	
	void Init()
	{
		Serial.println("OV7670 Init");
		Wire.begin();

        Wire.endTransmission();
        CaptureReq = false ;
        Busy = false ;
        Done = false ;
        LineCounter = 0 ;
		pinMode(rrst, OUTPUT);
		pinMode(oe, OUTPUT);
		pinMode(rclk, OUTPUT);
		pinMode(wen,OUTPUT);
		pinMode(reset,OUTPUT);
		pinMode(vsc, INPUT);
		pinMode(wrst, OUTPUT);
	
		// Fifo Data pins: PC1:PC8
		pinMode(33,INPUT);
		pinMode(34,INPUT);
		pinMode(35,INPUT);
		pinMode(36,INPUT);
		pinMode(37,INPUT);
		pinMode(38,INPUT);
		pinMode(39,INPUT);
		pinMode(40,INPUT);
	
		digitalWrite(rrst,HIGH);
		digitalWrite(wrst,HIGH);
		digitalWrite(oe,LOW); // Output is always enable
		digitalWrite(rclk,HIGH);
		digitalWrite(wen,LOW);
		digitalWrite(reset,HIGH);
		Serial.println("OV7670 leaving Init");
		Reset();
     }

    // capture request
    void CaptureNext(void)
    {
        CaptureReq = true ;
        Busy = true ;
    }
    
    // capture done? (with clear)
    bool CaptureDone(void)
    {
        bool result ;
        if (Busy) {
            result = false ;
        } else {
            result = Done ;
            Done = false ;
        }
        return result ;
    }

    // write to camera
    void WriteReg(int addr,int data)
    {
        // WRITE 0x42,ADDR,DATA
        Wire.beginTransmission(OV7670_WRITE>>1);
        //Wire.write(OV7670_WRITE>>1) ;
        //wait_us(OV7670_WRITEWAIT);
        Wire.write(addr) ;
        //wait_us(OV7670_WRITEWAIT);
        Wire.write(data) ;
        Wire.endTransmission() ;
    }

    // read from camera
    int ReadReg(int addr)
    {
        int data ;

        // WRITE 0x42,ADDR
		Wire.beginTransmission(OV7670_WRITE>>1);    
		//wait_us(OV7670_WRITEWAIT);
        Wire.write(addr) ;
        Wire.endTransmission();
        //wait_us(OV7670_WRITEWAIT);    

        // WRITE 0x43,READ
     	Wire.beginTransmission(OV7670_READ>>1);    
	    //camera.write(OV7670_READ) ;
        //wait_us(OV7670_WRITEWAIT);
        data = Wire.read() ;
        Wire.endTransmission();
    
        return data ;
    }

    void Reset(void) {    
        WriteReg(0x12,0x80) ; // RESET CAMERA
        wait_ms(200) ;
    }
    
    void InitQQVGA() {
        // QQVGA RGB444
        WriteReg(REG_CLKRC,0x80);
        WriteReg(REG_COM11,0x0A) ;
        WriteReg(REG_TSLB,0x04);
        WriteReg(REG_COM7,0x04) ; /* output  color bar */
        
        //WriteReg(REG_RGB444, 0x02);
        //WriteReg(REG_COM15, 0xd0);
        WriteReg(REG_RGB444, 0x00);     // Disable RGB 444?
        WriteReg(REG_COM15, 0xD0);      // Set RGB 565?
        
        WriteReg(REG_HSTART,0x16) ;
        WriteReg(REG_HSTOP,0x04) ;
        WriteReg(REG_HREF,0x24) ;
        WriteReg(REG_VSTART,0x02) ;
        WriteReg(REG_VSTOP,0x7a) ;
        WriteReg(REG_VREF,0x0a) ;
        WriteReg(REG_COM10,0x02) ;
        WriteReg(REG_COM3, 0x04);
        WriteReg(REG_COM14, 0x1a);
        WriteReg(REG_MVFP,0x27) ;
        WriteReg(0x72, 0x22);
        WriteReg(0x73, 0xf2);
#if 1
        // COLOR SETTING
        WriteReg(0x4f,0x80);
        WriteReg(0x50,0x80);
        WriteReg(0x51,0x00);
        WriteReg(0x52,0x22);
        WriteReg(0x53,0x5e);
        WriteReg(0x54,0x80);
        WriteReg(0x56,0x40);
        WriteReg(0x58,0x9e);
        WriteReg(0x59,0x88);
        WriteReg(0x5a,0x88);
        WriteReg(0x5b,0x44);
        WriteReg(0x5c,0x67);
        WriteReg(0x5d,0x49);
        WriteReg(0x5e,0x0e);
        WriteReg(0x69,0x00);
        WriteReg(0x6a,0x40);
        WriteReg(0x6b,0x0a);
        WriteReg(0x6c,0x0a);
        WriteReg(0x6d,0x55);
        WriteReg(0x6e,0x11);
        WriteReg(0x6f,0x9f);

        WriteReg(0xb0,0x84);
#endif		
    }    



    // vsync handler
    void VsyncHandler(void)
    {
        // Capture Enable
        if (CaptureReq) {
            digitalWrite(wen,HIGH);
            Done = false ;
            CaptureReq = false ;
        } else {
            digitalWrite(wen,LOW) ;
            if (Busy) {
                Busy = false ;
                Done = true ;
            }
        }

        // Hline Counter
        LastLines = LineCounter ;
        LineCounter = 0 ;
    }
    
    // href handler
    void HrefHandler(void)
    {
        LineCounter++ ;
    }
    
    // Data Read
    int ReadOneByte(void)
    {
        int result;
	    //digitalWrite(rclk, HIGH);
		CLOCK_HIGH;
	
		HrefHandler(); // Dummy call for delay
		HrefHandler(); // Dummy call for delay
		HrefHandler(); // Dummy call for delay
		HrefHandler(); // Dummy call for delay
		HrefHandler(); // Dummy call for delay
		HrefHandler(); // Dummy call for delay
		HrefHandler(); // Dummy call for delay
		HrefHandler(); // Dummy call for delay
		//HrefHandler(); // Dummy call for delay
		//HrefHandler(); // Dummy call for delay
		//HrefHandler(); // Dummy call for delay
	
	     result = *(int *) PIOC_PDSR;
		 //result = *(int *) PIOC_PDSR;
		result = (result>>1)&0x0ff;

        //digitalWrite(rclk,LOW);
		CLOCK_LOW;
     	//i=0; //min:1
		//while(i--)
			//HrefHandler(); // Dummy call for delay
		return result;
    }
    
    // Data Start
    void ReadStart(void)
    {        
        digitalWrite(rrst,LOW);
        digitalWrite(oe,LOW);
        //wait_us(1) ;
        digitalWrite(rclk,LOW);
        //wait_us(1) ;
        digitalWrite(rclk, HIGH);
        //wait_us(1) ;        
        digitalWrite(rrst, HIGH);
    }
    
    // Data Stop
    void ReadStop(void)
    {
        digitalWrite(oe,HIGH);
        ReadOneByte() ;
        digitalWrite(rclk,HIGH);
    }
	
	void wait_ms(int ms)
	{
		int i;
		int l = 1000 * ms;
		while(l--);
	}
	
	int vsync_period()
	{
		int time;
		//while(digitalRead(vsc)==HIGH);
		WHILE_VSYNC_H;
		time = millis();
		WHILE_VSYNC_L;
		WHILE_VSYNC_H;
		return  millis()-time;	
	}
	
	int capture_frame()
	{
		int time;
		WHILE_VSYNC_H; // Wait for Vsync
		time=millis();
		//Disable FIFO write
		digitalWrite(wen,LOW);
		//Reset FIFO write and read pointer
		digitalWrite(wrst,LOW); 
		digitalWrite(wrst,HIGH);
		
		digitalWrite(rrst,LOW); // Probable
		digitalWrite(rrst,HIGH); // Probable
		//Enable FIFO write
		digitalWrite(wen,HIGH);//Verified
		WHILE_VSYNC_L; //Wait for Vsync becomes High again
		WHILE_VSYNC_H;
		//Disable FIFO write
		digitalWrite(wen,LOW); // Verified
		return millis()-time;
	}

};

