#include "mbed.h"
#include "ov7670reg.h"

#define OV7670_WRITE (0x42)
#define OV7670_READ  (0x43)
#define OV7670_WRITEWAIT (20)
#define OV7670_NOACK (0)
#define OV7670_REGMAX (201)
#define OV7670_I2CFREQ (50000)

//
// OV7670 + FIFO AL422B camera board test
//
class OV7670 : public Base
{
public:
    I2C camera ;
    InterruptIn vsync,href;
    DigitalOut wen ;
    BusIn data ;
    DigitalOut rrst,oe,rclk ;
    volatile int LineCounter ;
    volatile int LastLines ;
    volatile bool CaptureReq ;
    volatile bool Busy ;
    volatile bool Done ;

    OV7670(
        PinName sda,// Camera I2C port
        PinName scl,// Camera I2C port
        PinName vs, // VSYNC
        PinName hr, // HREF
        PinName we, // WEN
        PinName d7, // D7
        PinName d6, // D6
        PinName d5, // D5
        PinName d4, // D4
        PinName d3, // D3
        PinName d2, // D2
        PinName d1, // D1
        PinName d0, // D0
        PinName rt, // /RRST
        PinName o,  // /OE
        PinName rc  // RCLK      
        ) : camera(sda,scl),vsync(vs),href(hr),wen(we),data(d0,d1,d2,d3,d4,d5,d6,d7),rrst(rt),oe(o),rclk(rc)
    {
        camera.stop() ;
        camera.frequency(OV7670_I2CFREQ) ;
        vsync.fall(this,&OV7670::VsyncHandler) ;
        href.rise(this,&OV7670::HrefHandler) ;
        CaptureReq = false ;
        Busy = false ;
        Done = false ;
        LineCounter = 0 ;
        rrst = 1 ;
        oe = 1 ;
        rclk = 1 ;
        wen = 0 ;
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
        camera.start() ;
        camera.write(OV7670_WRITE) ;
        wait_us(OV7670_WRITEWAIT);
        camera.write(addr) ;
        wait_us(OV7670_WRITEWAIT);
        camera.write(data) ;
        camera.stop() ;
    }

    // read from camera
    int ReadReg(int addr)
    {
        int data ;

        // WRITE 0x42,ADDR
        camera.start() ;
        camera.write(OV7670_WRITE) ;
        wait_us(OV7670_WRITEWAIT);
        camera.write(addr) ;
        camera.stop() ;
        wait_us(OV7670_WRITEWAIT);    

        // WRITE 0x43,READ
        camera.start() ;
        camera.write(OV7670_READ) ;
        wait_us(OV7670_WRITEWAIT);
        data = camera.read(OV7670_NOACK) ;
        camera.stop() ;
    
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
        WriteReg(REG_COM7,0x04) ;
        WriteReg(REG_RGB444, 0x02);
        WriteReg(REG_COM15, 0xd0);
        WriteReg(REG_HSTART,0x16) ;
        WriteReg(REG_HSTOP,0x04) ;
        WriteReg(REG_HREF,0x24) ;
        WriteReg(REG_VSTART,0x02) ;
        WriteReg(REG_VSTOP,0x7a) ;
        WriteReg(REG_VREF,0x0a) ;
        WriteReg(REG_COM10,0x02) ;
        WriteReg(REG_COM3, 0x04);
        WriteReg(REG_COM14, 0x1a);
        WriteReg(0x72, 0x22);
        WriteReg(0x73, 0xf2);

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
    }    



    // vsync handler
    void VsyncHandler(void)
    {
        // Capture Enable
        if (CaptureReq) {
            wen = 1 ;
            Done = false ;
            CaptureReq = false ;
        } else {
            wen = 0 ;
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
        int result ;
        rclk = 1 ;
//        wait_us(1) ;
        result = data ;
        rclk = 0 ;
        return result ;
    }
    
    // Data Start
    void ReadStart(void)
    {        
        rrst = 0 ;
        oe = 0 ;
        wait_us(1) ;
        rclk = 0 ;
        wait_us(1) ;
        rclk = 1 ;
        wait_us(1) ;        
        rrst = 1 ;
    }
    
    // Data Stop
    void ReadStop(void)
    {
        oe = 1 ;
        ReadOneByte() ;
        rclk = 1 ;
    }
};
