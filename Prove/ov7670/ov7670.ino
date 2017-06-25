//
// OV7670 + FIFO AL422B camera board test
//
#include "mbed.h"
#include "ov7670.h"

OV7670 camera(
    p28,p27,       // SDA,SCL(I2C / SCCB)
    p21,p22,p20,   // VSYNC,HREF,WEN(FIFO)
    p19,p18,p17,p16,p15,p14,p13,p12, // D7-D0
    p23,p24,p25) ; // RRST,OE,RCLK

Serial pc(USBTX,USBRX) ;

#define SIZEX (160)
#define SIZEY (120)

int main() {
    int i ;
    pc.baud(115200) ;
    pc.printf("Camera resetting..\r\n") ;

    camera.Reset() ;

    pc.printf("Before Init...\r\n") ;    
    pc.printf("AD : +0 +1 +2 +3 +4 +5 +6 +7 +8 +9 +A +B +C +D +E +F") ;
    for (i=0;i<OV7670_REGMAX;i++) {
        int data ;
        data = camera.ReadReg(i) ; // READ REG
        if ((i & 0x0F) == 0) {
            pc.printf("\r\n%02X : ",i) ;
        }
        pc.printf("%02X ",data) ;
    }
    pc.printf("\r\n") ;

    camera.InitQQVGA() ;

    pc.printf("After Init...\r\n") ;    
    pc.printf("AD : +0 +1 +2 +3 +4 +5 +6 +7 +8 +9 +A +B +C +D +E +F") ;
    for (i=0;i<OV7670_REGMAX;i++) {
        int data ;
        data = camera.ReadReg(i) ; // READ REG
        if ((i & 0x0F) == 0) {
            pc.printf("\r\n%02X : ",i) ;
        }
        pc.printf("%02X ",data) ;
    }
    pc.printf("\r\n") ;
    
    // CAPTURE and SEND LOOP
    while(1)
    {
        pc.printf("Hit Any Key to send RGBx160x120 Capture Data.\r\n") ;
        while(!pc.readable()) ;
        pc.getc() ;
        camera.CaptureNext() ;
        while(camera.CaptureDone() == false) ;
        pc.printf("*\r\n") ;
        camera.ReadStart() ;
        i = 0 ;
        for (int y = 0;y < SIZEY;y++) {
            int r,g,b,d1,d2 ;
            for (int x = 0;x < SIZEX;x++) {
                d1 = camera.ReadOneByte() ; // upper nibble is XXX , lower nibble is B
                d2 = camera.ReadOneByte() ; // upper nibble is G   , lower nibble is R
                b = (d1 & 0x0F) ;
                g = (d2 & 0xF0) >> 4 ;
                r = (d2 & 0x0F) ;
                pc.printf ("%1X%1X%1X",r,g,b) ;
            }
            pc.printf("\r\n") ;
        }
        camera.ReadStop() ;        
    }
}
