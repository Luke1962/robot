/*TEST LCD */


#include <Adafruit_GFX.h> // Hardware-specific library
#include <MCUFRIEND_kbv.h>
#include "arduino.h"

//
//
MCUFRIEND_kbv tft;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(9600);
    tft.reset();
    uint16_t identifier = tft.readID();
    Serial.print("ID = 0x");
    Serial.println(identifier, HEX);
    if (identifier == 0xEFEF) identifier = 0x9486;
    tft.begin(0x9488);
     tft.fillScreen(0);
}

char *msg[] = { "PORTRAIT", "LANDSCAPE", "PORTRAIT_REV", "LANDSCAPE_REV" };
uint8_t aspect;

void loop()
{
    // put your main code here, to run repeatedly:
    uint16_t x = 50, y = 100;
    tft.setRotation(aspect);
    tft.fillScreen(0x0000);
    tft.setCursor(0, 0);
    tft.setTextSize(2);
    tft.println(msg[aspect]);
    tft.setCursor(x, y);
    tft.println("[x=" + String(x) + ",y=" + String(y) + "]");
    delay(5000);
    tft.println("INVERT ON");
    tft.invertDisplay(true);
    delay(1000);
    tft.invertDisplay(false);
    tft.println("INVERT OFF");
    delay(1000);
    if (++aspect >= 4) aspect = 0;
}


