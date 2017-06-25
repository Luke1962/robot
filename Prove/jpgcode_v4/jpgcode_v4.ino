/***************************************************************************/
/*                                                                         */
/*  File: main.cpp                                                         */
/*  Autor: bkenwright@xbdev.net                                            */
/*  URL: www.xbdev.net                                                     */
/*                                                                         */
/***************************************************************************/
/*
	Jpeg File Format Explained
*/
/***************************************************************************/
#pragma region debug

#define dbg(cha) 	 Serial.println(F(cha));
#define dbg2(t,cha)	 Serial.print(F(t));Serial.println(cha);  
#define dbgHex(b)	 Serial.print(b,HEX);Serial.print(" ");  
//#include <MemoryFree/MemoryFree.h>

#pragma endregion



//#include <windows.h>
#include <Arduino.h>

//#include <stdio.h>		// sprintf(..), fopen(..)
//#include <stdarg.h>     // So we can use ... (in Serial.print)


#include "./loadjpg.h"	// ConvertJpgFile(..)
#include "./savejpg.h"    // SaveJpgFile(..)



/***************************************************************************/
/*                                                                         */
/* DRIVER TFT                                                           */
/*                                                                         */
/***************************************************************************/
#pragma region LIBRERIE ILI9341_due tft 


#include <SPI.h>
#include <ILI9341_due/ILI9341_due_config.h>
#include <ILI9341_due/ILI9341_due.h>

#include "ILI9341_due\fonts\Arial_bold_14.h"

#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8

ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);


#pragma endregion

/***************************************************************************/
/*                                                                         */
/* FeedBack Data                                                           */
/*                                                                         */
/***************************************************************************/

//Saving debug information to a log file
//void Serial.print(const char *fmt, ...) 
//{
//	va_list parms;
//	char buf[256];
//
//	// Try to print in the allocated space.
//	va_start(parms, fmt);
//	vsprintf (buf, fmt, parms);
//	va_end(parms);
//
//	// Write the information out to a txt file
//	FILE *fp = fopen("output.txt", "a+");
//	fprintf(fp, "%s", buf);
//	fclose(fp);
//
//}// End Serial.print(..)


 
/***************************************************************************/
/*                                                                         */
/* Entry Point                                                             */
/*                                                                         */
/***************************************************************************/


//int __stdcall WinMain (HINSTANCE hInst, HINSTANCE hPrev, LPSTR lpCmd, int nShow)
//{
//
//	// Create a jpg from a bmp
//	//SaveJpgFile("smiley.bmp", "ex.jpg");
//
//	// Create a bmp from a jpg
//	ConvertJpgFile("smiley.jpg", "smiley.bmp");
//
//
//	//ConvertJpgFile("cross.jpg", "cross.bmp");
//
//	return 0;
//}// End WinMain(..)


void setup() {
	Serial.begin(115200);

	ConvertJpgFile("smiley.jpg", "smiley.bmp");

}
void loop() {

}