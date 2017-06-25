/*
 * MFRC522 - Library to use ARDUINO RFID MODULE KIT 13.56 MHZ WITH TAGS SPI W AND R BY COOQROBOT.
 * The library file MFRC522.h has a wealth of useful info. Please read it.
 * The functions are documented in MFRC522.cpp.
 *
 * Based on code Dr.Leong   ( WWW.B2CQSHOP.COM )
 * Created by Nicola Pison (webingenerale.com), Jen 2014.
 * Released into the public domain.
 *
 * Sample program showing how to read data from a PICC using a MFRC522 reader on the Arduino SPI interface.
 *----------------------------------------------------------------------------- empty_skull
 * add pin configuration for arduino mega
 * http://www.webingenerale.com
 ----------------------------------------------------------------------------- Nicola Pison
 * Pin layout should be as follows:
 * Signal     Pin              Pin               Pin
 *            Arduino Uno      Arduino Mega      MFRC522 board
 * ------------------------------------------------------------
 * Reset      7                5                 RST
 * SPI SS     6                53                SDA
 * SPI MOSI   11               52                MOSI
 * SPI MISO   12               51                MISO
 * SPI SCK    13               50                SCK
 *
 * The reader can be found on eBay for around 5 dollars. Search for "mf-rc522" on ebay.com.
 */
 
#include <SPI.h>
#include <MFRC522.h>
 
#define SS_PIN 6
#define RST_PIN 7
 
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
 
long previousMillis = 0;
long interval = 10;
 
int rele = 5;  //pin relè
 
String uidRFID[] = {"codeRFID_A","codeRFID_B","codeRFID_C","codeRFID_D"};
 
 
void setup() {
     
        
        Serial.begin(9600); // Initialize serial communications with the PC
    SPI.begin();        // Init SPI bus
    mfrc522.PCD_Init(); // Init MFRC522 card
        pinMode(rele, OUTPUT);
    Serial.println("Ingresso Aziendale");
        Serial.println("RFID registrati n° --> "+String(sizeof(uidRFID)/sizeof(String)));
        Serial.println("In attesa di lettura...");
       
}
 
void loop() {
 
  unsigned long currentMillis = millis();
    
  if(currentMillis - previousMillis > interval) {
    previousMillis = currentMillis; 
     
      // Look for new cards
    if ( ! mfrc522.PICC_IsNewCardPresent()) {
        return;
    }
 
    // Select one of the cards
    if ( ! mfrc522.PICC_ReadCardSerial()) {
        return;
    }
 
String uid_s = "";
 
if (!mfrc522.PICC_IsNewCardPresent() && !mfrc522.PICC_ReadCardSerial()) {
   
    for (byte i = 0; i < mfrc522.uid.size; i++) {
         
        String uid_a = String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
        String uid_b = String(mfrc522.uid.uidByte[i], HEX);
           
        uid_s = uid_s+uid_a+uid_b;
          
 
    }
  }
   
  Serial.print("RFID UID rivelato --> ");
  Serial.println(uid_s);
  Serial.println("");
  boolean controllo = false;
  for (int i = 0; i < sizeof(uidRFID)/sizeof(String); i++){
   
    if(uidRFID[i] == uid_s){
      
            Serial.println("Accesso consentito");
            openDoor();
            controllo = true;
            break;
             
      }
    }
      
   if(!controllo){
   Serial.println("Accesso vietato");
   delay(1000);
   }
    
   Serial.println();
   Serial.println("In attesa di lettura...");
    
  }
}
 
void openDoor(){
 
  digitalWrite(rele, HIGH);
  Serial.println("Relè ON");
  delay(2000);
  digitalWrite(rele, LOW);
  Serial.println("Relè OFF");
   
}