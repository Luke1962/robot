/* SP0256-AL2 Speech Chip
 * Control Code

 * Text: HELLO WORLD.
 * Phoneme: HH EH LL AX OW (PAUSE) WW ER1 LL PA2 DD1 (PAUSE)
 * Octal: 033 007 055 017 065 003 056 063 055 001 025 004
 * Dec: 27 7 45 15 53 3 46 51 45 1 21 4
 */


char data[64];

// PINOUT
//  SBY -> PB1 / Pin 9
// !ALD -> PD6 / Pin 10
// A6..A1 -> PD5..0 / Pin 0..5
#define SBY 6    //was 9 LRQ
#define ALD 7    // was 10
//#define LED 13

void setup()
{
  int i;
  
  // DON'T USE Serial Ports!!!

  // LOAD data  
  i = 0;
  data[i++]  = 27; //HH1
  data[i++]  = 7;  //EH
  data[i++]  = 45; //LL
  data[i++]  = 15; //AX
  data[i++]  = 53; //OW
  data[i++]  = 3;  //PA4
  data[i++]  = 46; //WW
  data[i++]  = 51; //ER1
  data[i++]  = 45; //LL
  data[i++]  = 1;  //PA2
  data[i++]  = 21; //DD1
  data[i++]  = 4;  //PA5  
  data[i++]  = 0;  //end

  // Could do: DDRD = 0b00011111;
  for (int p=8; p < 14; p++) {
    pinMode(p, OUTPUT);
  }
  pinMode(ALD, OUTPUT);
  pinMode(SBY, INPUT);
//  pinMode(LED, OUTPUT); // LED
}
int i;

void loop() 
{
    
  i = 0;
  digitalWrite(ALD, HIGH);    
  while (data[i] != 0) {
    // Set address
    PORTB = data[i];    //WAS PORTD=
    // Set !ADL=0 for 2usec to tell the chip to read
    digitalWrite(ALD, LOW);
    delayMicroseconds(2);
    digitalWrite(ALD, HIGH);
    i++;
    // Wait for SBY=1 (standby) to indicate chip is done speaking
    while (digitalRead(SBY) == 0);
  }
  delay(2000); // delay 2 sec between phrases
}
