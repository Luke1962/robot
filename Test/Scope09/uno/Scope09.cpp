// Scopino - Scope application
// By Amit Zohar
// Ver 2.0, October 2014

// Defines for setting A2D register bits
#define a2dBitClr(bit) (_SFR_BYTE(ADCSRA) &= ~_BV(bit))
#define a2dBitSet(bit) (_SFR_BYTE(ADCSRA) |= _BV(bit))

#include "arduino.h"

//
void a2dSetup(int factor);
void getCommand();
unsigned long sample();
void sendSamples(unsigned long sampTime);
//
const int maxSamples = 500;
const int maxSampChan = maxSamples/2;
const int maxChannel = 2;
word samples[maxSamples];
int channel=1, timeRes=200, a2d=1, timeStep=1, voltAmp=2, voltCoup=0, trigVal=0, trigMode=0;
const int pinSquare=2;

void setup() {
  Serial.begin(115200); // Fast IO
  Serial.setTimeout(10); // Quick read
  pinMode(pinSquare, OUTPUT); // Square wave 1KHz
  tone(pinSquare, 1000);
}

// Set A2D register clock division factor, enabling faster sampling
void a2dSetup(int factor)
{
  if (factor & 1) a2dBitSet(ADPS0);
  else a2dBitClr(ADPS0);
  if (factor & 2) a2dBitSet(ADPS1);
  else a2dBitClr(ADPS1);
  if (factor & 4) a2dBitSet(ADPS2);
  else a2dBitClr(ADPS2);
}

///////////////////////// Process serial input
// Protocol: *C<channel>T<time resolution>A<A2D register>S<samples division>V<voltage amp.>P<voltage coupling>G<trigger value>H<trigger mode>E
// (C) Channel: 1-2
// (T) Time resolution: at least 20uSec per channel
// (A) A2D sampling speed: 0-7
// (S) Samples division: divide # of samples for speed
// (V) Voltage amp.: 1=amplified (H/W), 2=0-5V as-is, 3=0-50V attenuated by 10 (H/W)
// (P) Voltage coupling: 0=DC, 1=AC (H/W)
// (G) Trigger value: value needed to be crossed for trigger (0-1023)
// (H) Trigger mode: 0=free run, 1=positive slope, 2=negative slope, 3=both slopes
// Example: *C10T5S333V8P1G1H2E
// Default: *C1T200S1V2P0G0H0E
void getCommand()
{
  const int recBuf=64;
  if (Serial.available() > 0) // Incoming chars?
  {
    delay(10); // Let all chars arrive
    char recRare[recBuf];
    int chars = Serial.readBytes(recRare,recBuf); // Get buf to String
    recRare[chars] = 0;
    String rec = recRare;
    if (rec.startsWith("*")) // Valid frame start
    {
      //      Serial.write('*');
      int posC = rec.indexOf("C");
      int posT = rec.indexOf("T");
      int posA = rec.indexOf("A");
      int posS = rec.indexOf("S");
      int posV = rec.indexOf("V");
      int posP = rec.indexOf("P");
      int posG = rec.indexOf("G");
      int posH = rec.indexOf("H");
      int posE = rec.indexOf("E");
      String prm;
      char ca[recBuf];
      prm = rec.substring(posC+1,posT); // Channel
      prm.toCharArray(ca,sizeof(ca));
      channel = atoi(ca);
      prm = rec.substring(posT+1,posA); // Time resolution
      prm.toCharArray(ca,sizeof(ca));
      timeRes = atoi(ca);
      prm = rec.substring(posA+1,posS); // A2D speed (1=faster sampling)
      prm.toCharArray(ca,sizeof(ca));
      a2dSetup(atoi(ca));
      prm = rec.substring(posS+1,posV); // Samples division
      prm.toCharArray(ca,sizeof(ca));
      timeStep = atoi(ca);
      prm = rec.substring(posV+1,posP); // Voltage amp.
      prm.toCharArray(ca,sizeof(ca));
      voltAmp = atoi(ca);
      prm = rec.substring(posP+1,posG); // Voltage coup.
      prm.toCharArray(ca,sizeof(ca));
      voltCoup = atoi(ca);
      prm = rec.substring(posG+1,posH); // Trigger value
      prm.toCharArray(ca,sizeof(ca));
      trigVal = atoi(ca);
      prm = rec.substring(posH+1,posE); // Trigger mode
      prm.toCharArray(ca,sizeof(ca));
      trigMode = atoi(ca);
    }
  }
}

///////////////////////// Sample
unsigned long sample()
{
  // Trigger check
  int preSamp=-1, curSamp;
  const int waitTrig = 1000;
  for  (int i=0; i<=waitTrig; i++) // Try to catch the trigger
  {
    if ( trigMode == 0) break; // Free run
    curSamp = analogRead(0);
    if (preSamp != -1) // We have previous sample
    {
      boolean pos = false, neg = false; // Calculate which slope it is
      if (preSamp < trigVal && curSamp >= trigVal) pos = true;
      if (preSamp >= trigVal && curSamp < trigVal) neg = true;
      if ( trigMode == 1 && pos) break; // Positive slope
      if ( trigMode == 2 && neg) break; // Negative slope
      if ( trigMode == 3 && (pos || neg)) break; // Both
    }
    preSamp = curSamp; // Save last sample
    if (i == waitTrig) return 0; // No trigger, do some other work
  }

  // Sampling
  unsigned long samp1, timeFrom1, nextSamp;
  int rawSamp;
  samp1 = micros(); // Sampling start
  int sampNum = maxSamples/timeStep;
  for ( unsigned long i=0;i<sampNum;i++) // Sampling loop
  {
    samples[i] = analogRead(0); // Sampling channel 1
    if (channel == 2) samples[i+maxSampChan] = analogRead(1); // Channel 2

    nextSamp = timeRes*(i+1); // Time for next sample
    do timeFrom1 = micros()-samp1; // Wait for next sample time
    while (timeFrom1 < nextSamp);
  }
  return timeFrom1;
}

///////////////////////// Send output
// Protocol: *Channel>Time resolution:Sample1,Sample2...,
void sendSamples(unsigned long sampTime)
{
  String samp;
  for (int ch=0; ch<channel; ch++) // Sample all channels
  {
    //    Serial.print("*1:");
    Serial.print("*"); // Channel
    Serial.print(ch+1);
    Serial.print(":");

    Serial.print(sampTime/maxSamples); // Time resolution
    Serial.print(">");
    for (int i=0;i<maxSamples/timeStep;i++) // Samples
    {
      samp =  String(samples[i+maxSampChan*ch], HEX); // Send in HEX format   
      Serial.print(samp);
      Serial.print(',');
    }
    Serial.println();
  }
}

void loop(){
  getCommand(); // We listen to the master program
  unsigned long sampTime = sample(); // We sample
  if (sampTime > 0) sendSamples(sampTime); // We have samples
}



