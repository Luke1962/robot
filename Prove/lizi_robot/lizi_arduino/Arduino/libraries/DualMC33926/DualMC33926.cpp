#include "DualMC33926.h"

// Defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
// Constructors ////////////////////////////////////////////////////////////////

DualMC33926::DualMC33926()
{
  //Pin map
  _nD2 = 4;
  _M1DIR = 7;
  _M2DIR = 8;
  _nSF = 12;
  _M1PWM = 9;
  _M2PWM = 10;
  _M1FB = A0; 
  _M2FB = A1;
}

DualMC33926::DualMC33926(unsigned char M1DIR, unsigned char M1PWM, unsigned char M1FB,
                                               unsigned char M2DIR, unsigned char M2PWM, unsigned char M2FB,
                                               unsigned char nD2, unsigned char nSF)
{
  //Pin map
  //PWM1 and PWM2 cannot be remapped because the library assumes PWM is on timer1
  _M1PWM = M1PWM; 
  _M2PWM = M2PWM;
  _nD2 = nD2;
  _M1DIR = M1DIR;
  _M2DIR = M2DIR;
  _nSF = nSF;
  _M1FB = M1FB; 
  _M2FB = M2FB;
}

// Public Methods //////////////////////////////////////////////////////////////
void DualMC33926::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_M1DIR,OUTPUT);
  pinMode(_M1PWM,OUTPUT);
 // pinMode(_M1FB,INPUT);
  pinMode(_M2DIR,OUTPUT);
  pinMode(_M2PWM,OUTPUT);
// pinMode(_M2FB,INPUT);
  pinMode(_nD2,OUTPUT);
  digitalWrite(_nD2,HIGH); // default to on
  pinMode(_nSF,INPUT);

#ifdef __SAM3X8E__  //DUE
  analogReadResolution(12); //0-4095
// #undef PWM_FREQUENCY
 //#define PWM_FREQUENCY 20000 //20khz

#else //mega
// PWM frequency calculation
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz

//TCCR1A = 0b10100000;
 // TCCR1B = 0b00010001;
 // ICR1 = 400;
   // TCCR2A settings
        //---------------------------------------------------------------------
        // These bits control the Output Compare pin (OC2A) behavior. If one or
        // both of the COM2A1:0 bits are set, the OC2A output overrides the
        // normal port functionality of the I/O pin it is connected to.
        // However, note that the Data Direction Register (DDR) bit
        // corresponding to the OC2A pin must be set in order to enable the
        // output driver.
        // When OC2A is connected to the pin, the function of the COM2A1:0 bits
        // depends on the WGM22:0 bit setting.
        //
        // Fast PWM Mode
        // COM2A1 COM2A0
        // 0 0 Normal port operation, OC2A disconnected.
        // 0 1 WGM22 = 0: Normal Port Operation, OC0A Disconnected.
        //   WGM22 = 1: Toggle OC2A on Compare Match.
        // 1 0 Clear OC2A on Compare Match, set OC2A at BOTTOM
        // 1 1 Clear OC2A on Compare Match, clear OC2A at BOTTOM
        cbi(TCCR2A,COM2A1);
        cbi(TCCR2A,COM2A0);
        sbi(TCCR2A,COM2B1);
        cbi(TCCR2A,COM2B0);

        // Combined with the WGM22 bit found in the TCCR2B Register, these bits
        // control the counting sequence of the counter, the source for maximum
        // (TOP) counter value, and what type of waveform generation to be used
        // Modes of operation supported by the Timer/Counter unit are:
        // - Normal mode (counter),
        // - Clear Timer on Compare Match (CTC) mode,
        // - two types of Pulse Width Modulation (PWM) modes.
        //
        // Mode WGM22 WGM21 WGM20 Operation TOP
        // 0 0 0 0 Normal  0xFF
        // 1 0 0 1 PWM  0xFF
        // 2 0 1 0 CTC  OCRA
        // 3 0 1 1 Fast PWM 0xFF
        // 4 1 0 0 Reserved -
        // 5 1 0 1 PWM  OCRA
        // 6 1 1 0 Reserved -
        // 7 1 1 1 Fast PWM OCRA
        cbi(TCCR2B,WGM22);
        sbi(TCCR2A,WGM21);
        sbi(TCCR2A,WGM20);

        //---------------------------------------------------------------------
        // TCCR2B settings
        //---------------------------------------------------------------------
        // The FOC2A bit is only active when the WGM bits specify a non-PWM
        // mode.
        // However, for ensuring compatibility with future devices, this bit
        // must be set to zero when TCCR2B is written when operating in PWM
        // mode. When writing a logical one to the FOC2A bit, an immediate
        // Compare Match is forced on the Waveform Generation unit. The OC2A
        // output is changed according to its COM2A1:0 bits setting. Note that
        // the FOC2A bit is implemented as a strobe. Therefore it is the value
        // present in the COM2A1:0 bits that determines the effect of the
        // forced compare.
        // A FOC2A strobe will not generate any interrupt, nor will it clear
        // the timer in CTC mode using OCR2A as TOP.
        // The FOC2A bit is always read as zero.
        cbi(TCCR2B,FOC2A);
        cbi(TCCR2B,FOC2B);

        // The three Clock Select bits select the clock source to be used by
        // the Timer/Counter.
        // CS22 CS21 CS20 Prescaler
        // 0 0 0 No clock source (Timer/Counter stopped).
        // 0 0 1 No prescaling
        // 0 1 0 8
        // 0 1 1 32
        // 1 0 0 64
        // 1 0 1 128
        // 1 1 0 256
        // 1 1 1 1024
        cbi(TCCR2B,CS22);
        cbi(TCCR2B,CS21);
        sbi(TCCR2B,CS20);

#endif

 

}

// Return torque status
unsigned char DualMC33926::getTorque() {
  return digitalRead(_nD2);
}

// Set torque status
void DualMC33926::setTorque(bool t)
{
if (t) digitalWrite(_nD2,HIGH);
else digitalWrite(_nD2,LOW);
}

//restart driver
void DualMC33926::restart()
{
digitalWrite(_nD2,LOW);
delay(1);
digitalWrite(_nD2,HIGH);
}

// Set speed for motor 1, speed is a number betwenn -255 and 255
void DualMC33926::setM1Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 255)  // Max PWM dutycycle
    speed = 255;
 
  analogWrite(_M1PWM,speed);
  
  if (reverse)
    digitalWrite(_M1DIR,HIGH);
  else
    digitalWrite(_M1DIR,LOW);
}

// Set speed for motor 2, speed is a number betwenn -255 and 255
void DualMC33926::setM2Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 255)  // Max PWM dutycycle
    speed = 255;

  analogWrite(_M2PWM,speed); 

  if (reverse)
    digitalWrite(_M2DIR,HIGH);
  else
    digitalWrite(_M2DIR,LOW);
}

// Set speed for motor 1 and 2
void DualMC33926::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

// Return motor 1 current value in milliamps.
float DualMC33926::getM1CurrentMilliamps()
{
#ifdef __SAM3X8E__  //DUE
  // 3.3V / 4096 ADC counts / 525 mV per A = 1.5346 mA per count
  return (float)analogRead(_M1FB) * 1.5346;
#else //Mega
  // 5V / 1024 ADC counts / 525 mV per A = 9.3 mA per count
  return (float)analogRead(_M1FB) * 9.3;
#endif


}

// Return motor 2 current value in milliamps.
float DualMC33926::getM2CurrentMilliamps()
{
#ifdef __SAM3X8E__  //DUE
  // 3.3V / 4096 ADC counts / 525 mV per A = 1.5346 mA per count
  return (float)analogRead(_M2FB) * 1.5346;
#else //Mega
  // 5V / 1024 ADC counts / 525 mV per A = 9.3 mA per count
  return (float)analogRead(_M2FB) * 9.3;
#endif

}

// Return error status
unsigned char DualMC33926::getFault()
{
  return !digitalRead(_nSF);
}


