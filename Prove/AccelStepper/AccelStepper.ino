// ProportionalControl.pde
// -*- mode: C++ -*-
//
// Make a single stepper follow the analog value read from a pot or whatever
// The stepper will move at a constant speed to each newly set posiiton, 
// depending on the value of the pot.
//
// Copyright (C) 2012 Mike McCauley
// $Id: ProportionalControl.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
// ================================================================
// ===               SETUP SERIALE				                ===
// ================================================================
#define SERIAL_BAUD_RATE 9600

/* HW_CONFIG_MOTORS_H_ */
/*
* hw_config.h
*
* Created: 25/01/2015 16:53:17
*  Author: Luca
*/


#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

// parametri ficici del robot---------------
#define	STEPSPERCM 23.342725f 
#define	MMPERSTEPS  0.42840f 

//----------------------------------------------
// pin da configurare in INPUT -----------------
// N.B. Arduino (Atmega) pins default to inputs, n a high-impedance state. !!!
//----------------------------------------------

#define Pin_irproxy_FW 28		// sensore di prossimità anteriore
#define Pin_PIR1	29	//Sensore presenza umana

#define Pin_EncRa	2	//encoder Right Motor segnale a	
#define Pin_EncRb	31	//encoder Right Motor segnale b
#define Pin_EncLa	3	//encoder Left Motor segnale a	
#define Pin_EncLb	33	//encoder Left Motor segnale b
#define Pin_irproxy_FWHL 34		// sensore di prossimità anteriore per alta luce
#define Pin_irproxy_BK 35		// sensore di prossimità POSTERIORE
#define Pin_BtState 40
#define Pin_SonarEcho 32	// pin ECHO da Sonar


//----------------------------------------------
// pin da configurare in OUTPUT				----
//----------------------------------------------
#define  Pin_ONBOARD_LED 13


#define Pin_MotCK 10		//=FREQUENCYTIMER2_PIN=10	// CLOCK  << vedere in funzione del timer usato Timer 3 =>pin A=5, B=2, C=3
#define Pin_ServoSonarPWM  8	// 
#define Pin_LaserOn 22	// 23 Accensione Laser 1=CW  0=CCW
#define Pin_MotCWR 23	// 23 left CW  - 24 RIGHT MOTOR
#define Pin_MotCWL 24	// 23 RIGHT MOTOR 1=CW  0=CCW
#define Pin_MotENR 25	// 26 RIGHT MOTOR ENABLE
#define Pin_MotENL 26	// 25 LEFT MOTOR ENABLE

#define Pin_SonarTrig 30	// pin TRIGGER vs Sonar

#define Pin_Rele1	36
#define Pin_Rele2	38
#define Pin_BtOnOff  42



//iNGRESSI ANALOGICI----------------------------------------------
#define Pin_AnaBase 54 // A0 =54 per recuperara l'indice dell'array uso [Pin_AnaBase -Pin_AnaLight]
#define Pin_AnaPot1 A0		// Potenziometro che regola la velocità MAX
#define Pin_AnaVbat A1		// Potenziometro che regola la velocità MAX
#define Pin_AnaLight A2		// Sensore di luce LDR
//----------------------------------------------


#endif /* HW_CONFIG_MOTORS_H_ */


	//#include <Servo.h>
	//#include <Newping.h>
	//// ================================================================
	//// ===               ISTANZA OGGETTO ROBOT		                ===
	//// ================================================================

	#include <TimerThree.h>
	#include <AccelStepper.h>

//	struct robot_c robot;
	int ACCEL=	100;

static	int dist =1000;
// Define a stepper and the pins it will use
AccelStepper stepperR( AccelStepper::DRIVER, Pin_MotCK, Pin_MotCWR, Pin_MotENR ); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepperL( AccelStepper::DRIVER, Pin_MotCK, Pin_MotCWL, Pin_MotENL ); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

void setup()
{  
	//-----------------------------------------
	// Setup SERIALE
	//-----------------------------------------	
	Serial.begin(SERIAL_BAUD_RATE);
	Serial.println("AccelStepper.ino");

	/*

	#define MAXSPEED	400	// 770 comincia a perdere  colpi

	stepperR.setEnablePin(Pin_MotENR);stepperL.setEnablePin(Pin_MotENL);
//	stepperR.setPinsInverted(true,false,false);
	stepperR.setMaxSpeed(MAXSPEED);stepperL.setMaxSpeed(MAXSPEED);
	stepperR.setAcceleration(ACCEL);stepperL.setAcceleration(ACCEL);
	stepperR.move(dist);	stepperL.move(dist);

	Serial.print("_spd");Serial.print(", ");
	Serial.print("_acc");Serial.print(", ");
	Serial.print("_cn");Serial.print(", ");
	Serial.print("_c0");Serial.print(", ");
	Serial.print("_n");Serial.print(", ");
	Serial.print("_stepI");Serial.print(", ");
	Serial.print("distTo");Serial.print(", ");
	Serial.println("stp2Stop");

	stepperR.move(-dist); stepperL.move(dist); // R>0 e L<0 va indietro
	*/
#define ROBOT_MOTORENABLE_ACTIVE 0		// 1 = enable a logica positiva 0 = enable a logica negata

	pinMode( Pin_MotCK, OUTPUT );			digitalWrite( Pin_MotCK, 0 ); 

	pinMode( Pin_MotCWR, OUTPUT );		digitalWrite( Pin_MotCWR, 0 );
	pinMode( Pin_MotENR, OUTPUT );		digitalWrite( Pin_MotENR, !ROBOT_MOTORENABLE_ACTIVE );
	pinMode( Pin_MotCWL, OUTPUT );		digitalWrite( Pin_MotCWL, 0 );
	pinMode( Pin_MotENL, OUTPUT );		digitalWrite( Pin_MotENL, !ROBOT_MOTORENABLE_ACTIVE );
	pinMode( Pin_Rele1, OUTPUT );			digitalWrite( Pin_Rele1, 1 );
	pinMode( Pin_Rele2, OUTPUT );			digitalWrite( Pin_Rele2, 1 );	// i Rele 

	//while (!Serial.read()) {}
	digitalWrite( Pin_MotENR, 1 );
}

void loop()
{
	digitalWrite( Pin_MotCK, 1 );
	delay( 1 );
	digitalWrite( Pin_MotCK, 0 );
	delay( 1 );

	/*
    // If at the end of travel go to the other end
    if ((stepperL.distanceToGo() ==0)||(!digitalRead(Pin_irproxy_FW))){
		stepperR.stop();stepperL.stop();
		delay(5000);
		Serial.println("-inverto direzione-");

		dist =map(analogRead(Pin_AnaPot1),0,1023,500,1500);Serial.print("dist:");Serial.println(dist);

		dist =500;// -dist;	//

		stepperR.moveTo(dist); stepperL.moveTo(dist);
	}
    stepperR.run();
    stepperL.run();
	
	*/
}
