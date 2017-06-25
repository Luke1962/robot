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


#define Pin_MotCK 5	// CLOCK  << vedere in funzione del timer usato Timer 3 =>pin A=5, B=2, C=3
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
