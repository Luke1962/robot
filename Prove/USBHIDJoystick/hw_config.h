/*
 * hw_config.h
 *
 * Created: 25/01/2015 16:53:17
 *  Author: Luca
 */ 


#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_



//----------------------------------------------
// pin da configurare in INPUT --------------------------
//----------------------------------------------
#define Pin_GetDirection 28
#define Pin_irproxy_FW 28

//iNGRESSI ANALOGICI----------------------------------------------
	#define Pin_Pot1 A0		// Potenziometro che regola la velocità MAX

//----------------------------------------------

// The LED is attached to pin 13 on Arduino.
//----------------------------------------------
// pin da configurare in OUTPUT				----
//----------------------------------------------
#define  ONBOARD_LED_PIN  13

	#define Pin_MotCK 5	// CLOCK  << vedere in funzione del timer usato Timer 3 =>pin A=5, B=2, C=3
	#define Pin_MotCWR 23	// 23 left CW  - 24 RIGHT MOTOR
	#define Pin_MotENR 25	// 26 RIGHT MOTOR ENABLE

	#define Pin_MotCWL 24	// 23 RIGHT MOTOR 1=CW  0=CCW
	#define Pin_MotENL 26	// 25 LEFT MOTOR ENABLE






#endif /* HW_CONFIG_MOTORS_H_ */
