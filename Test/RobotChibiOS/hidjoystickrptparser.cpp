#include  <ChibiOS_AVR.h>
#include "hidjoystickrptparser.h"
#include "robot.h"

JoystickReportParser::JoystickReportParser(JoystickEvents *evt) :
joyEvents(evt),
oldHat(0xDE),
oldButtons(0) {
        for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
                oldPad[i] = 0xD;
}

void JoystickReportParser::Parse(HID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
        bool match = true;

        // Checking if there are changes in report since the method was last called
        for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++)
                if (buf[i] != oldPad[i]) {
                        match = false;
                        break;
                }

        // Calling Game Pad event handler
        if (!match && joyEvents) {
                joyEvents->OnGamePadChanged((const GamePadEventData*)buf);

                for (uint8_t i = 0; i < RPT_GEMEPAD_LEN; i++) oldPad[i] = buf[i];
        }

        uint8_t hat = (buf[5] & 0xF);

        // Calling Hat Switch event handler
        if (hat != oldHat && joyEvents) {
                joyEvents->OnHatSwitch(hat);
                oldHat = hat;
        }

        uint16_t buttons = (0x0000 | buf[6]);
        buttons <<= 4;
        buttons |= (buf[5] >> 4);
        uint16_t changes = (buttons ^ oldButtons);

        // Calling Button Event Handler for every button changed
        if (changes) {
                for (uint8_t i = 0; i < 0x0C; i++) {
                        uint16_t mask = (0x0001 << i);

                        if (((mask & changes) > 0) && joyEvents)
                                if ((buttons & mask) > 0)
                                        joyEvents->OnButtonDn(i + 1);
                                else
                                        joyEvents->OnButtonUp(i + 1);
                }
                oldButtons = buttons;
        }
}
void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt) {

	/* JOISTICK DI SINISTRA
		IN AVANTI:	evt->Z2 decresce da 0x80 a 00		Y
		INDIETRO:	evt->Z2 cresce da 0x80 A 0xFF
		SINISTRA	evt->Z1 decresce da 0x80 a 00		X
		destra:		evt->Z1 cresce da 0x80 A 0xFF
	*/
	long yForce, xForce;
	yForce = abs(evt->Z2 - joiCenterValue_Y);
	xForce = abs(evt->Z1 - joiCenterValue_X);
	int analog1, ck, limitCKmaxspeed;
	analog1 = analogRead(Pin_Pot1);	// 0-1023
	limitCKmaxspeed = map(analog1, 0,1023, ROBOT_MOTOR_CLOCK_microsecondMIN+128,ROBOT_MOTOR_CLOCK_microsecondMAX); 
	ck=map(yForce,0,127,ROBOT_MOTOR_CLOCK_microsecondMAX,limitCKmaxspeed);
	//ck=ROBOT_MOTOR_CLOCK_microsecondMAX-yForce*(ROBOT_MOTOR_CLOCK_microsecondMAX-ROBOT_MOTOR_CLOCK_microsecondMIN)/127; //ROBOT_MOTOR_CLOCK_microsecondMAX - yForce*(ROBOT_MOTOR_CLOCK_microsecondMAX - ROBOT_MOTOR_CLOCK_microsecondMIN)/127;

	if (evt->Z2  == joiCenterValue_Y) { // stop o rotazione
		if (evt->Z1 == joiCenterValue_X){ // stai fermo
			robot.stop();
		}
		else if (evt->Z1 > joiCenterValue_X){		// rotazione a DX - CW
			robot.goCW(map(xForce,0,127,ROBOT_MOTOR_CLOCK_microsecondMAX,limitCKmaxspeed));
		}
		else{		// rotazione a SX - CCW
			robot.goCCW(map(xForce,0,127,ROBOT_MOTOR_CLOCK_microsecondMAX,limitCKmaxspeed));

		}

	}
	else if (evt->Z2  > joiCenterValue_Y ) {	// forward
		robot.goFW(ck);
	}
	else{	// backward
		robot.goBK( ck);
	};

	// wait to enter print region
	//chMtxLock(&MutexSerial);

	Serial.print("enL:"); Serial.println(robot.lastCommand.enL); 
	Serial.print("enR:"); Serial.println(robot.lastCommand.enR); 
	Serial.print("X1: ");
	PrintHex<uint8_t > (evt->X, 0x80);
	Serial.print("\tY1: ");
	PrintHex<uint8_t > (evt->Y, 0x80);
	Serial.print("\tZ1: ");
	PrintHex<uint8_t > (evt->Z1, 0x80);
	Serial.print("\tZ2: ");
	PrintHex<uint8_t > (evt->Z2, 0x80);
	Serial.println("\tRz: ");
	PrintHex<uint8_t > (evt->Rz, 0x80);
	Serial.print("\t Yforce: ");
	Serial.print(yForce);
	Serial.print("\t ck: ");
	Serial.print(ck);		
	Serial.print("\t limitCKmaxspeed: ");
	Serial.println(limitCKmaxspeed); 

	Serial.println("");	

	// exit protected region

	//chMtxUnlock();
}

//void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt) {
//        Serial.print("X1: ");
//        PrintHex<uint8_t > (evt->X, 0x80);
//        Serial.print("\tY1: ");
//        PrintHex<uint8_t > (evt->Y, 0x80);
//        Serial.print("\tX2: ");
//        PrintHex<uint8_t > (evt->Z1, 0x80);
//        Serial.print("\tY2: ");
//        PrintHex<uint8_t > (evt->Z2, 0x80);
//        Serial.print("\tRz: ");
//        PrintHex<uint8_t > (evt->Rz, 0x80);
//        Serial.println("");
//}

void JoystickEvents::OnHatSwitch(uint8_t hat) {
        Serial.print("Hat Switch: ");
        PrintHex<uint8_t > (hat, 0x80);
        Serial.println("");
}

void JoystickEvents::OnButtonUp(uint8_t but_id) {
        Serial.print("Up: ");
        Serial.println(but_id, DEC);
}

void JoystickEvents::OnButtonDn(uint8_t but_id) {
        Serial.print("Dn: ");
        Serial.println(but_id, DEC);
}
