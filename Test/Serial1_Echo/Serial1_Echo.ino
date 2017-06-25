
/*
  Serial Call and Response
 Language: Wiring/Arduino
*/
#define SerialIN Serial
#define SerialOUT Serial
int inByte = 0;         // incoming serial byte
 #define Pin_Buzzer 13		//=FREQUENCYTIMER2_PIN=10	// CLOCK  << vedere in funzione del timer usato Timer 3 =>pin A=5, B=2, C=3
void playSingleNote(int nota, uint16_t noteDuration) {
	tone(Pin_Buzzer, nota, noteDuration);
	delay(noteDuration * 1.30);
	noTone(Pin_Buzzer);

}

void setup()
{
	SerialIN.begin(115200);
	SerialOUT.begin(115200);
	SerialOUT.println( "Echo su seriale" );


	playSingleNote(5, 1500);

}
void loop()
{

	if (SerialIN.available() > 0) {
		// get incoming byte:
		inByte = SerialIN.read();
		//playSingleNote(1000, 90);

		if (false)// (inByte<20)
		{
			SerialOUT.print("[");
			SerialOUT.print(inByte);
			SerialOUT.println("]");

		} else
		{
			//Serial1.write(inByte);
			SerialOUT.write(inByte);
		}
	}    
	if (SerialOUT.available() > 0) {
	// get incoming byte:
	inByte = SerialOUT.read();
	 
	// Serial.write(inByte);
	SerialIN.write(inByte);
	}
}



