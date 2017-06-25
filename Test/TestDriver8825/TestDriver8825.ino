#define SERIAL_BAUD_RATE 9600

int dirPin = 23;
int stepperPin = 10;
int enablePin = 25;

void setup() {
	Serial.begin( SERIAL_BAUD_RATE ); // Listen on serial connection for messages from the pc

 pinMode(dirPin, OUTPUT);
 pinMode(stepperPin, OUTPUT);
 pinMode( enablePin, OUTPUT );
 digitalWrite( enablePin, LOW );
}
 void step(boolean dir,int steps){
 digitalWrite(dirPin,dir);
 digitalWrite( enablePin, HIGH );

 delay(50);
	for(int i=0;i<steps;i++){
		digitalWrite(stepperPin, HIGH);
		delay(5);
		digitalWrite(stepperPin, LOW);
		delay(5);
	}
}
void loop(){
 step(true,100);
 delay(500);
 step(false,100);
 delay(500);
}
