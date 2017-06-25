//Henry's Bench
//Arduino Sound Detection Sensor Module

int soundDetectedPin = 10; // Use Pin 10 as our Input
int soundDetectedVal = HIGH; // This is where we record our Sound Measurement
boolean bAlarm = false;

unsigned long lastSoundDetectTime; // Record the time that we measured a sound


int soundAlarmTime = 500; // Number of milli seconds to keep the sound alarm high


void setup ()
{
  Serial.begin(115200);  
  pinMode (soundDetectedPin, INPUT) ; // input from the Sound Detection Module
}
void loop ()
{
	delay(100);
  soundDetectedVal = digitalRead (soundDetectedPin) ; // read the sound alarm time
 		Serial.print(10*(analogRead(0)-500));
		Serial.print(",");
		Serial.println(soundDetectedVal*500);
 
  if (soundDetectedVal == LOW) // If we hear a sound
  {
  
    lastSoundDetectTime = millis(); // record the time of the sound alarm
    // The following is so you don't scroll on the output screen
    if (!bAlarm){

      Serial.println("LOUD, LOUD");
      bAlarm = true;
    }
  }
  else
  {
    if( (millis()-lastSoundDetectTime) > soundAlarmTime  &&  bAlarm){
      Serial.println("quiet");
      bAlarm = false;
    }
  }
}