void setup() 
{ 
pinMode(13, OUTPUT); // define 13 feet as digital output interface
}
void loop() {
digitalWrite(13, HIGH); // Open the laser head
delay(1000); // delay a second
digitalWrite(13, LOW); // close the laser head
delay(1000); // delay a second
}
