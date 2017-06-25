#define Pin_Rele1	48

void setup()
{
  pinMode(Pin_Rele1, OUTPUT);
  Serial.begin(9600); // open serial
  Serial.println("Rele Test");
}

void loop()
{

       Serial.println("Relay on");
      digitalWrite(Pin_Rele1, HIGH);
      delay(1500);
     Serial.println("Relay off");
       digitalWrite(Pin_Rele1, LOW);
     delay(2500);
   
}
