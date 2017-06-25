// tensione batteria 12V

unsigned int ADCValue;
double A1_Voltage;
double Vbat;
double Vcc;
int percCarica;
#define ADCconversionFactor 0.004887585532746823069403714565 
#define VbatRatio 2.843   //calibrato con Voltmetro ai morsetti della batteria
void setup() {
  // declare the ledPin as an OUTPUT:

  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  //Vcc = readVcc()/1000.0;
  ADCValue = analogRead(1);
  A1_Voltage = ADCValue * ADCconversionFactor ; //Vbat 13.9 >> Va1 = 4,7v >> ADC =1018
  Vbat= A1_Voltage * VbatRatio;
percCarica= PercBattCarica(Vbat);
  Serial.print( " ADC: ");Serial.print( ADCValue, DEC);
  Serial.print( ", Voltage "); Serial.print( A1_Voltage, DEC );
  Serial.print( " , Vbat "); Serial.print( Vbat, DEC );
  ;Serial.print( ",  "); Serial.print(percCarica , DEC );Serial.println( "%");
  delay(700);
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}

int PercBattCarica(double Vbat){
  int c;
  c=0;
  if (Vbat > 12.9){ //2.15v  per elemento = 100%
    c=  100 ;
    }else
      if (Vbat > 12.6) { //2.1v  per elemento = 80%
        c=80;
        }else 
          if (Vbat > 12.3) { //2.05v  per elemento = 60%
            c=60;
          }else 
            if (Vbat > 12.18) { //2.03v  per elemento = 50%
              c=50;
             }else 
                if (Vbat > 11.88) { //1.983v  per elemento = 40%
                  c=40;
                  }else 
                    if (Vbat > 11.7) { //1.983v  per elemento = 30%
                      c=30;
                      }else 
                        if (Vbat > 11.46) { //1.913v  per elemento = 20%
                          c=20;
                         }else 
                             if (Vbat > 11.1) { //1.853v  per elemento = 10%
                                c=10;
                             }else 
                                c=0;
return c;
}





