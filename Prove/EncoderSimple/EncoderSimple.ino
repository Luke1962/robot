// The pin the encoder is connected
int encoder_in = 2;

// Initialize the counter
volatile unsigned int pulses = 0;

void count() {
  // This function is called by the interrupt
  pulses++;
}

void setup() {
  pinMode(encoder_in, INPUT);
  attachInterrupt(0, count, RISING);
}

void loop() {
  // Here you can output your counter value e. g. once a second
  //
  //
  delay(1000);
}