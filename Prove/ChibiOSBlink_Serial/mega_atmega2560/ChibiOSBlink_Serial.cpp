// NO ARDUINO UNO *****************************************************
// USA SERIAL 3
#include <ChibiOS_AVR.h>
#include <util/atomic.h>

#include "arduino.h"

static msg_t Thread1(void *arg);
static msg_t Thread2(void *arg);
static msg_t SerialHandler(void *arg);
//
//
const uint8_t LED_PIN = 13;

volatile uint32_t count = 0;

//------------------------------------------------------------------------------
// thread 1 - high priority for blinking LED
// 64 byte stack beyond task switch and interrupt needs
static WORKING_AREA(waThread1, 64);

static msg_t Thread1(void *arg) {
  pinMode(LED_PIN, OUTPUT);

  // Flash led every 200 ms.
  while (1) {
    // Turn LED on.
    digitalWrite(LED_PIN, HIGH);
    Serial.print("+");
    // Sleep for 50 milliseconds.
    chThdSleepMilliseconds(50);

    // Turn LED off.
    digitalWrite(LED_PIN, LOW);

    // Sleep for 150 milliseconds.
    chThdSleepMilliseconds(950);
  }
  return 0;
}
static WORKING_AREA(waThread2, 64);

static msg_t Thread2(void *arg) {
  // Flash led every 200 ms.
  while (1) {
    // Turn LED on.

    Serial.print(".");
    // Sleep for n milliseconds.
    chThdSleepMilliseconds(2000);
  }
  return 0;
}

static WORKING_AREA(waSerialHandler, 256);
static msg_t SerialHandler(void *arg)
{
  Serial.begin(9600);
  char str[20];
  //  String inputStr = "";
  //  inputStr.reserve(200);
  volatile uint8_t count = 0;
  char inChar = 0;
  while (true)
  {
    if (!Serial.available()) chThdYield();
    else
    {
      do
      {
        inChar = (char)Serial.read();
        if (inChar == 'c')
        {
          str[count] = 0;
          Serial.print("Count:  ");
          Serial.println(count);
          Serial.print("Output string:  ");
          Serial.println(str);
        }
        str[count] = inChar;
        //      inputStr.concat((String)inChar);
        count++;
      } while (Serial.available());
    }
    if (count >= 1)
    {
      str[count] = 0;
      Serial.println(str);
      //  Serial.println(inputStr);
      Serial.flush();
      count = 0;
      //    inputStr = "";
    }
    chThdSleep(1);
  }
}
void setup() {
  // put your setup code here, to run once:
  cli();
  halInit();
  chSysInit();

  Serial.begin(9600);

  chThdCreateStatic(waSerialHandler, sizeof(waSerialHandler), NORMALPRIO + 100, SerialHandler, NULL);
  // start blink thread
  chThdCreateStatic(waThread1, sizeof(waThread1),
                    NORMALPRIO + 100, Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2),
                    NORMALPRIO + 200, Thread2, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
  chThdSleep(10);

}


