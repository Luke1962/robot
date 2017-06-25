#include <ChibiOS_AVR.h>
#include <utility/hal.h>
#include <utility/hal.h>
#include <ChibiOS_Notifications.h>
//#include <chprintf.h>

#include "arduino.h"

static msg_t Thread1(void *arg);
static msg_t Thread2(void *arg);
static msg_t Thread3(void *arg);
int main(void);
class LEDData: public NotifierMsg<LEDData> {
public:
	uint8_t pin;
	bool_t set;
};

static const SerialConfig serial_config = { 115200, 0, USART_CR2_STOP1_BITS
		| USART_CR2_LINEN, 0 };

Notifier<LEDData> led_notifier1;
Notifier<LEDData> led_notifier2;

/*
 * Receive data from led_notifier1
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {
	Listener<LEDData, 5> listener(&led_notifier1, 1);
	LEDData *d;

	(void) arg;
	chRegSetThreadName("thd1");

	chprintf((BaseSequentialStream *) &SD2, "sizeof(listener) %d\r\n", sizeof(listener));

	while (TRUE) {
		chEvtWaitAny(ALL_EVENTS );
		while ((d = listener.get()) != NULL) {

			if (d->set)
				palSetPad(GPIOD, d->pin);
			else
				palClearPad(GPIOD, d->pin);

			listener.release(d);
		}
	}

	return 0;
}

/*
 * Receive data from led_notifier2
 */
static WORKING_AREA(waThread2, 128);
static msg_t Thread2(void *arg) {
	Listener<LEDData, 5> listener(&led_notifier2, 1);
	LEDData *d;

	(void) arg;
	chRegSetThreadName("thd2");

	while (TRUE) {
		chEvtWaitAny(ALL_EVENTS );
		while ((d = listener.get()) != NULL) {

			if (d->set)
				palSetPad(GPIOD, d->pin);
			else
				palClearPad(GPIOD, d->pin);

			listener.release(d);
		}
	}

	return 0;
}

/* print the leds to serial */
static WORKING_AREA(waThread3, 512);
static msg_t Thread3(void *arg) {
	Listener<LEDData, 2> listener1(&led_notifier1, 1);
	Listener<LEDData, 2> listener2(&led_notifier2, 2);
	LEDData *d;
	eventmask_t evt;

	(void) arg;
	chRegSetThreadName("thd3");

	while (TRUE) {
		evt = chEvtWaitAny(ALL_EVENTS );

		if (0x01 & evt) {
			while ((d = listener1.get()) != NULL) {

				if (d->set)
					chprintf((BaseSequentialStream *) &SD2,
							"%6d palSetPad(GPIOD, %d);\r\n", chTimeNow(),
							d->pin);
				else
					chprintf((BaseSequentialStream *) &SD2,
							"%6d palClearPad(GPIOD, %d);\r\n", chTimeNow(),
							d->pin);

				listener1.release(d);
			}
		}

		if (0x02 & evt) {
			while ((d = listener2.get()) != NULL) {

				if (d->set)
					chprintf((BaseSequentialStream *) &SD2,
							"%6d palSetPad(GPIOD, %d);\r\n", chTimeNow(),
							d->pin);
				else
					chprintf((BaseSequentialStream *) &SD2,
							"%6d palClearPad(GPIOD, %d);\r\n", chTimeNow(),
							d->pin);

				listener2.release(d);
			}
		}

	}

	return 0;
}

int main(void) {
	halInit();
	chSysInit();

	palSetPadMode(GPIOC, 1, PAL_MODE_OUTPUT_PUSHPULL);

	/*
	 * Activates the serial driver 2 using the driver default configuration.
	 * PA2(TX) and PA3(RX) are routed to USART2.
	 */
	sdStart(&SD2, &serial_config);
	palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

	chprintf((BaseSequentialStream *) &SD2, "sizeof(LEDData) %d\r\n", sizeof(LEDData));
	chprintf((BaseSequentialStream *) &SD2, "sizeof(led_notifier1) %d\r\n", sizeof(led_notifier1));

	/*
	 * Creates the example thread.
	 */
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1,
			NULL);
	chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO + 2, Thread2,
			NULL);
	chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, Thread3, NULL);

	chThdSleepSeconds(5);

	LEDData *d;
	while (TRUE) {
		palSetPad(GPIOC, 1);
		d = led_notifier1.alloc();
		if (d != NULL) {
			d->pin = GPIOD_LED3;
			d->set = true;
			led_notifier1.broadcast(d);
		}

		d = led_notifier2.alloc();
		if (d != NULL) {
			d->pin = GPIOD_LED5;
			d->set = true;
			led_notifier2.broadcast(d);
		}

		chThdSleepMilliseconds(500);

		d = led_notifier1.alloc();
		if (d != NULL) {
			d->pin = GPIOD_LED4;
			d->set = true;
			led_notifier1.broadcast(d);
		}

		chThdSleepMilliseconds(500);

		d = led_notifier1.alloc();
		if (d != NULL) {
			d->pin = GPIOD_LED3;
			d->set = false;
			led_notifier1.broadcast(d);
		}

		d = led_notifier2.alloc();
		if (d != NULL) {
			d->pin = GPIOD_LED5;
			d->set = false;
			led_notifier2.broadcast(d);
		}

		chThdSleepMilliseconds(500);

		d = led_notifier1.alloc();
		if (d != NULL) {
			d->pin = GPIOD_LED4;
			d->set = false;
			led_notifier1.broadcast(d);
		}

		chThdSleepMilliseconds(500);

		d = led_notifier2.alloc();
		if (d != NULL) {
			d->pin = GPIOD_LED5;
			d->set = true;
			led_notifier2.broadcast(d);
		}

		chThdSleepMilliseconds(500);

		d = led_notifier2.alloc();
		if (d != NULL) {
			d->pin = GPIOD_LED5;
			d->set = false;
			led_notifier2.broadcast(d);
		}

		chThdSleepMilliseconds(500);

		d = led_notifier2.alloc();
		if (d != NULL) {
			d->pin = GPIOD_LED6;
			d->set = true;
			led_notifier2.broadcast(d);
		}

		chThdSleepMilliseconds(500);

		d = led_notifier2.alloc();
		if (d != NULL) {
			d->pin = GPIOD_LED6;
			d->set = false;
			led_notifier2.broadcast(d);
		}

		chThdSleepMilliseconds(500);
	}
}

