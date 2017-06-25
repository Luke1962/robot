//*******************************************************************************
//*	Touch screen definitions
//*******************************************************************************
//*	Jan  4,	2009	<MLS> Started merging touchShield and touchSlide
//*	Jan  4,	2009	<MLS> Changed touchShield touchscreen_process_x args to int to be compatible with slide
//*******************************************************************************

#ifndef _TOUCHSCREEN_H_
#define _TOUCHSCREEN_H_


//#ifndef _HARDWARE_DEF_H_
//	#include	"HardwareDef.h"
//#endif

//#include "pindefs.h"
//#include "binary.h"

typedef struct
{
	unsigned int x; /*!< X coordinate */
	unsigned int y; /*!< Y coordinate  */
}POINT; 

#define	xDir	0
#define	yDir	1
#define	yes		1
#define	no		0
#define TOUCHSCR_IS_CALIBRATED 20 //!< Variable, that if changed will force a calibrate 

#ifdef _TOUCH_SLIDE_
	//pin defines
	#define	XPLUS_PIN		PF7
	#define	YPLUS_PIN		PF6
	#define	YMINUS_PIN		PF5
	#define	XMINUS_PIN		PF4

	#define	XPLUS			B01000111	//10-bit precision
	#define	YPLUS			B01000110
	#define	YMINUS			B01000101
	#define	XMINUS			B01000100
#endif
#ifdef _TOUCH_STEALTH_
	//pin defines
	#define	XPLUS_PIN	PF3
	#define	XMINUS_PIN	PF1
	#define	YPLUS_PIN	PF2
	#define	YMINUS_PIN	PF0

	#define	YPLUS		B01100010	//left adjusted result in ADCH (8-bit precision)
	#define	XPLUS		B01100011
	#define	XMINUS		B01100001
	#define	YMINUS		B01100000
#endif


#ifdef __cplusplus
extern "C"{
#endif

//*******************************************************************************
//*	Functions
void	delay_ms(unsigned char x);
char	touch_calibrate(void);
char	touch_getCursor(POINT *p);
char	touch_get_cursor(POINT *p);
char	touch_getTouched(POINT *p);

void    touchscreen_isr(void);

struct touchscreen_device {
  void (*init)(void);

  void (*setup_x)(void);
  void (*setup_y)(void);
  void (*setup_z1)(void);
  void (*setup_z2)(void);

  unsigned int (*process_x)(unsigned int adc_value);
  unsigned int (*process_y)(unsigned int adc_value);
  unsigned int (*process_z1)(unsigned int adc_value);
  unsigned int (*process_z2)(unsigned int adc_value);

  void (*enable)(void);
  void (*disable)(void);
  void (*resume)(void);
  void (*suspend)(void);
};

extern struct touchscreen_device *touchscreen;

#ifdef __cplusplus
} // extern "C"
#endif

#endif



