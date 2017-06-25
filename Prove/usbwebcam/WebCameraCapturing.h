#ifndef WEBCAMERACAPTURING_H_
#define WEBCAMERACAPTURING_H_

//#define Screen_160x120
#define Screen_176x144

#ifdef Screen_160x120
	#define LEFT_PIXEL		60
	#define TOP_PIXEL		80
	#define RIGHT_PIXEL		179
	#define BOTTOM_PIXEL	239
	#define TOTAL_PIXELS	19200	//120x160=19200
	#define FRAME_INDEX		5		//120x160
	#define EP_SIZE			2		//512 bytes
#endif // Screen_160x120

#ifdef Screen_176x144
	#define LEFT_PIXEL		48
	#define TOP_PIXEL		72
	#define RIGHT_PIXEL		191
	#define BOTTOM_PIXEL	247
	#define TOTAL_PIXELS	25344	//176x144=25344
	#define FRAME_INDEX		4		//176x144
	#define EP_SIZE			3		//1024 bytes
#endif // Screen_176x144

#define LOG_SIZE			(3000)
#define VIDEO_FRAME_SIZE	(TOTAL_PIXELS/2)

#define WHITE				RGB(0xFF, 0xFF, 0xFF)
#define RED					RGB(0xFF, 0x00, 0x00)
#define GREEN				RGB(0x00, 0xFF, 0x00)
#define BLUE				RGB(0x00, 0x00, 0xFF)
#define BLACK				RGB(0x00, 0x00, 0x00)

#endif /* WEBCAMERACAPTURING_H_ */