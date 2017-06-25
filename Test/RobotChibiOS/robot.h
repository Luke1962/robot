#if !defined(__ROBOTVAR_H__)
#define __ROBOTVAR_H__
#include "hw_config.h"

#define ANALOG_PORTS 5
#define ROBOT_MOTOR_CLOCK_microsecondMIN 500		// 1300 ck max velocità
#define ROBOT_MOTOR_CLOCK_microsecondMAX 12000 //	 12400 ck min velocità


enum commandStatus_t {pending, completed };

struct position_t{
	double x;
	double y;
	double r;	//angolo di rotazione rispetto al nord
};
struct cmd_t {
	unsigned long	speed; // clock motori in millisecondi
	bool	enL;
	bool	enR;
	bool	cwL;
	bool	cwR;
	position_t	target;
	commandStatus_t commandStatus;
};
struct sensors_t{
	double sonar[360];	//distanza rilevata in funzione dell'angolo
	int light;
	long analog[5];
};
struct irproxy_t{
	unsigned fw: 1;	//Forward
	unsigned fl: 1;	//Forward left
	unsigned fr: 1; //Forward right
	unsigned bk: 1; //back
	unsigned pirFw:	1; //  PID11
	unsigned pirTop:1; // PIR emisferico
};
struct dim_t{	//dimensioni fisiche
	float BodyRadius;
	float WeelRadius;
};

typedef int speed_t; 

struct parameter_t{
	int maxspeed_ck;	//clock col valore piu basso
	int minspeed_ck;	//clock col valore piu alto
};	
class robot_c{
public:
	robot_c();		//constructor
public:	enum direction_t {fw, bk, left, right };
public:		dim_t		dim;	// dimensions
public:	    position_t	pos;		//current position
public:		cmd_t		lastCommand;	// last pending command
public:		sensors_t	sensors;		// sensors readings
public:		irproxy_t	irproxy;
public:		parameter_t	parameter;

public:		void executecommand();
public:		void stop();
public:		void goFW(speed_t speed);
public:		void goBK(speed_t speed);
public:		void goCW(speed_t speed);
public:		void goCCW(speed_t speed);
} ;

extern struct robot_c robot;

#endif // __ROBOTVAR_H__