
//////////////////////////////////////////////////////////////////////////
// COSTANTI
//////////////////////////////////////////////////////////////////////////
#define DEG2RAD 0.0174533f  // pi/180
#define RAD2DEG 57.2957795f  //= 180/PI

// parametri meccanici-------------------
#define ROBOT_WEEL_DISTANCE	0.342f						// distanza in m tra le due ruote
#define ROBOT_WHEEL_DIAMETER 0.118f	//DIAMETRO RUOTE



// cinematica --------------------------------
#define ROBOT_ROTATION_CIRCUMPHERENCE_CM =	109.955f		// circonferenza = pi*ROBOT_WEEL_DISTANCE=109,956cm


//Nuove costanti
#define ROBOT_M2STEPS  3744.82219040// was 3536.7765f	 //   1m/s=ROBOT_MOTOR_STEPS_PER_M/s  
#define ROBOT_STEPS2M	0.000267035376	// reciproco di  ROBOT_M2STEPS  3744.82219040

#define ROBOT_RAD2STEPS 655.3439f // 3800 step/giro >> 3800/2*Pi= 604.789 rad2step// valore teorico ma non va bene	s
#define ROBOT_STEPS2RAD 0.00152591639290454980964955956712  //reciproco di ROBOT_RAD2STEPS


// lineare
//#define ROBOT_MOTOR_STEPS_PER_CM	37.44f	//	42.44f				//	valore di 78 trovato sperimentalmente impostando 50cm di distanza)
//#define ROBOT_MOTOR_MM_PER_STEP 	0.267035375555  //	42.44f				//	valore di 78 trovato sperimentalmente impostando 50cm di distanza)


//#define ROBOT_MOTOR_STEPS_PER_M		3744.8219f	//	42.44f				//	valore di 78 trovato sperimentalmente impostando 50cm di distanza)
//#define ROBOT_MOTOR_CLOCK_PER_CMs	23562.677f // attesa  in uSec tra 2 IMPULSI per velocità di 1cm/s
//#define ROBOT_MOTOR_CLOCK_PER_Ms	235.62677f // attesa  in uSec tra 2 IMPULSI per velocità di 1m/s

// angolare
//696 esegue un po' più di mezzo giro anzichè uno intero
//#define ROS_VELOCITYLINEAR2MOTORCLOCK ROBOT_MOTOR_CLOCK_PER_CMs/100 // Velocity in m/s  
//#define ROS_VELOCITYANGULAR2MOTORCLOCK	428.6f	// 1e6/ROBOT_MOTOR_STEPS_PER_RADIANT; Angular velocity in rad/sec 




//#define ROBOT_MOTOR_STEPS_PER_DEG	24.0f		// 24 calcolato empiricamente e va bene  (era	23.8237 )
//#define ROBOT_MOTOR_STEPS_PER_RADIANT ROBOT_RAD2STEPS   //was 806.0f // vale per le due ruote che si muovono in direzioni opposte


//#define ROBOT_MOTOR_STEPS_PER_RADIANT 604.789 // valore teorico ma non va bene			342	// con 1375 fa un po' piu di 180° // 371.36155//=ROBOT_MOTOR_STEPS_PER_ROTATION /2*pi


 //parametri accelerazione motori principali





//~ #define LDS_ARRAY_SIZE 180		// una cell per grado
//~ #define LDS_MAX_DISTANCE_CM 300
//~ #define LDS_TIMEOUT_DISTANCE_CM 10000


#define SONAR_ARRAY_SIZE 180		// una cell per grado
#define ANALOG_PORTS 5


//#define ROBOT_MOTOR_CLOCK_microsecondMIN 2500	//1e6/2500= 400 Step/s = 9,4 cm/s velocità max
//#define ROBOT_MOTOR_CLOCK_microsecondMAX 6000	//1e6/6000= 167 Step/sec =3.9 cm/sec; velocità min; 4000 = 250 Step/s	 12400 ck velocità min 
#define ROBOT_MOTOR_CLOCK_microsecondMIN 2200	//1e6/2200= 465 Step/s =12.8 cm/s velocità max
#define ROBOT_MOTOR_CLOCK_microsecondMAX 9000	//1e6/6000= 167 Step/sec =3.9 cm/sec; velocità min; 4000 = 250 Step/s	 12400 ck velocità min 
#define ROBOT_MOTOR_STEPSPERSECOND_MIN 111  // 1E6 / 9000
#define ROBOT_MOTOR_STEPSPERSECOND_MAX 455  // 1E6 / 2200
#define MOTOR_MINIMUM_LINEAR_VELOCITY 0.01	//ROBOT_MOTOR_STEPSPERSECOND_MIN /ROBOT_M2STEPS  //   3.1 cm/sec
#define MOTOR_MAXIMUM_LINEAR_VELOCITY ROBOT_MOTOR_STEPSPERSECOND_MAX /ROBOT_M2STEPS  //  12.8 cm/sec
#define MOTOR_MINIMUM_ANGULAR_VELOCITY  ROBOT_MOTOR_STEPSPERSECOND_MIN/ROBOT_RAD2STEPS // 0.18 rad/sec
#define MOTOR_MAXIMUM_ANGULAR_VELOCITY  ROBOT_MOTOR_STEPSPERSECOND_MAX/ROBOT_RAD2STEPS // 0.75

//#define ROBOT_MOTOR_VELOCITY2HZ = 4244, 12

#define MAX_TARGET_DISTANCE_CM 500	//max distanza da percorrere per un singolo comando
#define ROBOT_SONAR_SCAN_SPEED_DEFAULT 200


// parametri dinamici-------------------
#define ACCEL_LOOP_MICROS_WAIT 5					// Valori ok 5, 10 
#define ROBOT_MOTOR_CLOCK_ACCEL 6					// di quanti microsecondi aumenta o diminuisce in fase accelerativa il clock motori  was 5  // tra 8 e 5 ok
#define ROBOT_MOTOR_CLOCK_DECEL 20					// di quanti microsecondi aumenta o diminuisce in fase accelerativa il clock motori  was 5  // tra 8 e 5 ok

//la ruota che ho costruito ha 8 tacche >> 16 transizioni per  giro (=200 step )
#define ROBOT_MOTOR_STEPS_PER_ENCODERTICK 8.33 //8.33 è una prova empirica,   was 12.5f			//  = 200 steps/giro    e 16 su/giu (tick ) dell'encoder per giro ==> 200/16=12,5


//posizione home (sala dietro la mia sedia)
#define GPSHOME_LAT  45.471633
#define GPSHOME_LNG  9.124051
// CHIAVARI  LAT 44.326953  LNG 9.289679
