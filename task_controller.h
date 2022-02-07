 /*
 *	File: task_controller.h
 * 		Move all prototypes and macros pertaining to task_controller from public.h into task_controller.h
 *		public.h now contains only public prototypes exlcuding task_controller
 *
 *	Author				Date			Version
 *	Serge Hould			24 Dec 2020		1.0.0		Creates task_controller.h. Split public.h
 *													Add error setter-getter print
 *	Serge Hould			24 Dec 2020		1.0.1		add void close_files(void)
 *	Serge Hould			8 June 2021		1.0.2		acceleration, speed
 *	Serge Hould			8 June 2021		1.0.3		add  set_accel() and associated macros
 * 
 */

#ifndef TASK_CONTROLLER_H
#define TASK_CONTROLLER_H


#define JOG_MODE	0
#define NORM_MODE	1
#define TEST_MODE	2
#define RESET_ERROR	0xd0
#define EN_MOTOR	0xd1
#define	GRIP_DIS	0x00
#define	GRIP_CLOSE	0x02
#define	GRIP_OPEN	0x03

 /*		Acceleration-deceleration parameters															*/
 /* ACCEL_DECEL represents the acceleration and deceleration.  The higher ACCEL_DECEL is, the higher the acceleration-deceleration */
 /* SPEED_MAX represents the maximum speed.  This is when the acceleration stops and the speed is constant */
 /* SPEED_MIN represents the minimum speed. This is when the deleration stops and the speed is constant	*/
 /* SLOW_DEGREE is the angle difference when the deceleraton starts										*/
//#define 	ACCEL_DECEL	0.1		// decrease of the step at each loop 
#define 	ACCEL_DECEL	0.4		// high acceleration-deceleration
//#define 	ACCEL_DECEL	10		// too much acceleration-deceleration - overcurrent
#define 	SPEED_MAX	60.0	// high speed
//#define 	SPEED_MAX	120.0	// too much speed  - overcurrent
#define		SPEED_MED	30.0
#define		SPEED_SLOW	10.0
//#define 	SPEED_MIN	5.0	// Caps it and cannot be set over. 
//#define 	SPEED_MIN	8.0	// Faster 
#define 	SPEED_MIN	0.0	
//#define 	SLOW_DEGREE	650.0	// where the movement is slowed down 10 degrees*65tics/degrees 
//#define 	SLOW_DEGREE	1300.0	// where the movement is slowed down 20 degrees*65tics/degrees
#define 	SLOW_DEGREE	0.0	// Never decelerates
/* Assuming a DELAY_LOOP of 5 mS*/
#define		ACCEL_TIME_MIN		1
#define		ACCEL_TIME_100MS	20
#define		ACCEL_TIME_500MS	100
#define		ACCEL_TIME_1000MS	200


typedef struct
{
	double data[4];		//angle 
}kin_f;

typedef struct
{
	int data[4];		// position
}kin_i;


/*Prototype Area*/
int get_keyb_f(int s);
void set_keyb_f(int , int);
void set_gripper(int val);
int get_gripper(void);
//kin_f get_pv_angles(void);
void clear_error(void);
void enable_motor(void);
void resetJointsToZero(void);

kin_f get_all_pv_angles(void);
void set_all_pv_angles(kin_f);
void set_pv_angle(int i, double _angle);
double get_pv_angle(int nb);
double get_sp_angle(int nb);
void set_sp_angle(int nb, double _angle);
kin_f get_all_sp_angles(void);
void set_all_sp_angles(kin_f _angles);
void set_all_curr_angles(kin_f _angles);
kin_f get_all_curr_angles(void);
double get_curr_angle(int nb);
void startTasksControllerRx(void);
void pthread_joinControllerRx(void);
void delay_ms(int);
void set_warnings(char* str);
int get_warnings(char* str); 
void init_files(void);
void print_warnings(int v, int h);
void set_errors(char* str);
int get_errors(char* str);
void print_errors(int v, int h);
void close_files(void);
void set_accel_decel(double base, double shld, double elbow, double wrist);
void set_speed_max(double base, double shld, double elbow, double wrist);
void set_speed_min(double base, double shld, double elbow, double wrist);
void set_slow_degree(double base, double shld, double elbow, double wrist);
void set_accel(double base, double shld, double elbow, double wrist);


#endif
