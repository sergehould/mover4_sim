 /*
 *	File: task_controller.h
 * 		Move all prototypes and macros pertaining to task_controller from public.h into task_controller.h
 *		public.h now contains only public prototypes exlcuding task_controller
 *
 *	Author				Date			Version
 *	Serge Hould			24 Dec 2020		1.0.0		Creates task_controller.h. Split public.h
 *													Add error setter-getter print
 *	Serge Hould			24 Dec 2020		1.0.1		add void close_files(void)
 * 
 */

#ifndef TASK_CONTROLLER_H
#define TASK_CONTROLLER_H


#define JOG_MODE	0
#define NORM_MODE	1
#define RESET_ERROR	0xd0
#define EN_MOTOR	0xd1
#define	GRIP_DIS	0x00
#define	GRIP_CLOSE	0x02
#define	GRIP_OPEN	0x03


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


#endif
