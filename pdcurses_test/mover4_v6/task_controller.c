/*
*		To convert into object file
*		Object file given to students
*
*	Author				Date			Version
*	Serge Hould			26 Feb 2019		v1.0.0	First version
*	Serge Hould			25 Feb 2020		v1.1.0  
*												Detects whether z of the wrist and elbow are too close to the floor
*												Calculates and limit x-y radius
*												Limits Elbow-Wrist angle between 120 and 600 degrees to avoid 
*												the grip hitting the body of the robot.
*	Serge Hould			25 Feb 2020		v1.1.1	Clean up code.  Add colour to text if in error
*	Serge Hould			28 Feb 2020		v1.2.0	Shoulder-Elbow angle limited between 170 and 550 degrees TESTED 
*												Writes to a log file in pTask_Rx if in error DOES NOT WORK PROPERLY - TO DEBUG MORE
*												Grinds the system to a halt if error  TESTED
*	Serge Hould			3 March 2020	v1.3.0	Restructure file system.  Add set_pv_angles	
*												Rename of functions:
*												get_angle		get_sp_angle
*												set_angle		set_sp_angle
*												get_sp_angles	get_all_sp_angles
*												set_sp_angles	set_all_sp_angles
*												get_pv_angles	get_all_pv_angles
*												set_pv_angles	set_all_pv_angles
*												protect pv_angles
*	Serge Hould			10 March 2020	v1.4.0	Add declaration	and acceleration to controller TETSED OK
*												Sends a frame at a 5mS period rather than 20 mS TETSED OK
*	Serge Hould			10 March 2020	v1.4.1	Improve  controller	to enable acceleration/deceleration 
*												when a sudden change in direction of a motor TESTED OK
*
*	Serge Hould			11 March 2020	v1.4.2	In pTask_Rx, reads the current position and store it in global curr_angles.  
*												Create set_all_curr_angles(), get_all_curr_angles(), get_curr_angle() TESTED OK
*	Serge Hould			12 March 2020	v1.4.3	Clean jumbled up functions 
*	Serge Hould			13 March 2020	v1.4.4	Renamed task_controller and clean up 
*	SH					25 may 2020		v1.4.5	Add  #ifdef _WIN32	 NOT YET TESTED ON THE ROBOT
*	SH					29 may 2020		v1.4.6	Convert usleep() and sleep() into delay_ms()	 NOT YET TESTED ON THE ROBOT
*	SH					1 June 2020		v1.4.7	Clean #include clutter NOT YET TESTED ON THE ROBOT
*	SH					2 June 2020		v1.4.8	replace to_radians() by to_radians2() in order NOT to use kinematic.c NOT YET TESTED ON THE ROBOT
*	SH					15 Dec 2020		v2.0.3	prints current angle to a file so Excel Macro can read it.
*	SH					16 Dec 2020		v2.0.4	Disable some mvprint() calls used for debugging purpose
*												Use a getter-setter to display communicate warnings:
*												get_warnings() and set_warnings() Tested OK
*	Serge Hould			17 Dec 2020		v2.0.5	Move fd_s and fp from mover4.c DOES NOT WORK
*												Merged with task_controller
*												Rename task_ctl_sim.c into task_controller.c TESTED OK ON BBB
*	Serge Hould			23 Dec 2020				TESTED OK ON MOVER4 at Vanier
*	Serge Hould			24 Dec 2020				Add task_controller.h
*										v2.1.0	Add error setter-getter-print.  Tetsed OK in simulation mode.
*												Add DEBUG macro that allows testing on BBB without CAN blocking function in the main loop.
*												Allows the super loop to run.
*												test state and log file reading/writing  OK on both VB and BBB
*												Move fp, fd_s and init_files() inside task_controller.c Tested OK on both VS and BBB
*	Serge Hould			24 Dec 2020		v2.1.1	Declare close_files()
*	Serge Hould			13Jan. 2021		v2.1.2	Enable the following lines inside pTask_Rx when testing on Mover4 :
*												fseek(fd_s, 0, SEEK_SET);
*												fwrite(buf, 1, 1, fd_s);  // writes 1 to the state file
*	Serge Hould			4 March 2021	v2.2.0	Add mutex_skip to allow control over the warning skip counter.  Increased warning delay.
*	Serge Hould			4 March 2021			Tested OK on Mover4 
*												
*	Serge Hould			4 June 2021		v2.2.2	Test and comment acceleration-deceleration in WIN32 only.
*	Serge Hould			5 June 2021		v2.3.0	Improvement to controller algorithm:
*													Change macro name
*													Each motor has its own characteristics: max speed, min speed, accel-decel and slow degree
*													Not tested on Mover4 yet.
*	Serge Hould			8 June 2021					Tested OK on Mover4
*	Serge Hould			8 June 2021		v2.3.1		Add mutex protected functions to access motor characteristics (e.g.  mutex_motor_charact)
*														set_accel_decel(double base,  double shld, double elbow, double wrist)
*														set_speed_max(double base,  double shld, double elbow, double wrist)
*														set_speed_min(double base,  double shld, double elbow, double wrist)
*														set_slow_degree(double base,  double shld, double elbow, double wrist)
*														move_plastic_vial() with different speeds.
*														Clean up: remove extra variables, macros and mvprint()
*														Tested OK on Mover4
*	Serge Hould			8 June 2021		v2.3.2			Reduce speed control to only two functions: set_slow_degree() and set_accel()
*															set_accel() sets an acceleration time from 1 or higher - e.g 1 means 5ms and 100 means 500mS
*															Remove deceleraion by setting SLOW_DEGREE permanently to 0
*															Not tested yet on mover4 - see also task_algo.c
*	Serge Hould			9 June 2021		v2.3.3			step2[4] is initialized to 0 rather than SPEED_MIN - Not tested yet on mover4
*	Serge Hould			Jan. 7, 2022					Tested successfully on mover4
*												
*												
*	TO DO:
*		
*		Solve all warnings
*		Modify kinematics.c , task_controller.c and animation_v2 Excel macro.  See userguide-mover4-v10.pdf p.14
*		
* 			BASE_HGT 	8.625 		to      8.110
* 			HUMERUS 	7.375    	to  	7.480
* 			ULNA		8.625     	to 		8.661
* 			GRIPPER		6         	to 		5.511
*		
*		Sort out the difference between local pv_tics_memo and global pv_tics_mem
*
*			
*
*********************************************************************************************************/

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#include <ncurses.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <libgen.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include "../can-utils/terminal.h"
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include "../can-utils/lib.h"
#endif


#include "../mover4_v6/header/task_controller.h"
#include "../mover4_v6/header/can.h"
#include <math.h>	
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <curses.h>
#include <stdint.h>
#include <ctype.h> 
#include <errno.h>
#include <sys/types.h>
#ifdef _WIN32
#define HAVE_STRUCT_TIMESPEC  // for win32 only. Because TIMESPEC is re-defined inside pthread.h
#endif
#include <pthread.h>

//#define DEBUG			// allows testing on BBB without CAN blocking function in the main loop.



#define		PI	3.1416

/* Defines the minimum distance to the floor for motor 3, motor 4 and the tip */
#define 	Z_TIP_MIN	2.5	// sets the minimum z position of the tip
#define   	Z_M3_MIN	2.5	// sets the minimum z position of the elbow motor
#define 	Z_M4_MIN	2.5	// sets the minimum z position of the wrist motor
/* Sets the maximum reach of the arm*/
#define 	R_MAX		32	//no limit
//#define 	R_MAX		24	//limited

/* link length in inches.  Must also be modified in kinematic.c and animation_v2 Excel */
#define BASE_HGT 	8.625      //base height   
#define HUMERUS 	7.375      //shoulder-to-elbow "bone" 
#define ULNA		8.625      //elbow-to-wrist "bone" 
#define GRIPPER		6           //gripper

#define DELAY_LOOP	5		// control loop delay in mS
#define	WARNING_DELAY	150	// Warning display delay of about 3 seconds for DELAY_LOOP of 5mS

/* static local function prototypes */
static void set_velocity(int id, int vel);
static void set_max_lag(int low, int high);
static void maxMissedCom(int low, int high);
static void reset_error(void);
static int get_pv_tics_mem(int joint);
static void set_pv_tics_mem(int joint, int val);
//static void resetJointsToZero(void);
static void disable_motor(void);
static int enable_motors(void);
static double computeJointPos(int, int);
static int computeTics(int, double);
static kin_f to_cart(kin_f angle);
static void init_KinematicMover(void);
static float to_radians2(float degrees);
static void *pTask_Controller( void *ptr );
static void *pTask_Rx( void *ptr );
static kin_f to_3zs(kin_f angle);
static float to_r(kin_f angle);
static void set_error_f(int);
static int get_error_f(void);


//Global variables
static pthread_t thread_controller;
static pthread_t thread_Rx;
static kin_i pv_tics_mem;
static int jointIDs[4]={16,32,48,64};	// CAN IDs of the robots joints
static kin_f sp_angles ={0},pv_angles, curr_angles;   
static int gripper= GRIP_OPEN;
static double gearScale[4];
static int reset_err=0, en_motor_=0;
static int error_f=0;
static FILE * fd_s;	// state file
static FILE * fp;	//log file
static char buf_w1[250] = { 0 };	//warning messages
static char buf_err[250] = { 0 };	//error messages
static char buf_temp[250];	// temporary buffer
static char buf_temp2[250];	// temporary buffer
/* display flags*/
static  int		w1_f = 0, wb_f=0, ws_f=0, we_f=0, ww_f=0; 

static int flag_debug = 0; //debug
/* skip counter to erase warning messages after a delay*/
static int cnt = 0;
// Motor characteristics
 double accel_decel[4] = { ACCEL_DECEL, ACCEL_DECEL, ACCEL_DECEL, ACCEL_DECEL };
 double speed_max[4] = { SPEED_MAX , SPEED_MAX, SPEED_MAX, SPEED_MAX };
 double speed_min[4] = { SPEED_MIN, SPEED_MIN,SPEED_MIN,SPEED_MIN };
 double slow_degree[4] = { SLOW_DEGREE, SLOW_DEGREE ,SLOW_DEGREE ,SLOW_DEGREE };


/*	Mutexes */
/* Note: scope of variables and mutexes are the same */
static pthread_mutex_t mutex3 = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex5 = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex2 = PTHREAD_MUTEX_INITIALIZER;  //sp
static pthread_mutex_t mutex4 = PTHREAD_MUTEX_INITIALIZER;	// gripper
static pthread_mutex_t mutex6 = PTHREAD_MUTEX_INITIALIZER;  //pv
static pthread_mutex_t mutex_err_f = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_curr = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_w1 = PTHREAD_MUTEX_INITIALIZER; // warning messages
static pthread_mutex_t mutex_err = PTHREAD_MUTEX_INITIALIZER; // error messages
static pthread_mutex_t mutex_test = PTHREAD_MUTEX_INITIALIZER; // debug
static pthread_mutex_t mutex_skip = PTHREAD_MUTEX_INITIALIZER; // skip counter
static pthread_mutex_t mutex_motor_charact = PTHREAD_MUTEX_INITIALIZER; // motor speed, acceleration and slow_degree

void startTasksControllerRx(void){
/* Thread Area	*/
     const char *message = "Thread Task";

     int  iret1,iret2;
	 init_files();

    /* Create independent threads each of which will execute function */
     iret1 = pthread_create( &thread_controller, NULL, pTask_Controller, NULL);
     if(iret1)
     {
         fprintf(stderr,"Error - pthread_create() return code: %d\n",iret1);
         exit(EXIT_FAILURE);
     }
     iret2 = pthread_create( &thread_Rx, NULL, pTask_Rx, NULL);
     if(iret2)
     {
         fprintf(stderr,"Error - pthread_create() return code: %d\n",iret2);
         exit(EXIT_FAILURE);
     }
	 
     /* Wait till threads are complete before main continues. Unless we  */
     /* wait we run the risk of executing an exit which will terminate   */
     /* the process and all threads before the threads have completed.   */
}

void pthread_joinControllerRx(){
     pthread_join( thread_controller, NULL);
	 pthread_join( thread_Rx, NULL);
}



/*******Tasks area *********/

/*
	Task_Controller
	Task that sends command to the robot
*/

static void *pTask_Controller( void *ptr )
{
	float temp_r, elb_wrist_angle, shld_elbow_angle ;
	float diff[4], step[4] = {SPEED_MAX}, step2[4] = {0,0,0,0};
	int reset_step_neg_f[4]={1}, reset_step_pos_f[4] ={1};
	kin_i pv_tics = {0x7d00},sp_tics, pv_tics_memo, curr_tics;
	int j,i, byte_low=0x80, byte_high=0x7d, tics=0, grip = GRIP_OPEN,  cnt2;
	kin_f temp_sp_angles,_pos, zs, temp_curr_angles;
	can_frame_ canframe;	// structure containing can frame data and id
	char buffer[1] = {'0'};
	char debug_buf[1] = { '1' };
	/* Open state file for reading*/
	fseek(fd_s, 0, SEEK_SET);
	/* reads state file in case an error is received */ 
    fread(buffer, 1, 1, fd_s);
	/* Needs to be closed after reading.  Will be re-open for writing in Rx task */
	fclose(fd_s);
	
	/* If the system previously had an error*/
	//if(1){  // for debug
	if(buffer[0] == '1'){
		strcat(buf_temp, "The system cannot restart - Call the teacher to clear the errors -Ctrl C to quit  ");
		set_errors(buf_temp);
		while(1); // Grind the system to a halt
	}
	// init all gearScale values
	init_KinematicMover();
#ifdef _WIN32
	// init all values to 0 degree
	for (j = 0; j < 4; j++) {
		pv_tics.data[j] = 0x7fff;
		temp_sp_angles.data[j] = computeJointPos(j, 0x7fff); // converts tics to angles
	}
#else
#ifndef DEBUG
	/* Send 4 bytes to read the current position of the robot*/
	for (j = 0; j < 4; j++) {
		setFrame6(jointIDs[j], 0x04, 0x80, byte_high, byte_low, 0x23, 0x0);
		pv_tics.data[j] = 0x7d00;
		sp_tics.data[j] = 0x7d00;
	}
	/* Loops 4 times to retrieve the robot response*/
	cnt2 = 4;
	while (cnt2--) {
		canframe = get_can_mess();
		/* Reads frame response in proper order*/
		for (j = 0; j < 4; j++) {
			if (canframe.id == (jointIDs[j] + 1)) {
				tics = (256 * ((int)((unsigned char)canframe.data[2]))) + ((unsigned int)((unsigned char)canframe.data[3]));	// combine the two bytes to the position in encoder tics	
				// sets pv tic values to current tic values
				pv_tics.data[j] = tics;
				// sets temp_sp angles to current value
				temp_sp_angles.data[j] = computeJointPos(j, tics); // converts tics to angles
			}
		}
	}
#else 
	// init all values to 0 degrees
	for (j = 0; j < 4; j++) {
		pv_tics.data[j] = 0x7d00;
		temp_sp_angles.data[j] = computeJointPos(j, 0x7d00); // converts tics to angles
	}
#endif  //DEBUG
#endif  // _WIN32
	/* Memorizes pv tics in case it goes beyond the limits */
	pv_tics_memo = pv_tics;
	
	set_all_sp_angles(temp_sp_angles);
	//set_velocity(0x30,0x80);  // TESTED DOES NOT WORK
	enable_motors();

	while(1){
		temp_sp_angles = get_all_sp_angles();
		temp_curr_angles = get_all_curr_angles(); // NOT TETSED YET
		
		/* Converts angles set point into encoder tics set point */
		/* Converts current joint encoder tics into current angles */
		for(i=0; i<4; i++){
			sp_tics.data[i] = computeTics(i, temp_sp_angles.data[i]); 
			set_pv_angle(i, computeJointPos(i, pv_tics.data[i])); 
			curr_tics.data[i] = computeTics(i, temp_curr_angles.data[i]); // NOT TETSED YET 
			pv_angles.data[i]= computeJointPos(i, pv_tics.data[i]); // should be removed?? same as set_pv_angle()
		}	
		
		/* reads pv  angle */
		/* Shoulder angle - Elbow angle must be between 150 and 570 degrees */
		/* to avoid the Elbow hitting the body of the robot  */
		//shld_elbow_angle = 360 - get_curr_angle(1) - get_curr_angle(2) ; // (180- shoulder) + (180 - elbow) = 360 - shoulder - elbow //SH to test
		shld_elbow_angle = 360 - pv_angles.data[1] - pv_angles.data[2]; // (180- shoulder) + (180 - elbow) = 360 - shoulder - elbow
		
		/* Elbow angle - Wrist angle must be between 120 and 600 degrees */
		/* to avoid the grip hitting the body of the robot  */
		//elb_wrist_angle = 360- get_curr_angle(2)  - get_curr_angle(3); 
		elb_wrist_angle = 360-pv_angles.data[2] - pv_angles.data[3];
		// Finds out all zs 
		zs = to_3zs(get_all_pv_angles()); 
		temp_r = to_r(get_all_pv_angles()); 

		//refresh();
		// if out of range restore pv tics
		if(!((zs.data[2] > Z_TIP_MIN)  && (zs.data[0] > Z_M3_MIN) && (zs.data[1] > Z_M4_MIN) && (temp_r < R_MAX) && (elb_wrist_angle >135 &&  elb_wrist_angle < 585) && (shld_elbow_angle >170 &&  shld_elbow_angle < 720-170) )){
				pv_tics = pv_tics_memo;
				pthread_mutex_lock(&mutex_skip);
				if (w1_f == 0) {
					cnt = 0;
					strcat(buf_temp, "- z or r or angle limits exceeded");
					w1_f = 1;
					//sprintf(buf_temp, "z or r or angle limits exceeded");
					set_warnings(buf_temp);
				}
				pthread_mutex_unlock(&mutex_skip);
		}
		
		// memorizes pv tics
		pv_tics_memo = pv_tics; 

		/* Controller: slowly increase or decrease pv until it almost match sp*/
		for(j=0; j<4; j++){	
		
						/* Regulator with acceleration- deceleration */
						diff[j] = (float)(sp_tics.data[j] - pv_tics.data[j]);

						/* acceleration to full speed*/
						if (diff[j] < -slow_degree[j]){ 
							/* Starts with the minimum speeds - only the first time it enters this if()*/
							if(reset_step_neg_f[j] == 1){
								reset_step_neg_f[j] =0;
								step2[j] = speed_min[j];  // reloads it
							}
							reset_step_pos_f[j] =1;	// enable acceleration positives
							if(step2[j] < speed_max[j]) step2[j] += accel_decel[j];   
							if (step2[j] >= speed_max[j]) {
								step2[j] = speed_max[j];
							}
							
							step[j] = step2[j]; 	// reloads it
							pv_tics.data[j] -= (int)step2[j]; //updates pv

						}
						else if (diff[j] >= slow_degree[j]){
							/* Starts with the minimum speeds - only the first time it enters this if()*/
							if(reset_step_pos_f[j] == 1){
								reset_step_pos_f[j] =0;
								step2[j] = speed_min[j];  // reloads it
							}
							reset_step_neg_f[j] =1;	// enable acceleration negative
							if(step2[j] < speed_max[j]) step2[j] += accel_decel[j];  // 
							if (step2[j] >= speed_max[j]) {
								step2[j] = speed_max[j];
							}
							
							step[j] = step2[j]; 	// reloads it
							pv_tics.data[j] += (int)step2[j];
						}
						/* deceleration to minimum speed when reached some distance (in degrees) from the goal*/	
						else if (fabs(diff[j] ) <= slow_degree[j]){   // where it start  declaration
							reset_step_neg_f[j] =1;	// enable acceleration negative
							reset_step_pos_f[j] =1;	// enable acceleration positives
							if(step[j] > speed_min[j]) step[j] -= accel_decel[j];  //							
							if (step[j] <= speed_min[j]) step[j] = speed_min[j];
							if(diff[j] > 0.0){
								pv_tics.data[j] += (int)step[j];
							}
							else {
								pv_tics.data[j] -= (int)step[j]; 
							}

						}
		}// end of controller for loop
		
		if(get_keyb_f(RESET_ERROR) == 1){  // do a reset error if requested
			reset_error();
			set_keyb_f(RESET_ERROR,0);
			//Copy the memorized position into the pv_tics setPoint position
			for(j=0; j<4; j++){
				pv_tics.data[j]= get_pv_tics_mem(j);
			}
		}
		if(get_keyb_f(EN_MOTOR) == 1){ // do a motor enable if requested
					enable_motors();
					set_keyb_f(EN_MOTOR,0); // resets flag
		}
		
		if(get_error_f() == 1){ // do a motor enable if requested
					strcat(buf_temp, "Call the teacher to reset the errors.  The teacher will recalibrate the robot. Ctrl C to quit     ");
					set_errors(buf_temp);
					while(1); // Grind the system to a halt
		} 
		
		grip =get_gripper();
#ifdef _WIN32
		delay_ms(10); // Windows simulation delay
		//delay_ms(100); // debug only
#else		
#ifndef DEBUG		
		/* Loops to send CAN frames to all motors*/
		for(j=0; j<4; j++){	
					byte_low = pv_tics.data[j] & 0x000000ff; 
					byte_high = pv_tics.data[j]& 0x0000ff00;
					byte_high = pv_tics.data[j] / 256;
					setFrame6(jointIDs[j],0x04,0x80,byte_high,byte_low,0x23,grip);
					delay_ms(DELAY_LOOP);
		}	
#else
		delay_ms(DELAY_LOOP); // debug mode
#endif  //DEBUG
#endif //_WIN32

		//Erase error messages after 3 seconds
		pthread_mutex_lock(&mutex_skip);
		if(cnt++ > WARNING_DELAY){
			sprintf(buf_temp, "");
			set_warnings(buf_temp);
			cnt = 0;
			// resets warning flags 
			w1_f = 0;
			wb_f = 0;
			ws_f = 0;
			we_f = 0;
			ww_f = 0;
		}
		pthread_mutex_unlock(&mutex_skip);

	}// while 1
}

#ifdef _WIN32
/*
	Task_Rx Simulated
	Task that normally receives command from the CAN bus.
	It then sets parameter for Task_Controller
*/
#define KI      0.05     // Capacitor
#define KR      1       // Resistor

void* pTask_Rx(void* ptr1) {
	static double i[4] = { 0 }, ITerm2[4] = { 0 };
	int i_pid;
	kin_f temp_curr_angles;
	
	/* Excel file simulator*/
	FILE* fd_excel;
	char buf1[100];
	fd_excel = fopen("mover4_v6/excel_sim/export.txt", "w");
	//fd_s = fopen("mover4_v6/state", "r+");

	while (1) {
		/* Model to simulate all 4 motors */
		for (i_pid = 0; i_pid < 4; i_pid++) {
			//usleep(10000); // 
			delay_ms(10); // Windows
				 /* Vs = Vr + Vc                                         */
			/* Vc represents speed in RPM - integral term           */
			/* Vs/R represents the pwm input -  initial current     */
			/* Vs = Ri + i/C * integral of ic * dt                  */
			/* Vs - integral of ic * dt = Ri                        */
			/* (Vs - integral  of ic * dt) / R = i                  */
			/* KR* Vs - KI *integral  of ic * dt = i                */
			ITerm2[i_pid] = ITerm2[i_pid] + KI * i[i_pid];  // ITerm2 represents the speed in RPM - Vc voltage
			i[i_pid] = get_pv_angle(i_pid) * KR - ITerm2[i_pid];     // current in a RC circuit  
			temp_curr_angles.data[i_pid] = ITerm2[i_pid];
		}
		set_all_curr_angles(temp_curr_angles);
		/* Prints to file */
		sprintf(buf1, "%f\n%f\n%f\n%f\n%f\n", get_curr_angle(3), get_curr_angle(2), get_curr_angle(1), get_curr_angle(0), get_curr_angle(0));
		fwrite(buf1, 1, strlen(buf1), fd_excel);
		fseek(fd_excel, 0, SEEK_SET);
	}
	//close(s[0]);
	//fclose(fp);
	exit;
}
#else
/*
	Task_Rx
	Task that receive command from the CAN bus.
	It then sets parameter for Task_Controller
*/
void *pTask_Rx( void *ptr1 ){
	int j;
	int tics;
	kin_i temp_tics;
	kin_f temp_curr_angles;
	can_frame_ canframe;	// structure containing can frame data and id
	delay_ms(100);
	char buf[1] = {'1'};  // to be written to state file if error
	/* state file in case an error is received* /
	/* Was closed right before the main loop after reading.  Must be re-open for wriring */
	fd_s = fopen("../state", "r+"); 

	while (1) {

		/* Blocks and waits for a frame*/
		canframe = get_can_mess();
		for(j=0; j<4; j++){
			if(canframe.id == (jointIDs[j]+1)){
				tics = (256 * ((int)((unsigned char)canframe.data[2]))) + ((unsigned int)((unsigned char)canframe.data[3]));	// combine the two bytes to the position in encoder tics	
				set_pv_tics_mem(j, tics);  // Used only for reset zeros
				/*NOT TESTED YET*/
				//temp_tics.data[j]=tics;	
				temp_curr_angles.data[j] =computeJointPos(j,tics); // converts current tics to current angles
				set_all_curr_angles(temp_curr_angles);  // FUNCTION TO BE DEFINED
				
				// 0x02 Velocity lag,  0x08  Comm WDog, 0x10 Pos Lag  0x40 Over Current
				if(canframe.data[0] == 0x02) {
					sprintf(buf_temp2, "Motor %d Vel. lag     \n", j);
					strcat(buf_temp, buf_temp2);
					set_errors(buf_temp);
					fprintf(fp, "Motor %d Vel. lag     \n",j); // logs up
					set_error_f(1);   
					fseek(fd_s, 0, SEEK_SET);
					fwrite(buf, 1, 1, fd_s);  // writes 1 to the state file
				}
				if(canframe.data[0] == 0x08){
					sprintf(buf_temp2, "Motor %d Comm. WDog   \n", j);
					strcat(buf_temp, buf_temp2);
					set_errors(buf_temp);
					fprintf(fp, "Motor %d Comm. WDog   \n",j);
					set_error_f(1);   
					fseek(fd_s, 0, SEEK_SET);
					fwrite(buf, 1, 1, fd_s);  // writes 1 to the state file
				}
				if(canframe.data[0] == 0x10){
					sprintf(buf_temp2, "Motor %d Pos. lag     ", j);
					strcat(buf_temp, buf_temp2);
					set_errors(buf_temp);
					fprintf(fp, "Motor %d Pos. lag     \n",j);
					set_error_f(1);   
					fseek(fd_s, 0, SEEK_SET);
					fwrite(buf, 1, 1, fd_s);  // writes 1 to the state file
				}
				if(canframe.data[0] == 0x40){
					sprintf(buf_temp2, "Motor %d Over current \n", j);
					strcat(buf_temp, buf_temp2);
					set_errors(buf_temp);
					fprintf(fp, "Motor %d Over current \n",j);
					set_error_f(1);   
					fseek(fd_s, 0, SEEK_SET);
					fwrite(buf, 1, 1, fd_s);  // writes 1 to the state file
				}

			}
		}
				/*External CAN messages*/
				// if(frame.can_id == 0x0A0){
					// //mvprintw(17,0,"A0 id received \n");
					// if(frame.data[0] == 0xF0) set_angle1(0x89ff);
					// if(frame.data[0] == 0xF1) set_angle1(0x7D00);
				// } 	

		//out_fflush:fflush(stdout);
		
	}
	//close(s[0]);
	exit;
}
#endif

/***** Private definitions ************/

//disable motors
static void disable_motor(void) {
#ifndef _WIN32
	int i;
	for (i = 0; i < 4; i++) {
		setFrame2(jointIDs[i], 0x01, 0x0a);
		usleep(50000);
	}
#endif
}

//enable motors
static int enable_motors(void) {
#ifndef _WIN32
	int i;
	for (i = 0; i < 4; i++) {

		setFrame2(jointIDs[i], 0x01, 0x09);
		usleep(50000);

	}
#endif
	return 1;
}
/*
Send posH in byte 3, posL in byte 4
Typical: 0x01 0x08 0x00 0x00
Currently the provided position data are not used, the joint is set to zero (0x7D00
for CPR-CAN, 0x0000 for CPR-CAN-V2)
To ensure data integrity this command has to be send twice within the time of 50
ms to take effect.
Length has to be 4.
On some boards: Two acknowledge message are sent: 0x0208
*/
#ifdef _WIN32
static void set_zeros(void) {
	// no hardware involved in WIN32
}
/*
Reset Error
Sets Error Code to 0x04 (Motor not enabled)
Length has to be 2.
Acknowlede message 0x0106 0x001 is sent.
Also set reference
*/
static void reset_error(void) {
	// no hardware involved in WIN32
}

void resetJointsToZero(void) {
	int i;
	disable_motor();			// otherwise the robot will make a jump afterwards (until the lag error stops him)

	// no hardware involved in WIN32

	pthread_mutex_lock(&mutex_skip);
	cnt = 0;
	strcat(buf_temp, "- set joint pos. to zero ");
	set_warnings(buf_temp);
	pthread_mutex_unlock(&mutex_skip);

}


static void set_velocity(int id, int vel) {
	// no hardware involved in WIN32
}

/*
Allowed distance between current
position and setpoint position in
encoder tics.
When value is 0 then this test is
switched of.
Value is saved in EEPROM
Standard Value 1200
*/
static void set_max_lag(int low, int high) {
	// no hardware involved in WIN32
}
/*
0x02 0x30 data-H data-L
Number of cycles without incoming CAN
message before COM error
When value is 0 then this test is
switched of.
Value ist saved in EEPROM
Standard Value 1000
*/
static void maxMissedCom(int low, int high) {
	// no hardware involved in WIN32
}
#else
static void set_zeros(void) {
	//Set to zero
	setFrame4(0x10, 0x01, 0x08, 0x0, 0x0);
	sleep(1);
}
/*
Reset Error
Sets Error Code to 0x04 (Motor not enabled)
Length has to be 2.
Acknowlede message 0x0106 0x001 is sent.
Also set reference
*/
static void reset_error(void) {
	int i;
	//reset zeros p.8 UserGuide
	for (i = 0; i < 4; i++) setFrame2(jointIDs[i], 0x01, 0x06);
	sleep(1);
}

void resetJointsToZero(void) {
	int i;
	disable_motor();			// otherwise the robot will make a jump afterwards (until the lag error stops him)

	for (i = 0; i < 4; i++) {
		setFrame4(jointIDs[i], 0x01, 0x08, 0x0, 0x0);	// first reset command.. but thats not sufficient
		sleep(5);
		// the command has to be sent twice in the time of two seconds to take effect
		setFrame4(jointIDs[i], 0x01, 0x08, 0x0, 0x0);
		sleep(5);
	}

}


static void set_velocity(int id, int vel) {
	setFrame3(id, 0x05, vel, 0x05);
}

/*
Allowed distance between current
position and setpoint position in
encoder tics.
When value is 0 then this test is
switched of.
Value is saved in EEPROM
Standard Value 1200
*/
static void set_max_lag(int low, int high) {
	//0x02 0x31 data-H data-L
	setFrame3(0x2, 0x31, high, low);
}
/*
0x02 0x30 data-H data-L
Number of cycles without incoming CAN
message before COM error
When value is 0 then this test is
switched of.
Value ist saved in EEPROM
Standard Value 1000
*/
static void maxMissedCom(int low, int high) {
	//0x02 0x31 data-H data-L
	setFrame3(0x2, 0x30, high, low);
}
#endif


					
static int get_pv_tics_mem(int joint){
	int tempo;
	pthread_mutex_lock( &mutex3 );
	tempo= pv_tics_mem.data[joint];
	pthread_mutex_unlock( &mutex3 );
	return tempo;
}

static void set_pv_tics_mem(int joint, int val){
	pthread_mutex_lock( &mutex3 );
	pv_tics_mem.data[joint]=val;
	pthread_mutex_unlock( &mutex3 );
}



//***************************************************************
// int ticks:		joint encoder tics
// return value: 	joint position in degrees
static double computeJointPos(int joint, int ticks){
	double gearZero = 32000.0;
	double p = 0;
	p = (ticks - gearZero)/gearScale[joint];
	return p;
}


//***************************************************************
// double pos:		joint position in degree
// return value: 	joint encoder ticks
static int computeTics(int joint, double pos){
	double gearZero = 32000.0;
	int t =  ((int)(pos*gearScale[joint])) + gearZero;
	return t;
}


//***************************************************************
// init all values
static void init_KinematicMover(void)
{
	gearScale[0] = 65.0;		// reduction ratio of the joints: 1° = 65 encoder tics
	gearScale[1] = -65.0;
	gearScale[2] = 65.0;
	//gearScale[3] = -65.0;
	gearScale[3] = -90.0;	// needed some fine tuning

}

static kin_f to_cart(kin_f angle){
	kin_f cart;
	double r;
	/*r= HUMERUS*cos(to_radians2(90-shld)) + ULNA*cos(90-sld-elb) + GRIPPER*cos(90-elb-sld-wris);
	z=BASE_HGT+ 
	y= r*sin(to_radians2(base_angle));
	x= r*cos(to_radians2(base_angle));*/
	r= HUMERUS*cos(to_radians2(90-angle.data[1])) + ULNA*cos(to_radians2(90-angle.data[1]-angle.data[2])) + GRIPPER*cos(to_radians2(90-angle.data[1]-angle.data[2]-angle.data[3]));
	// z
	cart.data[2]=BASE_HGT+ HUMERUS*sin(to_radians2(90-angle.data[1])) + ULNA*sin(to_radians2(90-angle.data[1]-angle.data[2])) + GRIPPER*sin(to_radians2(90-angle.data[1]-angle.data[2]-angle.data[3]));
	// y
	cart.data[1] = r*sin(to_radians2(angle.data[0]));
	// x
	cart.data[0]= r*cos(to_radians2(angle.data[0])); 
	return cart;

}

/* returns distance from base to tip on the x-y plane*/
static float to_r(kin_f angle){
	double r;
	/*r= HUMERUS*cos(to_radians2(90-shld)) + ULNA*cos(90-sld-elb) + GRIPPER*cos(90-elb-sld-wris);*/
	r= HUMERUS*cos(to_radians2(90-angle.data[1])) + ULNA*cos(to_radians2(90-angle.data[1]-angle.data[2])) + GRIPPER*cos(to_radians2(90-angle.data[1]-angle.data[2]-angle.data[3]));
	return r;
}

/* returns distance all 3 z: z motor3, z motor4 and z tip*/
static  kin_f to_3zs(kin_f angle){
	kin_f cart;
	double r;
	// z tip
	cart.data[2]=BASE_HGT+ HUMERUS*sin(to_radians2(90-angle.data[1])) + ULNA*sin(to_radians2(90-angle.data[1]-angle.data[2])) + GRIPPER*sin(to_radians2(90-angle.data[1]-angle.data[2]-angle.data[3]));
	
	// z motor3
	cart.data[0]=BASE_HGT+ HUMERUS*sin(to_radians2(90-angle.data[1]));
	
	// z motor4
	cart.data[1]=BASE_HGT+ HUMERUS*sin(to_radians2(90-angle.data[1])) + ULNA*sin(to_radians2(90-angle.data[1]-angle.data[2])) ;
	return cart;

}

static float to_radians2(float degrees) {
    return (degrees * PI) /  180;
}

static void set_error_f(int m){
	pthread_mutex_lock(&mutex_err_f);
	error_f = m;
	pthread_mutex_unlock(&mutex_err_f);	
}

static int get_error_f(void){
	int temp;
	pthread_mutex_lock(&mutex_err_f);
	temp = error_f;
	pthread_mutex_unlock(&mutex_err_f);
	return temp;
}

/************** Public definitions *****************/

/* Sets the gripper:
	GRIP_CLOSE	
	GRIP_OPEN
*/
void set_gripper(int val){
	pthread_mutex_lock( &mutex4 );
	gripper = val;
	pthread_mutex_unlock( &mutex4 );	

}

int get_gripper(void){
	int tempo;
	pthread_mutex_lock( &mutex4 );
	tempo= 	gripper;
	pthread_mutex_unlock( &mutex4 );	
	return tempo;

}


void set_keyb_f(int s, int state){
	// add mutex
	switch(s){
		case RESET_ERROR: reset_err = state;
		break;
		case EN_MOTOR: en_motor_ = state;
		break;
	}
}

void clear_error(void) {
	set_keyb_f(RESET_ERROR, 1);
	pthread_mutex_lock(&mutex_skip);
		cnt = 0;
		strcat(buf_temp, "- error cleared          ");
		set_warnings(buf_temp);
		//mvprintw(20,0, "error cleared          ");
	pthread_mutex_unlock(&mutex_skip);
}
void enable_motor(void) {
	set_keyb_f(EN_MOTOR, 1);
	//mvprintw(20,0,"motor enable           ");
	pthread_mutex_lock(&mutex_skip);
		cnt = 0;
		strcat(buf_temp, "- motor enabled          ");
		set_warnings(buf_temp);
	pthread_mutex_unlock(&mutex_skip);
}


int get_keyb_f(int s){
// add mutex
	switch(s){
		case RESET_ERROR: return reset_err ;
		break;
		case EN_MOTOR: return en_motor_;
		break;
	}
}




/**************** sp angles******************/
/* returns the SP angle of the specified joint
	A value from 0 to 3 must be provided.
*/
double get_sp_angle(int nb){
	double tempo;
	pthread_mutex_lock( &mutex2 );
	tempo = sp_angles.data[nb];
	pthread_mutex_unlock( &mutex2 );
	return tempo;
}

/* 	Sets angles for joints 0 to 3 */
/*	Base (0) -150 to +150 degrees	*/
/*	Shoulder(1)-50 to +65*/
/*	Elbow (2) -110 to +140 */
/*	Wrist (3) -140 to +140*/
void set_sp_angle(int i, double _angle) {
	pthread_mutex_lock(&mutex2);
	// if(nb == 1) {
		// if(_angle <= 65 && _angle >= -50)	sp_angles.data[nb]= _angle;
	// }
	// else if(_angle <= 110 && _angle >= -110) sp_angles.data[nb]= _angle;
		// start_color();
		// init_pair(1, COLOR_RED, COLOR_BLACK);
		//attron (COLOR_PAIR (2)); //red
	if (i == 0) {
		if (_angle <= 150 && _angle >= -150)	sp_angles.data[i] = _angle;
		else {
			pthread_mutex_lock(&mutex_skip);
			if (wb_f == 0) {
				cnt = 0;
				strcat(buf_temp, "- exceeded angle base ");
				set_warnings(buf_temp);
				//mvprintw(21, 0, "exceeded angle  motor base  at %4.2fdeg.         ", _angle);
				wb_f = 1;
			}
			pthread_mutex_unlock(&mutex_skip);
		}

	}
	else if (i == 1) {
		if (_angle <= 65 && _angle >= -50)	sp_angles.data[i] = _angle;
		else {
			pthread_mutex_lock(&mutex_skip);
			if (ws_f == 0) {
				cnt = 0;
				strcat(buf_temp, "- exceeded angle shoulder ");
				set_warnings(buf_temp);
				//mvprintw(22, 0, "exceeded angle  motor shoulder  at %4.2fdeg.         ", _angle);
				ws_f = 1;
			}
			pthread_mutex_unlock(&mutex_skip);
		}

	}
	else if (i == 2) {
		if (_angle <= 140 && _angle >= -110)	sp_angles.data[i] = _angle;
		else {
			pthread_mutex_lock(&mutex_skip);
			if (we_f == 0) {
				cnt = 0;
				strcat(buf_temp, "- exceeded angle elbow ");
				set_warnings(buf_temp);
				we_f = 1;
			}
			pthread_mutex_unlock(&mutex_skip);
		}
		//else mvprintw(21,0,"exceeded angle  motor elbow  at %4.2fdeg.         ",_angle);

	}
	else if (i == 3) {
		if (_angle <= 140 && _angle >= -140)	sp_angles.data[i] = _angle;
		else {
			pthread_mutex_lock(&mutex_skip);
			if (ww_f == 0) {
				cnt = 0;
				strcat(buf_temp, "- exceeded angle wrist ");
				set_warnings(buf_temp);
				ww_f = 1;
			}
			pthread_mutex_unlock(&mutex_skip);
		}
		//else mvprintw(21,0,"exceeded angle  motor wrist  at %4.2fdeg.         ",_angle);

	}
	//attroff(COLOR_PAIR(2));
	//refresh();
	//angles.data[i]= _angles.data[i];
	pthread_mutex_unlock(&mutex2);
}


/* Returns all four SP angles 
	The return values are inside a kin_f structure
*/
kin_f get_all_sp_angles(void){
	kin_f tempo;
	int i;
	pthread_mutex_lock( &mutex2 );
	for(i=0 ; i< 4 ; i++){
		tempo.data[i] = sp_angles.data[i];
	}
	pthread_mutex_unlock( &mutex2 );
	return tempo;
}


/* 	Sets angles for joints 0 to 3					*/
/*  All angles must be stored in a kin_f strucure	*/
/*	before the call									*/
/*	Base (0) -150 to +150 degrees					*/
/*	Shoulder(1)-50 to +65							*/
/*	Elbow (2) -110 to +140							*/
/*	Wrist (3) -140 to +140							*/
void set_all_sp_angles(kin_f _angles) {

	int i;
	pthread_mutex_lock(&mutex2);
	// start_color();
	// init_pair(1, COLOR_RED, COLOR_BLACK);
	//attron (COLOR_PAIR (2)); //red
	for (i = 0; i < 4; i++) {
		if (i == 0) {
			if (_angles.data[i] <= 150 && _angles.data[i] >= -150)	sp_angles.data[i] = _angles.data[i];
			else {
				pthread_mutex_lock(&mutex_skip);
				if (wb_f == 0) {
					cnt=0;
					strcat(buf_temp, "- exceeded angle base ");
					set_warnings(buf_temp);
					//mvprintw(21, 0, "exceeded angle  motor base  at %4.2fdeg.         ", _angle);
					wb_f = 1;
				}
				pthread_mutex_unlock(&mutex_skip);
			}

		}
		else if (i == 1) {
			if (_angles.data[i] <= 65 && _angles.data[i] >= -50)	sp_angles.data[i] = _angles.data[i];
			else {
				pthread_mutex_lock(&mutex_skip);
				if (ws_f == 0) {
					cnt = 0;
					strcat(buf_temp, "- exceeded angle shoulder ");
					set_warnings(buf_temp);
					//mvprintw(21, 0, "exceeded angle  motor shoulder  at %4.2fdeg.         ", _angle);
					ws_f = 1;
				}
				pthread_mutex_unlock(&mutex_skip);
			}

		}
		else if (i == 2) {
			if (_angles.data[i] <= 140 && _angles.data[i] >= -110)	sp_angles.data[i] = _angles.data[i];
			else {
				pthread_mutex_lock(&mutex_skip);
				if (we_f == 0) {
					cnt = 0;
					strcat(buf_temp, "- exceeded angle elbow ");
					set_warnings(buf_temp);
					we_f = 1;
				}
				pthread_mutex_unlock(&mutex_skip);
			}

		}
		else if (i == 3) {
			if (_angles.data[i] <= 140 && _angles.data[i] >= -140)	sp_angles.data[i] = _angles.data[i];
			else {
				pthread_mutex_lock(&mutex_skip);
				if (ww_f == 0) {
					cnt = 0;
					strcat(buf_temp, "- exceeded angle wrist ");
					set_warnings(buf_temp);
					ww_f = 1;
				}
				pthread_mutex_unlock(&mutex_skip);
			}

		}
		//angles.data[i]= _angles.data[i];
	}
	//attroff(COLOR_PAIR(2));
	pthread_mutex_unlock(&mutex2);
	//refresh();

}


/************* pv angles*******************/

double get_pv_angle(int nb){
	double tempo;
	pthread_mutex_lock( &mutex6 ); //pv mutex
	tempo = pv_angles.data[nb];
	pthread_mutex_unlock( &mutex6 );
	return tempo;
}

void set_pv_angle(int i, double _angle){
	pthread_mutex_lock( &mutex6 );
	pv_angles.data[i]= _angle;
	pthread_mutex_unlock( &mutex6 );
}

kin_f get_all_pv_angles(void){
	kin_f tempo;
	int i;
	pthread_mutex_lock( &mutex6 );
	for(i=0 ; i< 4 ; i++){
		tempo.data[i] = pv_angles.data[i];
	}
	pthread_mutex_unlock( &mutex6 );
	return tempo;
}

void set_all_pv_angles(kin_f _pv_angles){
	int i;
	pthread_mutex_lock( &mutex6 );
	for(i=0 ; i< 4 ; i++){
		pv_angles.data[i]= _pv_angles.data[i];
	}
	pthread_mutex_unlock( &mutex6 );
}	


/************* curr angles*******************/

void set_all_curr_angles(kin_f _angles){
	int i;
	pthread_mutex_lock( &mutex_curr );

	for(i=0 ; i< 4 ; i++){
			curr_angles.data[i]= _angles.data[i];
	}
	pthread_mutex_unlock( &mutex_curr );
}

/* 	Gets all curPV angles for joints 0 to 3				*/
/*  The return values are inside a kin_f structure		*/
kin_f get_all_curr_angles(void){
	kin_f tempo;
	int i;
	pthread_mutex_lock( &mutex_curr );
	for(i=0 ; i< 4 ; i++){
		tempo.data[i] = curr_angles.data[i];
	}
	pthread_mutex_unlock( &mutex_curr );
	return tempo;
}
/* returns the curPV angle of the specified joint
	A value from 0 to 3 must be provided.
*/
double get_curr_angle(int nb){
	double tempo;
	pthread_mutex_lock( &mutex_curr); 
	tempo = curr_angles.data[nb];
	pthread_mutex_unlock( &mutex_curr );
	return tempo;
}


/******* Warning and error messages setter-getter section **********/

void set_warnings(char* str) {
	pthread_mutex_lock(&mutex_w1);
	//mvprintw(22, 0, " set warmoings ");
	strcpy(buf_w1, str);
	pthread_mutex_unlock(&mutex_w1);
}

int get_warnings(char* str) {
	pthread_mutex_lock(&mutex_w1);
	strcpy(str, buf_w1);
	pthread_mutex_unlock(&mutex_w1);
	if (str[0] != 0)return 0; // valid string if none null
	else return -1;
}

/* Prints all current warnings to the line and column specified*/
void print_warnings(int v, int h) {
	pthread_mutex_lock(&mutex_w1);
	mvprintw(v, h, buf_w1);
	pthread_mutex_unlock(&mutex_w1);
}

void set_errors(char* str) {
	pthread_mutex_lock(&mutex_err);
	strcpy(buf_err, str);
	pthread_mutex_unlock(&mutex_err);
}

int get_errors(char* str) {
	pthread_mutex_lock(&mutex_err);
	strcpy(str, buf_err);
	pthread_mutex_unlock(&mutex_err);
	if (str[0] != 0)return 0; // valid string if none null
	else return -1;
}

/* Prints all current errors to the line and column specified*/
void print_errors(int v, int h) {
	pthread_mutex_lock(&mutex_err);
	mvprintw(v, h, buf_err);
	pthread_mutex_unlock(&mutex_err);
}



/*****************************************************/

void init_files(void) {
#ifdef _WIN32
	//fp = fopen("mover4_v6/log", "w+"); // w+ erases the file if already existing!
	//fd_s = fopen("mover4_v6/state", "w+");

	fp = fopen("mover4_v6/log", "r+"); // created inside mover4_v5.  Path required
	fd_s = fopen("mover4_v6/state", "r+");
#else
	//fp = fopen("log", "w+"); // created inside mover4_v6.  
	//fd_s = fopen("state", "w");
	fp = fopen("../log", "r+"); // created inside mover4_v6.  Cannot specify a path otherwise does not work
	fd_s = fopen("../state", "r+");
#endif
	//test for debug only
	//char buf[] = { '1' };
	//fseek(fd_s, 0, SEEK_SET);
	//fwrite(buf, sizeof(char), sizeof(buf), fd_s);  // writes 1 to the state file
	//fprintf(fp, "test log file %d   \n",4);
	//fclose(fp);
}


void close_files(void) {
	fclose(fp); //log
	fclose(fd_s); // state
}

/* Sets a blocking delay in mS*/
void delay_ms(int d) {
#ifdef	WIN32
	Sleep(d);
#else
	usleep(1000 * d);
#endif
}

/*** speed and acceleration section ******/
/* See task_controller.h for macros	 */

void set_accel_decel(double base, double shld, double elbow, double wrist) {
	pthread_mutex_lock(&mutex_motor_charact);
	accel_decel[0] = base;
	accel_decel[1] = shld;
	accel_decel[2] = elbow;
	accel_decel[3] = wrist;
	pthread_mutex_unlock(&mutex_motor_charact);
}

//speed_max set
void set_speed_max(double base, double shld, double elbow, double wrist) {
	
	pthread_mutex_lock(&mutex_motor_charact);
	speed_max[0] = base;
	speed_max[1] = shld;
	speed_max[2] = elbow;
	speed_max[3] = wrist;
	pthread_mutex_unlock(&mutex_motor_charact);
}
//speed_min set
void set_speed_min(double base, double shld, double elbow, double wrist) {
	
	pthread_mutex_lock(&mutex_motor_charact);
	speed_min[0] = base;
	speed_min[1] = shld;
	speed_min[2] = elbow;
	speed_min[3] = wrist;
	pthread_mutex_unlock(&mutex_motor_charact);
}

//slow_degree set
void set_slow_degree(double base, double shld, double elbow, double wrist) {
	pthread_mutex_lock(&mutex_motor_charact);
	slow_degree[0] = base;
	slow_degree[1] = shld;
	slow_degree[2] = elbow;
	slow_degree[3] = wrist;
	pthread_mutex_unlock(&mutex_motor_charact);
}


/* sets an acceleration time from 1 or higher - e.g 1 means 5ms and 100 means 500mS */
void set_accel(double base, double shld, double elbow, double wrist) {
	pthread_mutex_lock(&mutex_motor_charact);
	accel_decel[0] = speed_max[0] / base;
	accel_decel[1] = speed_max[1] / shld;
	accel_decel[2] = speed_max[2] / elbow;
	accel_decel[3] = speed_max[3] / wrist;
	pthread_mutex_unlock(&mutex_motor_charact);
}
