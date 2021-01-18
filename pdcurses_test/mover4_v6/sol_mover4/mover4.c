/****************************************************************************
* mover4.c
*
*	Author				Date			Version
*	Serge Hould			19 March 2019	v1.0.0	Tested ok
*	Serge Hould			3  March 2020	v1.0.1	Add file open
*	Serge Hould			29  March 2020	v1.1.0	Add timer task
 *	Serge Hould			15 May 2020		v2		Tested with pdcurses and pthread for windows
											   see 247-611-VA Embedded Operating Systems\W21\Labs\sol_intro_multithread_ncurse
*	Serge Hould			25 May 2020		v3		All task were added and tested. Works fine. Add  #ifdef _WIN32
*	Serge Hould			17 Dec 2020		v3.0.1	Move fd_s and fp from mover4.c DOES NOT WORK
*												TESTED OK ON BBB
*	Serge Hould			23 Dec 2020				TESTED OK ON MOVER4 at Vanier
*	Serge Hould			23 Dec 2020				Ncurse white background
 *	Serge Hould			24 Dec 2020				Add task_controller.h.
 *												Move fp, fd_s and init_files() inside task_controller.c Tested OK on both VS and BBB
*	Serge Hould			24 Dec 2020		v3.0.2	add call to close_files()
*	Serge Hould			15 Jan 2021		v3.0.2	add call init_ncurses()
*
*		
*
 *		Bitrate should be 500000 for Mover4
 *		Command to copile:
 *							see build file
 *
 *
 *			Note: The master (original) files are always on the SD card
 *					Files on my PC are copies.
 *
 *****************************************************************************/


#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#include <ncurses.h>
#include <sys/ioctl.h>
 //#include <sys/time.h>
 //#include <libgen.h>
#endif
#include "../header/can.h"
#include "../header/task_controller.h"
#include "../header/ncurses_init.h"
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



int main(int argc, char **argv)
{
	 /*Ncurse config */
	init_ncurses();  
	
	/**** rx-tx CAN init ***********/
#ifndef _WIN32
	open_socket(); // CAN socket init
#endif
	
	startTasksControllerRx();// combined tasks pTask_Controller and pTask_Rx

	pthread_joinControllerRx();  

	close_files();

    exit(EXIT_SUCCESS);
}
