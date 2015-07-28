#ifndef __TYPES__
#include <avr/interrupt.h>

#define BUFSIZE 32

	#define __TYPES__
	typedef unsigned char  u08;
	typedef signed char  s08;
	typedef unsigned int  u16;
	typedef signed int  s16;
	typedef unsigned long  u32;
	
	typedef struct {
        u16 id;
        u08 data[8];
		u08 size;
	} can_msg;
	
	typedef struct {
		u08 type;
		u08 size;
		u08 data[10];
	} RF_msg;
	
	typedef struct {
		u08 id;
		u08 size;
		u08 data[8];
	} msg;
	
	typedef struct {
		u08 head;	//next FREE spot
		u08 tail;	//oldest DATA
		u08 data[BUFSIZE];
	} circ_buf;

#endif
