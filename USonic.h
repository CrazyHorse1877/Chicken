#include <typedefs.h>


//#define UPDATE_DELAY 488 //1sec at 8Mhz
//#define UPDATE_DELAY 977 //1sec at 16Mhz
#define UPDATE_DELAY 1221 //1sec at 20Mhz


#define T0PS	0x05	//timer0 prescale value 0x05=/1024
#define T1PS	0x02	//timer1 prescale value 0x02=/8
#define T2PS	0x04	//timer2 prescale value 0x04=/64

#define USDDR	DDRB	//ultrasonic sensor trigger port (for data direction setting)
#define USPORT	PORTB	//ultrasonic sensor trigger port
#define USTRIG	1		//ultrasonic sensor trigger pin

#ifndef __TYPES__
#include <avr/interrupt.h>

	#define __TYPES__
	typedef unsigned char  u08;
	typedef signed char  s08;
	typedef unsigned int  u16;
	typedef signed int  s16;
	typedef unsigned long  u32;
	typedef signed long  s32;

#endif
