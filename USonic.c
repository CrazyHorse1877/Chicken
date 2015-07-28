#include <avr/io.h>
#include <stdio.h>
#include <USonic.h>
#include <PingUltrasonic.h>
#include <timing.h>


/*
Timer usage:
TIMER0:	Output compare 0A/0B for motor PWM
TIMER1:	Input capture 1A for ultrasonic measurement	
TIMER2:	1.024ms overflow for general timing
		Output compare 2B for servo pulse generation (if used)

Pin usage:
PORTB:
	5	SCK  (for programming)
	4	MISO (for programming)
	3	MOSI (for programming)
	2
	1   ultrasonic sensor (Trigger Pulse Input pin)
	0	ultrasonic sensor (Echo pulse output pin)
PORTC:
	6	RESET (also for programming)
	5	heartbeat LED
PORTD:
	6	PWM (OC0A)
	5	PWM (OC0B)
	4
	3	servo output (OC2B)
	2
	1
	0
*/

//everything that changes in an ISR must be declared as volatile!
volatile u16 nextupdate=0, time_ms=0;	//use for general timing
volatile u08 flg = 0;		//general purpose flags
volatile u16 us_range = 0;	//range measured by ultrasonic sensor
volatile u08 servopos = 0;	//
volatile u08 servoflg = 0;	//

//note: time_ms will overflow after ~65sec
//this is only a problem if you want to time something longer than this
int main (void) {

//PIN SETUP
//All pins are inputs by default. Set DDR to 1 for output pins
//Outputting to an input pin sets a pullup/pulldown resistor on that pin

	DDRC |= (1<<5);	//Set PORTC pin 5 to output for an LED
	DDRD |= (1<<3);	//Set PORTD pin 3 to output for a servo (OC2B)
	DDRD |= 0x60;		//Set PORTD pins 5 and 6 to outputs for PWM (OC0A and OC0B)
	USDDR |= (1<<USTRIG);	//Set ultrasonic sensor trigger pin as output
	
//INTERRUPTS AND TIMER SETUP
//If you enable an interrupt but don't have an ISR for it, Bad Things(tm) will happen
//ISRs go down the bottom after the main() function

	TCCR0A = 0xA3;		//fast PWM on OC0A/B
	TCCR0B = T0PS;		//set prescaler
	//TIMSK0 |= 0x01;		//interrupt on timer0 overflow
	timing_init2(T2PS);	//timer2 prescale setup/interrupt enable

sei();	//enable interrupts

time_ms = 0;		//your time starts...NOW!
nextupdate = 100;	//start doing something after a 100ms delay

OCR0A = 0x80;	//PWM on OC0A at 50%
OCR0B = 0x80;	//PWM on OC0B at 50%
OCR2B = 0x80;	//centre the servo

	while (1) {
		if (flg & 0x01) {	//do something regularly so you know it is running
			//toggling the top right hand pin (PC5) is convenient.
			//connect an LED to it or something
			
			PORTC ^= (1<<5);	//toggle PORTC pin 5 (^ means XOR, XOR with 1 toggles)
			ping();			

			flg &= ~0x01;	//clear the flag so it only does it once each time
		}

		if (flg & 0x02) {		//do something else


			flg &= ~0x02;	//clear flag
		}
	}
}

//INTERRUPT SERVICE ROUTINES
//Make them SHORT. If you need to do a calculation or anything that takes more
//than a couple of instructions, set a flag in the ISR and do the hard work in
//the main program.
//interrupts you might need:
//SIG_OVERFLOW0
//SIG_OVERFLOW2
//SIG_INPUT_CAPTURE1
//SIG_OUTPUT_COMPARE2B

ISR(SIG_INPUT_CAPTURE1) {	//input capture event

	if (TCCR1B & (1<<6)) {	//rising edge captured
		TCCR1B |= T1PS;			//start timer1
		TCCR1B &= ~(1<<6);		//capture on falling edge
		TIFR1 = (1<<5);		//clear interrupt by writing 1 (0 in other locations)
	}
	else {					//falling edge captured
		us_range = ICR1;		//store measured pulse width
		TIMSK1 &= ~(1<<5);		//disable interrupt on capture
		flg &= ~0x02;	//set new data flag
	}
}

ISR(SIG_OVERFLOW2) {	//1.024ms "tick". Runs each time timer2 overflows.
	time_ms++;	//increment the time
	
	if (time_ms == nextupdate) {	//if it is time to toggle the LED again
		nextupdate += UPDATE_DELAY;	//set when it will be time to do it again
		flg |= 0x01;	//set the flag so it will be done in the main program
	}
	if (time_ms%25 == 0) {	//do something every 25 milliseconds (40Hz)
		//note: servo pulses are usually between 30-50Hz repetition rate
		//that's convenient.
		
		TCCR2A |= 0x30;		//set output OC2B high on compare match
		TCCR2B |= (1<<6);	//force compare (set OC2B high)
		
		servoflg = 1;	//servo pulse in progress
	}
	else if (servoflg) {
		TCCR2A &= ~(1<<4);		//set output OC2B low on compare match
		if (servopos == 0) TCCR2B |= (1<<6);	//force compare (set OC2B low immediately)	
		servoflg = 0;	//servo pulse completed
	}

}
