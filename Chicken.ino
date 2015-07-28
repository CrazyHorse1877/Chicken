#include <analogue_input_code.h>
#include <ChickenDoor.h>
#include <PingUltrasonic.h>
#include <USonic.h>
#include <ADC.h>
#include <serial.h>
#include <timing.h>
#include <typedefs.h>


/*      AVR Chicken Coop Door Controller MAIN       */
/*        PHIL SAMMONS                  */

#define OUTPUT_ON 1    //Digital Output
#define LED_ON 1    //LED Output
#define FLASHER_ON 1  //LED indicator of operation conneted to PD2
#define SERVO_ON 0    //SERVO Output
#define LDR_ON 1    //Light Dependant Resistor
#define SERIALIN_ON 0 //Output to Serial Port
#define SERIALOUT_ON 0  //Input from serial Port
#define SONIC_ON 0  //Get range from ultrasonic

#define PNTS 128    //power of 2. Points in the generated wave

#define ILIM 10000    //integral limit (< 32767)
//#define UPDATE_DELAY 488 //1sec at 8Mhz
#define UPDATE_DELAY 977 //1sec at 16Mhz
//#define UPDATE_DELAY 1221 //1sec at 20Mhz

typedef unsigned char  u08;
//typedef signed int s16;
//typedef signed int s08;
//typedef unsigned int  u16;

// Define the number of samples to keep track of.  The higher the number,
// the more the readings will be smoothed, but the slower the output will
// respond to the input.  Using a constant rather than a normal variable lets
// use this value to determine the size of the readings array.
const int numReadings = 50;

volatile int position=0, target=0;
volatile int velocity=0, vold=0;  
volatile int error=0, olderr=0, integral=0, derivative=0; //for PID
volatile int P=600, I=0, D=600;
volatile int vo=0;              //output
volatile long ltmp=0, ltmp2=0;        //for calculations
volatile int itmp=0, itmp2=0, divi=0;   //for calculations

//everything that changes in an ISR must be declared as volatile!
volatile u16 nextupdate=0;//, time_ms=0;    //use for general timing

volatile u08 gen_flag = 0;          //general purpose flags
volatile u08 servo_flag = 0;        //Servo flags
volatile u08 ldr_flag = 0;          //LDR flags
volatile u16 us_range = 0;          //range measured by ultrasonic sensor
volatile u16 ldr_value = 0;         //value measured by ldr sensor
volatile u08 servopos = 0;          //Servo Position
volatile u08 servoflg = 0;          //Servo Flag
volatile u08 door_timer = 0;        //Door time to open/close
volatile u08 door_open = 0x01;          //Door open sensor
volatile u08 door_closed = 0x00;        //Door closed sensor
volatile u08 daylight_flag = 0x01;        //Daylight detected?
volatile u08 motor_control_flag = 0;    //Motor control state

volatile s16 analogout = 0;
volatile s16 analogin = 0;
volatile u08 analogLow = 0;
volatile u08 analogHigh = 0;
volatile s16 ldr_input = 0;

volatile u32 wdtimer = 0;
volatile u32 wdcounter = 0;
volatile u32 timer1 = 0;
volatile u32 timer2 = 0;
volatile u08 timer2_flag = 0;       //Timer2 Flag
volatile u08 scale;
volatile char string [64];
s08 point[8];

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int inputPin = A0;

void setup()
{
  // initialize serial communication with computer:
  Serial.begin(9600);
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;

  if(FLASHER_ON){
    DDRD |= (1<<2);     // Flashing LED on pin PD2
  }

  if(SERVO_ON){
//    DDRD |= (1<<3);     //Set PORTD pin 3 to output for a servo (OC2B)
//    DDRD |= (1<<2);     //Set PORTD pin 2 to output for servoing motor
//    DDRD |= (1<<4);     //Set PORTD pin 4 to output for servoing motor
//    DDRD |= 0x60;     //Set PORTD pins 5 and 6 to outputs for PWM (OC0A and OC0B)
  }

    if(OUTPUT_ON){
//    DDRC |= (1<<0);     // Digital Output set on pin PC0
//    DDRC |= (1<<1);     // Digital Output set on pin PC1
    DDRC |= (1<<2);     // Digital Output set on pin PC2
    DDRC |= (1<<3);     // Digital Output set on pin PC3
    DDRC |= (1<<4);     // Digital Output set on pin PC4
    DDRC |= (1<<5);     // Digital Output set on pin PC5
  }

  if(LED_ON){
//    DDRC |= (1<<LED1);    // LED on pin PC0
//    DDRC |= (1<<LED2);    // LED on pin PC1
//    DDRC |= (1<<LED3);    // LED on pin PC2
//    DDRC |= (1<<LED4);    // LED on pin PC3
//    DDRC |= (1<<LED5);    // LED on pin PC4
//    DDRC |= (1<<LED6);    // LED on pin PC5
    DDRD |= (1<<4);     // LED on pin PD3
  }

  if(FLASHER_ON){
    DDRD |= (1<<2);     // Flashing LED on pin PD2
  }

  if(SERVO_ON){
//    DDRD |= (1<<3);     //Set PORTD pin 3 to output for a servo (OC2B)
//    DDRD |= (1<<2);     //Set PORTD pin 2 to output for servoing motor
//    DDRD |= (1<<4);     //Set PORTD pin 4 to output for servoing motor
//    DDRD |= 0x60;     //Set PORTD pins 5 and 6 to outputs for PWM (OC0A and OC0B)
  }
    
  if(SONIC_ON){
    USDDR |= (1<<USTRIG); //Set ultrasonic sensor trigger pin 1 PORT B as output
  }

//INTERRUPTS AND TIMER SETUP
//If you enable an interrupt but don't have an ISR for it, Bad Things(tm) will happen
//ISRs go down the bottom after the main() function


  gen_flag |= 0x00;

  sei();                  //Enable interrupts

  time_ms = 0;
  nextupdate = 1000;            //Start sending data after a delay
  
if(SERVO_ON){
  TCCR0A = 0xA3;              //fast PWM on OC0A/B
  TCCR0B = T0PS;              //set prescaler
  //TIMSK0 |= 0x01;           //interrupt on timer0 overflow
} 
  gen_flag |= 0x00;
  timing_init2(T2PS);           //timer2 prescale setup/interrupt enable

  if(SERIALOUT_ON || SERIALIN_ON)
  ser_init();               //Initialise Serial Stuff

  if(SERVO_ON || LDR_ON ) ADC_init(1);  //Initialise ADC Stuff

  gen_flag |= 0x01;           //  init 1 sec do something
  servo_flag |= 0x14;           //  init Servo Sampling
  ldr_flag |= 0x01;           //  init sampling

  sei();                  //Enable interrupts

  time_ms = 0;
  nextupdate = 1000;            //Start sending data after a delay
  
  if(SERVO_ON){
    OCR0A = 0x80; //PWM on OC0A at 50%
    OCR0B = 0x80; //PWM on OC0B at 50%
    OCR2B = 0x80; //centre the servo
  }

}


void loop() {
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(inputPin);
   Serial.print("sensor \t");
   Serial.print(readings[readIndex]);
   Serial.print("\tOut\t");
   
   // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings)
    // ...wrap around to the beginning:
    readIndex = 0;

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits
  //Serial.read();
  Serial.println(average);
  delay(100);        // delay in between reads for stability
}


ISR(SIG_OVERFLOW2) {  //1.024ms "tick". Runs each time timer2 overflows.
  time_ms++;  //increment the time
  
  if (time_ms == nextupdate) {        //if it is time to toggle the LED again
    nextupdate += UPDATE_DELAY;       //set when it will be time to do it again
    gen_flag |= 0x01;           //set the flag so it will be done in the main program
    ldr_flag |= 0x01;           //set the LDR flag so it will be sampled
  }
  if (time_ms%500 == 0) gen_flag |= 0x02;   //do something every 500 milliseconds (0.5Hz)
  if (time_ms%100 == 0) gen_flag |= 0x20;   // send serial information
  if (time_ms%30 == 0 && SERVO_ON) servo_flag |= 0x01;  // sample servo pot
  if (time_ms%25 == 0) {            //do something every 25 milliseconds (40Hz)
    //note: servo pulses are usually between 30-50Hz repetition rate
    //that's convenient.
    
    TCCR2A |= 0x30;   //set output OC2B high on compare match
    TCCR2B |= (1<<6); //force compare (set OC2B high)
    
    servoflg = 1; //servo pulse in progress
  }
  else if (servoflg) {
    TCCR2A &= ~(1<<4);    //set output OC2B low on compare match
    if (servopos == 0) TCCR2B |= (1<<6);  //force compare (set OC2B low immediately)  
    servoflg = 0; //servo pulse completed
  }

}

SIGNAL(SIG_ADC) { //not interruptable
    analogLow = ADCL;
    analogHigh = ADCH;

  if(SERVO_ON){
    if(servo_flag & 0x10){
      analogout = analogHigh <<8;
      analogout |= analogLow;
      servo_flag &= ~0x10;  //clear flag
      servo_flag |= 0x20;
      wdtimer = 0;
      }
    else if(servo_flag & 0x20){
      analogin  = analogHigh <<8;
      analogin |= analogLow;
      servo_flag &= ~0x20;  //clear flag
      servo_flag |= 0x10;
      wdtimer = 0;
      }
    servo_flag |= 0x04;
  }
  if(LDR_ON){
    ldr_input = analogHigh <<8;
    ldr_input |= analogLow;
    ldr_flag |= 0x02;     //data updated
    wdtimer = 0;
  }
}


