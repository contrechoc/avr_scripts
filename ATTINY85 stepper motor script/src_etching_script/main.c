#define F_CPU 8000000UL // Define software reference clock for delay duration

#include <stdlib.h>
#include <util/delay.h>
 
#include <avr/io.h>

#include <inttypes.h>
#include <avr/interrupt.h>

#define MIN(a,b)  ((a) < (b) ? (a) : (b)) 
#define MAX(a,b)  ((a) < (b) ? (b) : (a)) 
 
//#define pin0 	PB1
//#define pin1 	PB2
//#define pin2 	PB3
//#define pin3 	PB0

#define pin0 	PB0
#define pin1 	PB1
#define pin2 	PB2
#define pin3 	PB3

/*
  ATTiny85 IO pins

             ___^___
           -|PB5 VCC|-
motor	   -|PB3 PB2|- motor      
ADC	   -|PB4 PB1|- motor 
           -|GND PB0|- motor
             -------
 */

int raw;
int oldRaw;
short counter;

int rng(int, int);
int adc_read(unsigned char pin);
void adc_init(); 
void delay_ms(uint16_t ms) ;

int oldLDRVal = 1000;
uint64_t timer = 0;
uint64_t timer2 = 0;
unsigned char doMoveOnce = 0;
 

int readVcc();

    // speed setter method:
    void setSpeed(long whatSpeed);
    // mover method:
    void step(int number_of_steps);
 
    void stepMotor(int this_step);
    
    int direction;        // Direction of rotation
    int speed;          // Speed in RPMs
    unsigned long step_delay;    // delay between steps, in ms, based on speed
    int number_of_steps;      // total number of steps this motor can take
    int pin_count;        // whether you're driving the motor with 2 or 4 pins
    int step_number = 0;        // which step the motor is on

    long last_step_time;  

/* some vars */
uint64_t _millis = 0;
uint16_t _1000us = 0;

uint64_t old_millis = 0;
int readLDR = 0;

/* interrupts routines */
// timer overflow occur every 0.256 ms
ISR(TIM0_OVF_vect) {
  _1000us += 256;
  while (_1000us > 1000) {
    _millis++;
    _1000us -= 1000;
  }
}

// safe access to millis counter
uint64_t millis() {
  uint64_t m;
  cli();
  m = _millis;
  sei();
  return m;
}



void step(int steps_to_move){

 int steps_left = abs(steps_to_move);  // how many steps to take
 int stepsTotal = abs(steps_to_move);
  // determine direction based on whether steps_to_mode is + or -:
 if (steps_to_move > 0) {direction = 1;}
 if (steps_to_move < 0) {direction = 0;}

 int stepStart = 0;

 // decrement the number of steps, moving one step each time:
  while(steps_left > 0) {
  // move only if the appropriate delay has passed:

   step_delay = 50;

  if ( stepStart <= 4 ) step_delay = 60L * 1000L / 100 / 15;
	else if ( stepStart <= MIN( stepsTotal*.05, 5 ) ) step_delay = 60L * 1000L / 100 / 20;
	else if ( stepStart <= MIN (stepsTotal*.1 , 10) ) step_delay = 60L * 1000L / 100 / 30;
	else if ( stepStart <= MIN( stepsTotal*.15, 15 ) ) step_delay = 60L * 1000L / 100 / 50;
	else if ( stepStart <= MIN( stepsTotal*.2, 20) ) step_delay = 60L * 1000L / 100 / 55;
	else if ( stepStart <= MIN ( stepsTotal - 10, stepsTotal*.8)  ) step_delay = 60L * 1000L / 100 / 60;
	else if ( stepStart <= MIN( stepsTotal - 5, stepsTotal*.85) ) step_delay = 60L * 1000L / 100 / 50;
	else if ( stepStart <= MIN ( stepsTotal - 3, stepsTotal*.90) ) step_delay = 60L * 1000L / 100 / 30 ;
	else if ( stepStart > MIN( stepsTotal - 1, stepsTotal*.95) ) step_delay = 60L * 1000L / 100 / 15;

 
  if (millis() - last_step_time >= step_delay) {
	stepStart++;

      // get the timeStamp of when you stepped:
      last_step_time = millis();
      // increment or decrement the step number,

      // depending on direction:
      if (direction >= 1) {
        step_number++;
        if ( step_number ==  stepsTotal) {
           step_number = 0;
        }
      } 

       if (direction == 0) { 
       
        if ( step_number == 0) {
           step_number = stepsTotal;
        }
        step_number--;
        
      }

      // decrement the steps left:
      steps_left--;

      // step the motor to step number 0, 1, 2, or 3:
      stepMotor( step_number % 4);
    }

  }

}

void stepMotor(int thisStep){
cli();
 switch (thisStep) {
      case 0:    // 1010
     PORTB |=(1<<pin0);
     PORTB &=~(1<<pin1);
     PORTB |=(1<<pin2);
     PORTB &=~(1<<pin3);
      break;
      case 1:    // 0110
     PORTB &=~(1<<pin0);
     PORTB |=(1<<pin1);
     PORTB |=(1<<pin2);
     PORTB &=~(1<<pin3);
      break;
      case 2:    //0101
     PORTB &=~(1<<pin0);
     PORTB |=(1<<pin1);
     PORTB &=~(1<<pin2);
     PORTB |=(1<<pin3);
      break;
      case 3:    //1001
     PORTB |=(1<<pin0);
     PORTB &=~(1<<pin1);
     PORTB &=~(1<<pin2);
     PORTB |=(1<<pin3);    
  break;
    } 
sei();

}


 
int readVcc() {
  
  // Read 1.1V reference against AVcc
  ADMUX = _BV(MUX3) | _BV(MUX2);  

  _delay_ms(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  int result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts

}

void init(){

adc_init();//initialize the LDR input

	DDRB = 0x00; 
  	DDRB |= (1<<PB3);//set led OUTPUT
	DDRB |= (1<<PB2);//set led OUTPUT
	DDRB |= (1<<PB1);//set led OUTPUT
	DDRB |= (1<<PB0);//set led OUTPUT
 
	step_delay = 60L * 1000L / 100 / 25;//max 75

  cli();

  /* interrup setup */
  // prescale timer0 to 1/8th the clock rate
  // overflow timer0 every 0.256 ms
  TCCR0B |= (1<<CS01);
  // enable timer overflow interrupt
  TIMSK |= 1<<TOIE0;

  // Enable global interrupts
  sei();
 
timer = millis() + 3000;
timer2 = millis() + 5000;
readLDR = adc_read(2) ;
oldLDRVal = readLDR;


}

 
int main(void)
{
 
	init();

	while (1 == 1)
	{
 
	readLDR = adc_read(2);

	if ( (readLDR < 5 ) || (readLDR > 500 ) ) {
		readLDR = 100;
		oldLDRVal = 100;
	}
 
	if (  timer < millis() ){
	
		oldLDRVal = readLDR - 25;
		timer = millis() + 25000;
		doMoveOnce = 1;
	}
 
	if ( (abs( oldLDRVal - readLDR ) > 50) || ( doMoveOnce == 1) )
	{
 
 		step( MAX(100*abs( oldLDRVal - readLDR ), 500) + 100);
		step(-500);
	
		oldLDRVal = readLDR;
		timer = millis() + 25000;
		doMoveOnce = 0;
	}
 
/*
	if (  timer2 < millis() ) 
	{
		timer2 = millis() + 30000;

		unsigned char testStep = 215;
		while (testStep++ > 0)
		{
			stepMotor(testStep%4);
			delay_ms(1);
		}
	}
*/


   }

   return 1;
}
 

 void delay_ms(uint16_t ms) {
  while (ms) {
    _delay_ms(50);
    ms--;
  }
}

// Return a random number in the range min <= n <= max
int rng(int min, int max)
{
	return (min + (rand() / (RAND_MAX/(max+1))));	// not using _quite_ the most naive application of rand()
}

void adc_init()
{
	/* internal pull-ups interfere with the ADC. disable the
	 * pull-up on the pin if it's being used for ADC. either
	 * writing 0 to the port register or setting it to output
	 * should be enough to disable pull-ups. */
	PORTB &= ~_BV(PB4);
	//DDRB = 0x00;
	DDRB &= ~_BV(PB4);
	/* unless otherwise configured, arduinos use the internal Vcc
	 * reference. MUX 0x0f samples the ground (0.0V). */
	ADMUX = 0x0D;//B1101 = GND attiny45 page 139
	/*
	 * Enable the ADC system, use 128 as the clock divider on a 16MHz
	 * arduino (ADC needs a 50 - 200kHz clock) and start a sample. the
	 * AVR needs to do some set-up the first time the ADC is used; this
	 * first, discarded, sample primes the system for later use.
	 */
	ADCSRA |= _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADSC);
	/* wait for the ADC to return a sample */
	loop_until_bit_is_clear(ADCSRA, ADSC);
}

int adc_read(unsigned char pin)
{
	unsigned char l, h, r;
	ADMUX = 0x0D;
	r = (ADMUX & 0xf0) | (pin & 0x0f);
	ADMUX = r; /* select the input channel */
	ADCSRA |= _BV(ADSC);
	loop_until_bit_is_clear(ADCSRA, ADSC);

	/* must read the low ADC byte before the high ADC byte */
	l = ADCL;
	h = ADCH;

	return ((int)h << 8) | l;
}
