/* 

  ATTiny85 IO pins

             ___^___
           -|PB5 VCC|-
clock	   -|PB3 PB2|- latch       ----------------adc instead of reset
ADC	   -|PB4 PB1|- enable -- win a PIN like PB4
           -|GND PB0|- data
             -------
 
74HC595
             ___^___
           -|QB  VCC|-
reset	   -|QC  QA |-  
 	   -|QD  PB1|-  data
 	   -|QE  PB1|-  enable  - win a pin: make this GND permanent
 	   -|QF  PB1|-  latch
 	   -|QH  PB1|-  clock
 	   -|QI  PB1|-  reset - win a pin: make this V permanent
           -|GND PB0|-  
             -------


Author: Roznerd   http://roznerd.blogspot.com/
AVR:  ATtiny13   with internal clock set at 1MHz  and CLKDIV8 Fuse activated so delays are timely
Compiler: AVRStudio 4.17 build 666 + WinAVR GCC 20090313
Circuit: ATtiny13 controlling a 74HC595 hooked to 8 LED's with 470ohm resistors
Size:  200-300 bytes When Compiled with -Os optimization

Purpose:  

To demonstrate how a serial to parallel shift registers work.

----------------------------------------------------------------
---------------------------HEADER FILES-------------------------
----------------------------------------------------------------*/

#include<avr/io.h>
#include<util/delay.h>  // sets up the use of _delay_ms  and _delay_us

#include <inttypes.h>
#include <avr/interrupt.h>
#include <stdlib.h>

/*-------------------------------------------------------------------
-------------CONNECTION BETWEEN 74HC595 AND ATTINY13-----------------
---------------------------------------------------------------------*/
#define CONTROL_DDR	 	 DDRB       // this is where you will change the port if you are using a different AVR
#define CONTROL_PORT 	 PORTB
#define Data_Pin		 0
#define Enable_Pin		 1
#define Latch_Clk_Pin     2     
#define Shift_Clk_Pin     3
#define Reset_Pin    	  4

/*---------------------------------------------------------------------
-------------------CONTROL BITS OF SHIFT REGISTER ---------------------
This is basically just renaming everything to make it easy to work with
-----------------------------------------------------------------------*/

#define Clear_Enable  				CONTROL_PORT|=_BV(Enable_Pin)
#define Set_Enable	 			CONTROL_PORT&=~_BV(Enable_Pin)
#define Shift_Clk_H			   	CONTROL_PORT|=_BV(Shift_Clk_Pin)
#define Shift_Clk_L			    	CONTROL_PORT&=~_BV(Shift_Clk_Pin)
#define Latch_Clk_H			    	CONTROL_PORT|=_BV(Latch_Clk_Pin)
#define Latch_Clk_L				CONTROL_PORT&=~_BV(Latch_Clk_Pin)
#define Reset					CONTROL_PORT&=~_BV(Reset_Pin)
#define Reset_Clear			        CONTROL_PORT|=_BV(Reset_Pin)
#define delay(a)				_delay_ms(a)

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // clear bit
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))  // set bit


/*---------------------------------------------------------------------
-------------------MACRO TO MAKE SIMPLE BITWISE DEFINITIONS -----------
Creates new syntax for bitwise calls in registers. 
-----------------------------------------------------------------------*/

typedef struct
{
  unsigned int bit0:1;
  unsigned int bit1:1;
  unsigned int bit2:1;
  unsigned int bit3:1;
  unsigned int bit4:1;
  unsigned int bit5:1;
  unsigned int bit6:1;
  unsigned int bit7:1;
} _io_reg;

#define REGISTER_BIT(rg,bt) ((volatile _io_reg*)&rg)->bit##bt


/*----------------------------------------------------------------
----------------------------FUNCTIONS-----------------------------
Initialize functions that will be used later on
-----------------------------------------------------------------*/
void shift_in(void);  // function toggles the Shift Clock
void latch_in(void);  // function toggles the Latch Clock
void reset_SR(void);  // function toggles the Reset Pin

void doLoopThing(int j);
unsigned short adc_read(unsigned char pin);
void adc_init(); 
uint64_t millis(void);

/* some vars */
volatile uint64_t _millis    = 0;
volatile uint16_t _1000us    = 0;
uint64_t old_millis = 0;

/*----------------------------------------------------------------
---------------------------MAIN FUNCTION--------------------------
------------------------------------------------------------------*/

int output[8] = {1,1,0,0,0,0,0,0};   // set up array of integer data to create pattern to show on the LEDs hooked to the 595 outputs
int oldVal = 0;
int dir = 0;

void main(void)
{

adc_init();//initialize the LDR input

DDRB |= _BV(PB0);
DDRB |= _BV(PB1);
DDRB |= _BV(PB2);
//DDRB |= _BV(PB4);
DDRB |= _BV(PB3);

//CONTROL_DDR = 0xFF;  // Set the Control DDR (i.e. DDRB) to be all outputs

int i;  // initialize for loop variables

//int j;  // Loop variable for OPTION 2 - If you use OPTION 1 you should comment this out or you will get a warning upon compiling

//reset_SR();  // Toggle the Reset Pin on the 595 to clear out SR

Set_Enable;  // Set the Output Enable Pin on the 595 (PB1 in this case) LOW to allow data to show on the outputs upon being latched

 /* interrup setup */
  // call ISR(TIM0_COMPA_vect) every 103us (for 9600 bauds)
  // set CTC mode : clear timer on comapre match
  // -> reset TCNTO (timer 0) when TCNTO == OCR0A
  sbi(TCCR0A, WGM01);
  // prescaler : clk/8 (1 tic = 1us for 8MHz clock)
  sbi(TCCR0B, CS01);
  // compare register A at 103 us
  OCR0A = 103;
  // interrupt on compare match OCROA == TCNT0
  sbi(TIMSK, OCIE0A);
  // Enable global interrupts
  sei();


old_millis = millis() + 500;

while(1)  // infinite loop
{
// --- You have 2 options in this code by commenting in or out these sections.
// The options vary how the output array is read and how the display is latched

// ------------------------- OPTION 1 ------------------------------------------
// Read the output array from position 0 to 7 (i=0-7) and latch the display
// as each bit is shifted in so the display shows the "scrolling effect" and utilizes 
// the fact that everything in the shift register "shifts" over when a new bit is latched.
// Option 1 is simple. Comment this code out if you plan to use Option 2.

/*
		for (i=0; i<8; i++)  
		{
		REGISTER_BIT(PORTB,0) = output[i];   // Send data from the "output" array out to the Serial Input Pin on the 595 via PB0
		// Whatever state (hi or low) the Data Pin (PB0) is in when the shift clock is hit, is the state that is stored in the Shift Register
		shift_in();  // toggle Shift Clock Pin on 595 to shift current bit into SR
		
		latch_in();  // toggle the Latch Clock Pin on 595 to update the display after each new bit is shifted in

		}  // end for
*/

//--------------------------- OPTION 2 ----------------------------------------- 
// Read 8 bits from the output array iterating through the starting positions (i=0-7)
// and "wrapping" around the array to include all 8 bits in the output array. Once there are 8 new bits
// shifted into the shift register the Latch Pin is toggled to update the display. This option will
// yield the same results on the display, but the for loop is effectively doing the scrolling 
// (not the shifting of the register like in option 1).
// This type of setup is more useful to adapt for a situation where you want to send a byte out the 595 for
// a LCD or 7 seg display. This code doesnt exactly satisfy that application because of the iterating
// starting location and the fact that the array is small. Ideally for a 8-bit data application
// you would have an array of Char's (bytes) and you would clock them into the 595 bitwise. I will
// cover this in another Instructable.

int an4 = adc_read(2);

if (   old_millis < millis())
 {

	if ( abs(an4-oldVal) > 50 ){

		if ( dir == 0 ) dir = 1; else dir = 0;
		old_millis = millis() + 500;

	}
}


if ( dir == 0 ){

for (int j=7; j>=0; j--)  // this for loop allows the address of the data in the "output" array to advance after each read.
						 // Without this loop the leds wouldn't scroll, the pattern would just display statically
	{
	doLoopThing(j);
	}  // end for

}else{
for (int j=0; j<8; j++)  // this for loop allows the address of the data in the "output" array to advance after each read.
						 // Without this loop the leds wouldn't scroll, the pattern would just display statically
	{
	doLoopThing(j);
	}  // end for
}

oldVal = an4;


}  // end while

}	// end main

void doLoopThing(int j){

		// This FOR Loop picks the starting point for looking up the data in the "output" array
		// It starts at i=j and goes to i=8. This excludes any values for i that are less than j.
		int i;

		for (i=j; i<8; i++)  
		{
		REGISTER_BIT(PORTB,0) = output[i];   // Send data from the "output" array out to the Serial Input Pin on the 595 via PB0
		// Whatever state (hi or low) the Data Pin (PB0) is in when the shift clock is hit, is the state that is stored in the Shift Register
		shift_in();  // toggle Shift Clock Pin on 595 to shift current bit into SR
		
		}  // end for

		// This FOR Loops picks up where the other FOR loop left off, but addressing the data that was less than j.
		// This loop effectively "wraps" around the array, so when i=8 and there is still room in the SR, the Loop fills in
		// the remaining spots starting back at i=0. This is useful for making a "ring counter" to 
		// "scan" the columns of a Scrolling Dot Matrix LED display.

		for (i=0; i<j; i++)  // this for loop actually picks the addresses  in the "output" array to put out.
		{
		REGISTER_BIT(PORTB,0) = output[i];   // Send data from the "output" array out to the Serial Input Pin on the 595 via PB0
		// Whatever state (hi or low) the Data Pin (PB0) is in when the shift clock is hit, is the state that is stored in the Shift Register
		shift_in();  // toggle Shift Clock Pin on 595 to shift current bit into SR

		}  // end for


	latch_in(); // After 8 new bits have been clocked into the SR, Toggle the Latch Clock Pin on the 595 to present the 8 bits in the SR to th 595 Ouput Pins



}
	


/*---------------------------------------------------------------------------
----------------FUNCTION TO SHIFT DATA INTO 74HC595 -------------------------
Toggles the Shift Clock
-----------------------------------------------------------------------------*/
void shift_in(void)
{
Shift_Clk_H;
delay(20);   // these delays are arbitrary - they are here to slow things down so you can see the pattern
// for Option 1 - the delay should be around 20 for Option 2 - the delay should be about 3.
Shift_Clk_L;
delay(20);
}

 	
/*---------------------------------------------------------------------------
-----------------FUNCTIONS TO LATCH DATA TO OUTPUTS IN 74HC595 --------------
Toggles the Latch Clock
-----------------------------------------------------------------------------*/
void latch_in(void)
{
Latch_Clk_H;
delay(3); // these delays are arbitrary - they are here to slow things down so you can see the pattern
Latch_Clk_L;
delay(3);
}

/*---------------------------------------------------------------------------
-----------------FUNCTIONS TO RESET SHIFT REGISTER in 74HC595----------------
Toggles Reset Pin to Clear Shift Register
-----------------------------------------------------------------------------*/
void reset_SR(void)
{
Reset;
delay(10);
Reset_Clear;
delay(10);
}

void adc_init()
{
	/* internal pull-ups interfere with the ADC. disable the
	 * pull-up on the pin if it's being used for ADC. either
	 * writing 0 to the port register or setting it to output
	 * should be enough to disable pull-ups. */
	PORTB &= ~_BV(PB4);
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

unsigned short adc_read(unsigned char pin)
{
	unsigned char l, h, r;

	r = (ADMUX & 0xf0) | (pin & 0x0f);
	ADMUX = r; /* select the input channel */
	ADCSRA |= _BV(ADSC);
	loop_until_bit_is_clear(ADCSRA, ADSC);

	/* must read the low ADC byte before the high ADC byte */
	l = ADCL;
	h = ADCH;

	return ((unsigned short)h << 8) | l;
}

// safe access to millis counter
uint64_t millis() {
  uint64_t m;
  cli();
  m = _millis;
  sei();
  return m;
}

ISR(TIM0_COMPA_vect) { 
  // software UART
  // send data
 
  // millis update
  _1000us += 103;
  while (_1000us > 1000) {
    _millis++;
    _1000us -= 1000;
  }
}








