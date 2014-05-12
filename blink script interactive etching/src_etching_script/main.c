 
#define F_CPU 8000000UL // Define software reference clock for delay duration

#include <stdlib.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/io.h>

 
 int raw;
short counter;

int rng(int, int);
unsigned short adc_read(unsigned char pin);
void adc_init(); 
void delay_ms(uint16_t ms) ;
void ledON(int ms, uint16_t mss);
 void pauseOFF(int ms, uint16_t mss);
void ledBlink(uint16_t ms);

int main(void)
{
	 
	adc_init();//initialize the LDR input


	DDRB = 0x00;
  	DDRB |= (1<<PB4);//set led OUTPUT
	//PB2 input LDR

ledBlink(5);

	// Retrieve, increment and store a randseed to eeprom - so the number sequence isn't the same each time!
	uint8_t randSeed = eeprom_read_byte ((uint8_t*)1);
	++randSeed;
	eeprom_write_byte ((uint8_t*)1, randSeed);
	srand (randSeed);

	while (1 == 1)
	{
			raw = adc_read(1);//reading change in light value
		    	delay_ms(40);

		if ( raw < 100){
			ledON( rng(2,3) , 100);
			pauseOFF(rng(3,10), 10);
			 
		}	
		counter++;
		if (counter > 200)
			{
				ledBlink(rng(1,5));
				counter=0;
			}	
	
   }

   return 1;
}

void ledBlink(uint16_t  ms)
{
 while (ms) {
PORTB |=(1<<PB4);
  delay_ms(100);
PORTB &=~(1<<PB4);
  delay_ms(100);
    ms--;
  }
}

void ledON(int ms, uint16_t mss)
{
PORTB |=(1<<PB4);
 while (ms) {
    delay_ms(mss);
    ms--;
  }
PORTB &=~(1<<PB4);

}

void pauseOFF(int ms, uint16_t mss)
{
 
 while (ms) {
    delay_ms(mss);
    ms--;
  }
 

}



 
 void delay_ms(uint16_t ms) {
  while (ms) {
    _delay_ms(1);
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
	PORTB &= ~_BV(PB2);
	//DDRB = 0x00;
	DDRB &= ~_BV(PB2);
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
