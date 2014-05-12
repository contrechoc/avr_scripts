/*
attiny85

measuring analog

transferring value Serial
http://source.perl.free.fr/spip.php?article10






*/

/* ATTiny85 IO pins

             ___^___
           -|PB5 VCC|-
LED        -|PB3 PB2|-
serial out -|PB4 PB1|-
           -|GND PB0|-
             -------
*/

#define F_CPU 8000000UL // Define software reference clock for delay duration

#include <stdlib.h>
#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

 
int an3;
short counter;

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // clear bit
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))  // set bit


int rng(int, int);
unsigned short adc_read(unsigned char pin);
void adc_init(); 
void delay_ms(uint16_t ms) ;

 
// misc routines
void init_printf(void);
int serial_putc(char c, FILE *file);
void serial_write(uint8_t tx_byte);
uint64_t millis(void);

int readVcc();

/* some vars */
volatile uint64_t _millis    = 0;
volatile uint16_t _1000us    = 0;
uint64_t old_millis = 0;

// must be volatile (change and test in main and ISR)
volatile uint8_t tx_buzy = 0;
volatile uint8_t bit_index;
volatile uint8_t _tx_buffer; 

// compare match interrupt service for OCR0A
// call every 103us
ISR(TIM0_COMPA_vect) { 
  // software UART
  // send data
  if (tx_buzy) {
    if (bit_index == 0) {
      // start bit (= 0)
      cbi(PORTB, PB4);
    } else if (bit_index <=8) {
      // LSB to MSB
      if (_tx_buffer & 1) {
        sbi(PORTB, PB4);
      } else {
        cbi(PORTB, PB4);
      }
      _tx_buffer >>= 1;        
    } else if (bit_index >= 9) {
      // stop bit (= 1)
      sbi(PORTB, PB4);
      tx_buzy = 0;
    }
    bit_index++;
  }
  // millis update
  _1000us += 103;
  while (_1000us > 1000) {
    _millis++;
    _1000us -= 1000;
  }
}

// send serial data to software UART, block until UART buzy
void serial_write(uint8_t tx_byte) {
  while(tx_buzy);
  bit_index  = 0;
  _tx_buffer = tx_byte;
  tx_buzy = 1;
}

void serial_print(const char *str) {
  uint8_t i;
  for (i = 0; str[i] != 0; i++) {
    serial_write(str[i]);
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

/*** connect software UART to stdio.h ***/
/*
void init_printf() {
  fdevopen(&serial_putc, 0);
}

int serial_putc(char c, FILE *file) {
  serial_write(c);
  return c;
}
*/
/*** main routines ***/

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

 
int main(void)
{
	 
	adc_init();//initialize the LDR input

	DDRB = 0x00; 
 
	//PB2 input temp

  // LED IO
  sbi(DDRB,  PB1); // set LED pin as output
  //sbi(PORTB, PB1); // turn the LED on
  // Software UART IO
  sbi(DDRB,  PB4); // PB4 as output
  sbi(PORTB, PB4); // serial idle level is '1'
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
  // init stdout = serial
  //init_printf();


	while (1 == 1) 
	{

	int an4 = adc_read(3);//reading a sensitivity setting

	int voltageCheck = readVcc();
	if (voltageCheck > 320){

  		if ((millis() - old_millis) > 2000) {
  	 	// Toggle Port B pin 3 output state
   			PORTB ^= 1<<PB1;

   			old_millis = millis();

			an3 = adc_read(1);//reading temperature value

			an3 += an4-512;

			if ( an3 < 0 ) an3 = 10;
			if ( an3 > 1024 ) an3 = 1010;

  			int an3_0 = an3/1000;
  			int an3_1 = (an3 - an3_0*1000)/100;
  			int an3_2 = (an3 - an3_0*1000- an3_1*100 )/10;
  			int an3_3 = (an3 - an3_0*1000- an3_1*100- an3_2*10 );

  			char buf[] = {   "*      "  };

  			buf[1] = an3_0 + 48;
  			buf[2] = an3_1 + 48;
  			buf[3] = an3_2 + 48;
  			buf[4] = an3_3 + 48;

			buf[5] = '\r';
			buf[6] = '\n';

   			serial_print(buf);
 			 PORTB ^= 1<<PB1;
			}	
		}
	else{ //low voltage warning
		if ((millis() - old_millis) > 200) {
  	 	// Toggle Port B pin 3 output state
   			PORTB ^= 1<<PB1;
   			old_millis = millis();
 
			}
		}

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
