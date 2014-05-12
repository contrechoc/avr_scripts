#define F_CPU 9600000UL // Define software reference clock for delay duration
#include <avr/io.h>
#include <util/delay.h>

#define ABSDIFF(a, b) ((a) < (b)? ((b) - (a)): ((a) - (b))) 

#define LED PB0
 

 volatile int i  ;
 
 

/*
PORTB |= (1 << PB0)  ;
for (i = 0 ; i < 100 ; i++ ) 
_delay_ms(30);
 PORTB  &=~(1<<PB0);  
 */
 

 int main (void)
 { 
 
//init PWM

 DDRB |= (1 << LED); // OC0A on PB0
 
 
 TCCR0A |= (1 << COM0A1)  // COM0A1 - COM0A0 (Set OC0A on Compare Match, clear OC0A at TOP)
 | (1 << WGM00) | (1 << WGM02); // WGM01 - WGM00 (set fast PWM)
 OCR0A = 100; // initialize Output Compare Register A to 0
 //TCCR0B |= (1 << CS01)  ; // Start timer at Fcpu / 256
 

//init ADC
 ADCSRA |= (1 << ADEN)| // Analog-Digital enable bit
 (1 << ADPS1)| // set prescaler to 8 (clock / 8)
 (1 << ADPS0); // set prescaler to 8 (clock / 8)
 
 ADMUX |= (1 << ADLAR)| // AD result store in (more significant bit in ADCH)
 (1 << MUX0); // Choose AD input AD2 (BP 4)

unsigned char oldVal = 0;
 

 for (;;)
 {
	//read add
	ADCSRA |= (1 << ADEN); // Analog-Digital enable bit
 	ADCSRA |= (1 << ADSC); // Discarte first conversion
 	while (ADCSRA & (1 << ADSC)); // wait until conversion is done
 	ADCSRA |= (1 << ADSC); // start single conversion
 	while (ADCSRA & (1 << ADSC)) // wait until conversion is done
 	ADCSRA &= ~(1<<ADEN); // shut down the ADC
	/*
	 if ( ABSDIFF(ADCH,oldVal) > 2 ){

loopServo();

//read add
	ADCSRA |= (1 << ADEN); // Analog-Digital enable bit
 	ADCSRA |= (1 << ADSC); // Discarte first conversion
 	while (ADCSRA & (1 << ADSC)); // wait until conversion is done
 	ADCSRA |= (1 << ADSC); // start single conversion
 	while (ADCSRA & (1 << ADSC)) // wait until conversion is done
 	ADCSRA &= ~(1<<ADEN); // shut down the ADC
oldVal = ADCH;

_delay_ms(230);
}
*/
		
for (i = 0 ; i < 100; i++ ) 
_delay_ms(30);


TCCR0B |= (1 << CS01)  ;
_delay_ms(50);
 	for (i = 253 ; i > 0 ; i-=4) // For loop (down counter 255 - 0 )
 	{
 	OCR0A = i; // Update Output Compare Register (PWM 0 - 255)
 	_delay_ms(100);
 	}
 
 
 
 
  TCCR0B  &=~(1<<CS01);  
 }
 }