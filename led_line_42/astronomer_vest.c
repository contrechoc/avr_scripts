#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU  8000000L 

#include <util/delay.h>

#define TIMER1_PRESCALE_64  2
 

#define HIGH 0
#define LOW 1
 
unsigned char col = 0;
unsigned char counter = 0;
unsigned char counterPrescaler = 0;
unsigned char counterP = 2;

unsigned char bitCounter = 0;
unsigned char dir = 1;

unsigned char  rows[6] = { 5, 3, 4, 2, 1, 0 };
unsigned char  cols[7] = { 12, 11, 10, 9, 8, 7, 6  };

unsigned char leds[7] ;

void delay_ms( uint16_t milliseconds);
void clearLeds() ;
void setPattern();
void digitalWrite(unsigned char pinNum, unsigned char highLow) ;
void InitUART (unsigned char baudrate);
unsigned char ReceiveByte (void);
void TransmitByte (unsigned char data);
void fancy();


void digitalWrite(unsigned char pinNum, unsigned char highLow) {
	if ( highLow == 0 )
	{
		 switch ( pinNum ) {

			case 0: PORTD &= ~_BV(PD2); break;
			case 1: PORTD &= ~_BV(PD3); break;
			case 2: PORTD &= ~_BV(PD4); break;
			case 3: PORTD &= ~_BV(PD5); break;
			case 4: PORTD &= ~_BV(PD6); break;
			case 5: PORTB &= ~_BV(PB0); break;

			case 6: PORTB &= ~_BV(PB1); break;
			case 7: PORTB &= ~_BV(PB2); break;
			case 8: PORTB &= ~_BV(PB3); break;
			case 9: PORTB &= ~_BV(PB4); break;
			case 10: PORTB &= ~_BV(PB5); break;
			case 11: PORTB &= ~_BV(PB6); break;
			case 12: PORTB &= ~_BV(PB7); break;
 
		 }
	 }
	 else
	 {
		 switch ( pinNum ) {
		 case 0: PORTD |= (1<<PD2); break;
		 case 1: PORTD |= (1<<PD3); break;
		 case 2: PORTD |= (1<<PD4); break;
			case 3: PORTD |= (1<<PD5); break;
			case 4: PORTD |= (1<<PD6); break;
			case 5: PORTB |= (1<<PB0); break;

			case 6: PORTB |= (1<<PB1); break;
			case 7: PORTB |= (1<<PB2); break;
			case 8: PORTB |= (1<<PB3); break;
			case 9: PORTB |= (1<<PB4); break;
			case 10: PORTB |= (1<<PB5); break;
			case 11: PORTB |= (1<<PB6); break;
			case 12: PORTB |= (1<<PB7); break;

		 }
	 }
}

void setPattern() {
 
	//
 
 //	for (unsigned char row = 0; row < 2; row++) {
 counter+= dir ;
 unsigned char c1 = counter+1;

 // if ( (counter%2 == 1) )
	 {
		leds[0] = 0xff  - ( 1 << (counter%6))- ( 1 << (c1%6)) ; 
		leds[1] = 0xff  - ( 1 << (counter%6))- ( 1 << (c1%6)) ;  
		leds[2] = 0xff  - ( 1 << (counter%6))- ( 1 << (c1%6)) ;  
		leds[3] = 0xff  - ( 1 << (counter%6))- ( 1 << (c1%6)) ;  
		leds[4] = 0xff  - ( 1 << (counter%6))- ( 1 << (c1%6)) ;  
		leds[5] = 0xff  - ( 1 << (counter%6))- ( 1 << (c1%6)) ; 
		leds[6] = 0xff  - ( 1 << (counter%6))- ( 1 << (c1%6)) ; 
	}
 
 if ( counter >  25 ) dir = -1;
 if ( counter < 1 ) dir = 1;
 
}



void clearLeds() {
  for (unsigned char i = 0; i < 5; i++)
      leds[i] = 0x00;
}


void delay_ms( uint16_t milliseconds)
{
   for( ; milliseconds > 0; milliseconds--)
   {
      _delay_ms( 1);
   }
}

int main (void)
{

	DDRB = 0xFF;			// Set output.
	DDRD = 0xFF;

	PORTD = 0x00;
	PORTB = 0x00;

	clearLeds();

  TCCR0B = (1 << WGM02) | TIMER1_PRESCALE_64; //WGNM12 bit3 of tccr1b timer control register
  OCR0A = (uint16_t)200;//output compare register comined with WGM02 set
  TIMSK |= 1 << OCIE0A;   // Output Compare Interrupt Enable (timer 1, OCR1A)
  
  	sei();
  	InitUART (51);  //for communicating with arduino at 16000 attiny2313 on 8000

	//fancy();

	 while (1)
    {

	 	 //	unsigned char inChar = 1; //ReceiveByte ();
      	//counterPrescaler++;
		delay_ms(150);

		 setPattern() ;

 
	   if ( counterPrescaler%50 == 0)
 		 if ( 1 == 0 )
	 	{
			cli();
			TCCR0B = (1 << WGM02) | counterP; //WGNM12 bit3 of tccr1b timer control register
			OCR0A = (uint16_t)200;//output compare register comined with WGM02 set
			TIMSK |= 1 << OCIE0A;   // Output Compare Interrupt Enable (timer 1, OCR1A)
			counterP++;
			delay_ms(200);
  			if ( counterP > 4 ) counterP = 2;	
			sei();
	 	}
	 
 
	}
}

void fancy(){
	for ( unsigned char c = 0; c<25; c++){
			setPattern();
			counterPrescaler++;
			delay_ms(50);
			}
}

// Interrupt routine
SIGNAL( TIMER0_COMPA_vect   )  {

		// extendedDigitalWrite( cols[col], HIGH );  // Turn whole previous column off
	digitalWrite( cols[col], LOW );
  	col++;
  	if (col == 6)
    col = 0;

  	for (unsigned char row = 0; row < 7; row++) {

	    if (  (leds[col] & (1 << row )) == HIGH )
	     	digitalWrite( rows[row], LOW);  // Turn on this led
	    else
	     	digitalWrite( rows[row], HIGH); // Turn off this led
	  	}

	 // extendedDigitalWrite(cols[col], LOW); // Turn whole column on at once (for equal lighting times)
	digitalWrite( cols[col],  HIGH);
}


/* Initialize UART */
void InitUART (unsigned char baudrate)
{
  /* Set the baud rate */
  UBRRL = baudrate;

  /* Enable UART receiver and transmitter */
  UCSRB = (1 << RXEN) | (1 << TXEN);

  /* set to 8 data bits, 1 stop bit */
  UCSRC = (1 << UCSZ1) | (1 << UCSZ0);

}

/* Read and write functions */
unsigned char ReceiveByte (void)
{

  /* Wait for incomming data */
  while (!(UCSRA & (1 << RXC))   );

  /* Return the data */

  return UDR;

}

void TransmitByte (unsigned char data)
{
  /* Wait for empty transmit buffer */
  while (!(UCSRA & (1 << UDRE)));
//	blinkEm(100,0);
  /* Start transmittion */
  UDR = data;
}


