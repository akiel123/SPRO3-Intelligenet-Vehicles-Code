/*
 * SPRO3 Intelligent Vehicles.c
 *
 * Created: 14/09/2016 11:12:31 AM
 * Author : Me
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>


int main(void)
{
    /* Replace with your application code */
    while (1) 
    {
		//TEst
    }
}

void init(){
	DDRD = 0xFF;// configuring the PORTD to be all outputs
	PORTC = 0XFF; //configuring The portC to be inputs with pull down enabled
	TCCR0A |= (1 << WGM01); //Configuration for interrupt every 100us
	OCR0A = 199;
	TIMSK0 |= (1 << CE0A);
	TCCR0B |= (1 << CS01);
	PORTD |= (1 << PORTD6);
	sei();//enabling interrupts
	
	EICRA |= (1 << ISC00);    // set INT0 to trigger on ANY logic change 
	EIMSK |= (1 << INT0);     // Turns on INT0 
	sei();                    // turn on interrupts 
}
