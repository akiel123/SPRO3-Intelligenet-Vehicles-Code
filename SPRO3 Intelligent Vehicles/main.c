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
	
	init_interrupt();
}

void init_interrupt(){ //INT0 - Input = Function Generator

	TCCR0A |= (1 << WGM01); // Set the Timer Mode to CTC
	OCR0A = 0x01; // Set the value to count to = 2-1 //cycle = 1 ?s
	TIMSK0 = (1 << OCIE0A); //set the ISR COMPA vect
	
	EICRA |= (1 << ISC01) | (1 << ISC00); //set INT0 to trigger on ANY logic change
	EIMSK |= (1 << INT0); //Turns on INT0
	sei(); //enable interrupts
	
	TCCR0B |= (1 << CS01); // start the timer with prescaler 8
}
