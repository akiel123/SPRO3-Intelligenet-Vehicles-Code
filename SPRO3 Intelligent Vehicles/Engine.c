/*
 * Engine.c
 *
 * Created: 7/11/2016 10:57:30 AM
 *  Author: Me
 */ 

#define F_CPU 16E6
#include <avr/io.h>
#include <stdio.h>
#include "usart.h" //Enable connection between PC and microcontroller
#include <avr/interrupt.h>
#include <util/delay.h>

volatile int turns = 0

void initialize(){
	DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
	PORTD &= ~(1 << PORTD2);    // make PD2 input
	
	EICRA &= ~(0 << ISC01 | 0 << ISC00) //Set int0 to trigger on rising edge (input 1 on ISC11 and ISC10 (reversing))
	EIMSK &= ~(0 << INT0) //Enable external interrupt1
	sei();
}

ISR (INT0_vect){  // external interrupt 0/int0
	turns++;
}

int getTurns(void){return turns;}
	
int setTurns(int new){turns = new;}