/*
 * HardwareControl.c
 *
 * Created: 11/10/2016 15:33:09
 *  Author: manke
 */ 

#define F_CPU 16E6
#include <avr/io.h>
#include <stdio.h>
#include "usart.h" //Enable connection between PC and microcontroller
#include <avr/interrupt.h>
#include <util/delay.h>
#include "MathExtra.h"

#include "HardwareControl.h"

double speed = 0; // < 0: backwards, > 0: forwards, == 0: freedrive
int direction = 0; //1 = force towards right, 0 = do nothing, -1 = force towards left 
int breaking = 0; //Break if true

volatile double distanceDriven = 0;
volatile double angle = 0;
volatile int j = 0, i = 0;

void SetCommand(double fspeed, int fdirection, int fbreaking){
	speed = fspeed;
	direction = fdirection;
	breaking = fbreaking;
}
void resetDistance(){
	distanceDriven = 0;
}
double getDistance(){
	return distanceDriven;
}
void resetAngle(){
	angle = 0;
}
double GetBodyAngleDeg(void){ //Returns between 0 and 360
	return angle;
} 
double GetBodyAngleRad(void){ //Returns between 0 and 2*PI
	return angle;
}


double GetVelocity(void){return 1;}

double GetUSSData(int id){
	PORTD |= 0x40; //turn on PD6 (Trig Pin) for 10 ?
	_delay_us(10);
	PORTD &= ~0x40; //turn off PD6 (Trig Pin)
	i = 0; //reset timer
	int meassurementTime = 2;
	_delay_ms(meassurementTime);
	int res = (int)i;
	if(res < 150) return 0;
	else if(res > 6000) return -1;	
}
double GetIRData(int id){
	switch(id){
		case IR1:
		break;
		case IR2;
		break;
		case IR3;
		break;
		default
		return -2;
	}
}
double * GetObstacleOrientationR(){
	static double result[6];
	double sens1 = GetIRData(IR1);
	double sens2 = GetIRData(IR2);
	double sens3 = GetIRData(IR3);
	double dsens1 = sens2 - sens1;
	double dsens2 = sens3 - sens2;
	double dsens3 = sens3 - sens1;
	
	
	double angle1 = acos(wSideSens1/dsens1);
	double angle2 = acos(wSideSens2/dsens2);
	double angle3 = acos(wSideSens3/dsens1);
	
	if(dsens1 < 0) angle1 = 180 * Deg2Rad - angle1;
	if(dsens2 < 0) angle3 = 180 * Deg2Rad - angle2;
	if(dsens3 < 0) angle2 = 180 * Deg2Rad - angle3;
	
	result[0] = dsens1;
	result[1] = dsens2;
	result[2] = dsens3;
	result[3] = angle1;
	result[4] = angle2;
	result[5] = angle3;
	
	return result;
}
double GetObstacleOrientationD(){
	double rad = GetObstacleOrientationR();
	return rad * Rad2Deg;
}

ISR (INT0_vect){
	j = ~j; //switch On/Off counter
}

ISR (TIMER0_COMPA_vect){
	if (j == 1)
	i++; //counts in ?s
}