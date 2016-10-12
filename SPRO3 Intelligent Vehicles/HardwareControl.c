/*
 * HardwareControl.c
 *
 * Created: 11/10/2016 15:33:09
 *  Author: manke
 */ 

#include HardwareControl.h
#include <stdbool.h>

float speed = 0; // < 0: backwards, > 0: forwards, == 0: freedrive
int direction = 0; //1 = right, 0 = straight, -1 left 
bool breaking = false; //Break if true

void SetCommandSet(float fspeed, int fdirection, bool fbreaking){
	speed = fspeed;
	direction = fdirection;
	breaking = fbreaking;
}

void SetSpeed(){};
void SetTurnDirection(){};

void ReturnSensorData();