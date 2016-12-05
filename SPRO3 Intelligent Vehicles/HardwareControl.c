/*
 * HardwareControl.c
 *
 * Created: 11/10/2016 15:33:09
 *  Author: manke
 */ 

#include "HardwareControl.h"

double speed = 0; // < 0: backwards, > 0: forwards, == 0: freedrive
int direction = 0; //1 = force towards right, 0 = do nothing, -1 = force towards left 
int breaking = 0; //Break if true


/*
void SetCommand(double fspeed, int fdirection, int fbreaking){
	speed = fspeed;
	direction = fdirection;
	breaking = fbreaking;
}

void SetSpeed(void){};
void SetTurnDirection(void){};

double GetVelocity(void){return 1;}

int GetWheelPosition(void){return 0;};
double GetDistance(void){return 0;};
double GetBodyAngleDeg(void){return 0;}; //Returns between 0 and 360
double GetBodyAngleRad(void){return 0;}; //Returns between 0 and 2*PI

double GetSensorData(int id){return 0;};

void SetRotationOfSensor(double angle){};
*/