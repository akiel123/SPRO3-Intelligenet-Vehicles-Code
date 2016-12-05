/*
 * HardwareControl.c
 *
 * Created: 11/10/2016 15:33:09
 *  Author: manke
 */ 

#include HardwareControl.h

float speed = 0; // < 0: backwards, > 0: forwards, == 0: freedrive
int direction = 0; //1 = force towards right, 0 = do nothing, -1 = force towards left 
bool breaking = false; //Break if true

void SetCommand(float fspeed, int fdirection, int fbreaking){
	speed = fspeed;
	direction = fdirection;
	breaking = fbreaking;
}

void SetSpeed(){};
void SetTurnDirection(){};

float GetVelocity(){
	return 1;
}

int GetWheelPosition();
float GetDistance();
float GetBodyAngleDeg(); //Returns between 0 and 360
float GetBodyAngleRad(); //Returns between 0 and 2*PI

float GetSensorData(int id);

void SetRotationOfSensor(float angle);