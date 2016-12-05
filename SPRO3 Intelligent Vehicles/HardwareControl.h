/*
 * HardwareControl.h
 *
 * Created: 11/10/2016 15:32:55
 *  Author: manke
 */ 


#ifndef HARDWARECONTROL_H_
#define HARDWARECONTROL_H_

#define sidBackPointingRight = 123; //find better names?
#define sidBackMiddlePointingBack1 = 123;
#define sidBackMiddlePointingBack2 = 123;
#define sidFrontRightAngular = 123;

#define ibr 1
#define obr 1
#define ifr 1
#define ofr 1


void SetCommand(float, int, int)
float GetSensorData(int id);
float GetVelocity();
int GetWheelPosition();
float GetDistance();
void SetRotationOfSensor(float angle);
float GetBodyAngleDeg();
float GetBodyAngleRad();


#endif /* HARDWARECONTROL_H_ */