/*
 * HardwareControl.h
 *
 * Created: 11/10/2016 15:32:55
 *  Author: manke
 */ 


#ifndef HARDWARECONTROL_H_
#define HARDWARECONTROL_H_

#define sidBackPointingRight 123 //find better names?
#define sidBackMiddlePointingBack1 123
#define sidBackMiddlePointingBack2 123
#define sidFrontRightAngular 123

#define ibr 1
#define obr 1
#define ifr 1
#define ofr 1
#define wSideSens1 10 //Distance from front of car, to middle of sideSens1
#define wSideSens2 10 //Distance from front of car, to middle of sideSens2
#define wSideSens3 10 //Distance from back of car, to middle of sideSens3
#define wSideSens13 10 //Distance between sideSens1 and sideSens2
#define lowerLimitSens1 15
#define lowerLimitSens2 15
#define lowerLimitSens3 15
#define upperLimitSens1 70
#define upperLimitSens2 70
#define upperLimitSens3 70
#define PerfectAlignmentTolerance 0.1
#define carLength 30
#define carWidth 20
#define parkingMargin 5

//Sensor port values
#define USSPort PORTD
#define USS1 (1 << 6)
#define IR1 (1 << 0)
#define IR2 (1 << 1)
#define IR3 (1 << 2)
#define IR4 (1 << 3)
#define intervalLength 1 //Interval between each reed switch update on the wheel

void SetCommand(double, int, int);
double GetVelocity(void);
int GetWheelPosition(void);
double GetDistance(void);
void SetRotationOfSensor(double);
double GetBodyAngleDeg(void);
double GetBodyAngleRad(void);
double GetObstacleOrientationD(void);
double GetObstacleOrientationR(void);
double GetUSSData(int);
double GetIRData(int id);
void resetDistance();
double getDistance();
void resetBodyAngle(){

#endif /* HARDWARECONTROL_H_ */