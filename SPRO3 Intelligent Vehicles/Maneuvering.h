/*
 * IncFile1.h
 *
 * Created: 11/10/2016 15:27:56
 *  Author: manke
 */ 


#ifndef MANEUVERING_H_
#define MANEUVERING_H_

#define rrad 6.28318530718
#define breakSpeed 1
#define actionTurnLeft 1
#define actionTurnRight 2
#define actionDriveStraight 4

void shiftDistanceBack1(double);
void shiftDistanceBack2(double, double);
void shiftDistanceBackOffset(double, double);
void shiftDistanceFront1(double);
void shiftDistanceFront2(double, double);
void shiftDistanceBackFront(double);
void shiftDistanceFrontBack(double);
void shifDistanceLimitedSpace(double, double);
void breaK(void);
void turnLeft(void);
void turnMiddle(void);
void turnRight(void);
void backUpDistance(double);
void driveDistance(double);

#endif /* INCFILE1_H_ */