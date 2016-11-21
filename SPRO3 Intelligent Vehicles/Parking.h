/*
 * Parking.h
 *
 * Created: 21/11/2016 11:07:57 AM
 *  Author: Me
 */ 


#ifndef PARKING_H_
#define PARKING_H_

#define irChangeThreshhold 5

void doGeneralPark(void);
void doAlignedPark(int);
void doDefinedAndAlignedPark(double);
int alignWithObstacle(void);
double locatedParkingSpace(double);
int isOriented(double*);
int isWithounBounds(double);


#endif /* PARKING_H_ */