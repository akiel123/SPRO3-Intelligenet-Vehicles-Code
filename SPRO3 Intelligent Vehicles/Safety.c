/*
 * Safety.c
 *
 * Created: 21/11/2016 11:03:05 AM
 *  Author: Me
 */ 

#include "Safety.h"

void USSHandle(double distance){
	switch(ussCase){
		case findingSpot:
		break;
		case: finishingPark:
		if(distance < 2){
			reachedWallFlag = 1;
		}
		break;
		
	}
}

void yield(void); //do a manual interrupt

void ERROR(void);
