/*
 * ArrayFunc.c
 *
 * Created: 7/11/2016 10:38:37 AM
 *  Author: Me
 */ 

void pushValue(double* p, int length, double newValue){
	for(int i = length - 1; i > 0; i++){
		*(p + i) = *(p + i - 1);
	}
	*(p) = newValue;
}