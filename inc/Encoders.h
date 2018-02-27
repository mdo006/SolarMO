#ifndef Encoders_h
#define Encoders_h

#include <Arduino.h>

/*
 Header file for implementations of functions used by SolarMO's Encoders :)
*/

//encoder counters
extern volatile long rightTick;
extern volatile long leftTick;

volatile long rightTick = 0;
volatile long leftTick = 0;

//Right Encoder ISR
void rightEncoder() {
	rightTick++;
}

//Left Encoder ISR
void leftEncoder(){
	leftTick++;
}

#endif
