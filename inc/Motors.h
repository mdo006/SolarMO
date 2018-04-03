#ifndef Motors_h
#define Motors_h

#define RFULL_SPEED 75 //75
#define LFULL_SPEED 80 //82

#include <Arduino.h>
#include <Encoders.h>
#include <Adafruit_MotorShield.h>



/*
 Header file for implementations of functions used by SolarMO's Motors :)
*/

// Create the motor shield object with the default I2C address
extern Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//Adafruit_MotorShield AFMS = Adafruit_MotorShield;

// Select which 'port' M1, M2, M3 or M4. In this case, M1
extern Adafruit_DCMotor *motor1 = AFMS.getMotor(1); //M1 (Left Motor)
extern Adafruit_DCMotor *motor2 = AFMS.getMotor(3); //M3 (Right Motor)
extern Adafruit_DCMotor *motor3 = AFMS.getMotor(4);

//Functions for movement(without encoders)
void forward() {
    motor1->run(FORWARD);
    motor2->run(FORWARD);
}

void reverse() {
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
}

void halt() {
    motor1->run(RELEASE);
    motor2->run(RELEASE);
}

void leftTurn() {
    motor1->run(BACKWARD);
    motor2->run(FORWARD);
}

void rightTurn() {
    motor1->run(FORWARD);
    motor2->run(BACKWARD);
}

void runBrush() {
    motor3->run(FORWARD);
}

//Functions for movement(with encoders)
void turnRightAmt(int tickAmt) {
    long prevTickR = rightTick;
    long prevTickL = leftTick;
    
    while(((rightTick - prevTickR) + (leftTick - prevTickL)) / 2 <= tickAmt) {
        rightTurn();
    }
    halt();
}

void turnLeftAmt(int tickAmt) {
    long prevTickR = rightTick;
    long prevTickL = leftTick;
    
    while(((rightTick - prevTickR) + (leftTick - prevTickL)) / 2 <= tickAmt) {
        leftTurn();
    }
    halt();
}

void reverseAmt(int tickAmt) {
    long prevTickR = rightTick;
    long prevTickL = leftTick;
    
    while(((rightTick - prevTickR) + (leftTick - prevTickL)) / 2 <= tickAmt) {
        reverse();
    }
    halt();
    
}

void fwdAmt(int tickAmt) {
    long prevTickR = rightTick;
    long prevTickL = leftTick;
    
    while(((rightTick - prevTickR) + (leftTick - prevTickL)) / 2 <= tickAmt) {
        forward();
    }
    halt();
    
}


#endif:
