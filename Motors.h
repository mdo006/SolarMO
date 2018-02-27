#ifndef Motors_h
#define Motors_h

#include <Arduino.h>
#include <Encoders.h>
#include <Adafruit_MotorShield.h>

/*
 Header file for implementations of functions used by SolarMO's Motors :)
*/

// Create the motor shield object with the default I2C address
extern Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
extern Adafruit_DCMotor *motor1 = AFMS.getMotor(1); //M1 (DC Motor 1)
extern Adafruit_DCMotor *motor2 = AFMS.getMotor(3); //M3 (DC Motor 2)

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
    motor1->run(FORWARD);
    motor2->run(BACKWARD);
}

void rightTurn() {
    motor1->run(BACKWARD);
    motor2->run(FORWARD);
}

//Functions for movement(with encoders)
void turnRightAmt(int tickAmt) {
    long prevTickR = rightTick;
    long prevTickL = leftTick;
    
    while(rightTick - prevTickR <= tickAmt || leftTick - prevTickL <= tickAmt) {
        rightTurn();
    }
    halt();
}

void turnLeftAmt(int tickAmt) {
    long prevTickR = rightTick;
    long prevTickL = leftTick;
    
    while(rightTick - prevTickR <= tickAmt || leftTick - prevTickL <= tickAmt) {
        leftTurn();
    }
    halt();
}

#endif:
