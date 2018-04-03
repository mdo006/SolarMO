#ifndef PIDTask_h
#define PIDTask_h

#include <Arduino.h>

int rSpeed = 0;
int lSpeed = 0;
int speedAdjustR = 0;
int speedAdjustL = 0;
int Kp = 10;

enum SpeedState{INIT_3, SAMPLE} Speed_State;
enum PIDState { INIT_4, PCONT } PID_State;
enum SETState { INIT_5, SET} SET_State;


void getSpeed_Init() {
    Speed_State = INIT_3;
}

void getSpeed_Tick() {
    static long rightVal = 0;
    static long leftVal = 0;
    //Actions
    switch(Speed_State) {
        case INIT_3:
            break;
        case SAMPLE:
            rSpeed = rightTick - rightVal;
            lSpeed = leftTick - leftVal;
            rightVal = rightTick;
            leftVal = leftTick;
            break;
        default:
            break;
    }
    
    //Transitions
    switch(Speed_State) {
        case INIT_3:
            if(flagLawn == 1)
                Speed_State = SAMPLE;
            else
                Speed_State = INIT_3;
            break;
        case SAMPLE:
            Speed_State = SAMPLE;
            break;
        default:
            break;
    }
    
}

void PID_Init() {
    //Serial.begin(9600);
    //attachInterrupt(digitalPinToInterrupt(interruptR), rightEncoder, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(interruptL), leftEncoder, CHANGE);

    PID_State = INIT_4;
}

void PID_Tick() {
    static int error = 0;
    //Actions
    switch(PID_State) {
        case INIT_4:
            break;
        case PCONT:
            if (lSpeed < rSpeed) {
                Serial.println("Here");
                error = rSpeed - lSpeed;
                error = error * Kp;
                speedAdjustR = RFULL_SPEED - error;
                speedAdjustL = LFULL_SPEED;
            }
            else if (rSpeed < lSpeed) {
                error = lSpeed - rSpeed;
                error = error * Kp;
                speedAdjustL = LFULL_SPEED - error;
                speedAdjustR = RFULL_SPEED;
            }
            else {
                speedAdjustR = RFULL_SPEED;
                speedAdjustL = LFULL_SPEED;
            }
            break;
        default:
            break;
    }
    
    //Transitions
    switch(PID_State) {
        case INIT_4:
            if(flagLawn == 1)
                PID_State = PCONT;
            else
                PID_State = INIT_4;
            break;
        case PCONT:
            PID_State = PCONT;
            break;
        default:
            break;
    }
}

void setSpeed_Init() {
    //Serial.begin(9600);
    SET_State = INIT_5;
}

void setSpeed_Tick() {
    
    //Actions
    switch(SET_State) {
        case INIT_5:
            break;
        case SET:
            motor1->setSpeed(speedAdjustL);
            motor2->setSpeed(speedAdjustR);
            /*
            Serial.print("Right Speed:" );
            Serial.println(speedAdjustR);
            Serial.print("Left Speed:" );
            Serial.println(speedAdjustL);
             */
            break;
        default:
            break;
    }
    
    //Transitions
    switch(SET_State) {
        case INIT_5:
            if(flagLawn == 1)
                SET_State = SET;
            else
                SET_State = INIT_5;
            break;
        case SET:
            SET_State = SET;
            break;
        default:
            break;
    }
}


#endif
