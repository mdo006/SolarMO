#ifndef Gyro_h
#define Gyro_h
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

enum GyroState {INIT_8, CALIBRATE, CHECK} Gyro_State;

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

int counter = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


double ythresh;
double pthresh;
double rthresh;



void Gyro_Init() {
    Gyro_State = INIT_8;
    // join I2C bus (I2Cdev library doesn't do this automatically)
    
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
}

void Gyro_Tick() {
    
    //Actions
    switch(Gyro_State) {
        case INIT_8:
            break;
        case CALIBRATE:
            // if programming failed, don't try to do anything
            if (!dmpReady) return;
            
            // wait for MPU interrupt or extra packet(s) available
            while (!mpuInterrupt && fifoCount < packetSize) {
                // other program behavior stuff here
                // .
                // .
                // .
                // if you are really paranoid you can frequently test in between other
                // stuff to see if mpuInterrupt is true, and if so, "break;" from the
                // while() loop to immediately process the MPU data
                // .
                // .
                // .
            }
            
            // reset interrupt flag and get INT_STATUS byte
            mpuInterrupt = false;
            mpuIntStatus = mpu.getIntStatus();
            
            // get current FIFO count
            fifoCount = mpu.getFIFOCount();
            
            // check for overflow (this should never happen unless our code is too inefficient)
            if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                // reset so we can continue cleanly
                mpu.resetFIFO();
                Serial.println(F("FIFO overflow!"));
                
                // otherwise, check for DMP data ready interrupt (this should happen frequently)
            } else if (mpuIntStatus & 0x02) {
                // wait for correct available data length, should be a VERY short wait
                while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
                
                // read a packet from FIFO
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                
                // track FIFO count here in case there is > 1 packet available
                // (this lets us immediately read more without waiting for an interrupt)
                fifoCount -= packetSize;
                
                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                
                    ythresh = ypr[0];
                    pthresh = ypr[1];
                    rthresh = ypr[2];
                    Serial.println("---CALIBRATING---");

                // blink LED to indicate activity
                blinkState = !blinkState;
                digitalWrite(LED_PIN, blinkState);
            }
            counter++;
            break;
        case CHECK:
                // if programming failed, don't try to do anything
                if (!dmpReady) return;
                
                // wait for MPU interrupt or extra packet(s) available
                while (!mpuInterrupt && fifoCount < packetSize) {
                    // other program behavior stuff here
                    // .
                    // .
                    // .
                    // if you are really paranoid you can frequently test in between other
                    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
                    // while() loop to immediately process the MPU data
                    // .
                    // .
                    // .
                }
                
                // reset interrupt flag and get INT_STATUS byte
                mpuInterrupt = false;
                mpuIntStatus = mpu.getIntStatus();
                
                // get current FIFO count
                fifoCount = mpu.getFIFOCount();
                
                // check for overflow (this should never happen unless our code is too inefficient)
                if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                    // reset so we can continue cleanly
                    mpu.resetFIFO();
                    Serial.println(F("FIFO overflow!"));
                    
                    // otherwise, check for DMP data ready interrupt (this should happen frequently)
                } else if (mpuIntStatus & 0x02) {
                    // wait for correct available data length, should be a VERY short wait
                    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
                    
                    // read a packet from FIFO
                    mpu.getFIFOBytes(fifoBuffer, packetSize);
                    
                    // track FIFO count here in case there is > 1 packet available
                    // (this lets us immediately read more without waiting for an interrupt)
                    fifoCount -= packetSize;
                    
                    // display Euler angles in degrees
                    mpu.dmpGetQuaternion(&q, fifoBuffer);
                    mpu.dmpGetGravity(&gravity, &q);
                    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                Serial.print("Yaw: ");
                Serial.println(ypr[0] * (180/M_PI));
                }
                /*
                Serial.println((ythresh - ypr[0]) * 180/M_PI);
                if(((ythresh - ypr[0]) * 180/M_PI) >= 90.0) {
                    ythresh = ypr[0];
                }
                else if(((ythresh - ypr[0]) * 180/M_PI) <= -90.0) {
                    ythresh = ypr[0];
                }
                 */
        default:
            break;
        }

    
    //Transitions
    switch(Gyro_State) {
        case INIT_8:
            Gyro_State = CALIBRATE;
            break;
        case CALIBRATE:
            if (counter == 1000) {
                Gyro_State = CHECK;
                counter = 0;
            }
            else
                Gyro_State = CALIBRATE;
            break;
        case CHECK:
            Gyro_State = CHECK;
            break;
        default:
            break;
    }
    
}

#endif
