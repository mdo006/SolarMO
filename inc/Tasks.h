#ifndef Tasks_h
#define Tasks_h

#include <Arduino.h>
#include <NewPing.h>
#include <Motors.h>

#define interruptR 19
#define interruptL 18
#define frontTRIG 9
#define frontECHO 22
#define rightTRIG 10
#define rightECHO 26
#define leftTRIG 3
#define leftECHO 24
#define fMAX_DIS 6
#define sideMAX_DIS 10

//variable declarations
extern int turnTick;
extern int frontSensorAvg;

NewPing frontPing(frontTRIG, frontECHO, fMAX_DIS);
NewPing rightPing(rightTRIG, rightECHO, sideMAX_DIS);
NewPing leftPing(leftTRIG, leftECHO, sideMAX_DIS);

enum PINGState { INIT_1, PING_1, PING_2, PING_3 } PING_State;
enum rBOTState { INIT_2, FWD, BCK, rTURN, lTURN, TURN90 } rBOT_State;
enum Bluetooth_State {B_WAIT, B_FORWARD, B_LEFT, B_REVERSE, B_RIGHT, B_HALT} b_state;

//variable definitions(definitions should usually be done in the source file)
int tick180 = 232;
int tick90 = 116;
int tick45 = 58;
int frontSensorAvg = 0;
int rightSensorAvg = 0;
int leftSensorAvg = 0;

char input = ' ';

void PING_Init() {
    Serial.begin(9600);
    PING_State = INIT_1;
}

//Samples data at each of the three states and averages them at the end
void PING_Tick() {
    static int fdata1;
    static int fdata2;
    static int fdata3;
    static int rdata1;
    static int rdata2;
    static int rdata3;
    static int ldata1;
    static int ldata2;
    static int ldata3;
    
    //Actions
    switch(PING_State) {
        case INIT_1:
            break;
        case PING_1:
            //Serial.println(frontPing.ping_cm());
            fdata1 = frontPing.ping_cm();
            rdata1 = rightPing.ping_cm();
            ldata1 = leftPing.ping_cm();
            break;
        case PING_2:
            fdata2 = frontPing.ping_cm();
            rdata2 = rightPing.ping_cm();
            ldata2 = leftPing.ping_cm();
            break;
        case PING_3:
            fdata3 = frontPing.ping_cm();
            rdata3 = rightPing.ping_cm();
            ldata3 = leftPing.ping_cm();
            frontSensorAvg = (fdata1 + fdata2 + fdata3) / 3;
            rightSensorAvg = (rdata1 + rdata2 + rdata3) / 3;
            leftSensorAvg = (ldata1 + ldata2 + ldata3) / 3;
            /*Serial.print("Front: ");
            Serial.println(frontSensorAvg);
            Serial.print("Right: ");
            Serial.println(rightSensorAvg);
            Serial.print("Left: ");
            Serial.println(leftSensorAvg);*/
            break;
        default:
            break;
    }
    
    //Transitions
    switch(PING_State) {
        case INIT_1:
            PING_State = PING_1;
            break;
        case PING_1:
            PING_State = PING_2;
            break;
        case PING_2:
            PING_State = PING_3;
            break;
        case PING_3:
            PING_State = PING_1;
        default:
            break;
    }
}

void rBOT_Init() {
    
    AFMS.begin(); //initialize the shield with default freq of 1.6 KHz
    
    //initialize motor speed
    motor1->setSpeed(125); //DC Motor 1
    motor2->setSpeed(125); //DC Motor 2
    attachInterrupt(digitalPinToInterrupt(interruptR), rightEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(interruptL), leftEncoder, CHANGE);
    rBOT_State = INIT_2;
}

void rBOT_Tick() {
    
    //Actions
    switch(rBOT_State) {
        case INIT_2:
            break;
        case FWD:
            Serial.println("FWD");
            forward();
            break;
        case BCK:
            Serial.println("BCK");
            reverse();
            break;
        case rTURN:
            Serial.println("rTURN");
            turnRightAmt(tick45);
            break;
        case lTURN:
            Serial.println("lTURN");
            turnLeftAmt(tick45);
            break;
        case TURN90:
            turnRightAmt(tick90);
            break;
        default:
            break;
    }
    
    //Transitions
    switch(rBOT_State) {
        case INIT_2:
            rBOT_State = FWD;
            break;
        case FWD:
            //only the front sensor is extended out -> reverse
            if (frontSensorAvg <= 0 && rightSensorAvg > 0 && leftSensorAvg > 0) {
                //halt();
                rBOT_State = BCK;
            }
            //all of the sensors are extended out -> reverse
            else if(frontSensorAvg <= 0 && rightSensorAvg <= 0 && leftSensorAvg <= 0) {
                rBOT_State = BCK;
            }
            //whenever the right sensor is extended out -> left turn
            else if (rightSensorAvg <= 0) {
                rBOT_State = lTURN;
            }
            //whenever the left sensor is extended out -> right turn
            else if (leftSensorAvg <= 0) {
                rBOT_State = rTURN;
            }
            else
                rBOT_State = FWD;
            break;
        case BCK:
            //when the front sensor doesn't hang from the edge then turn right
            if (frontSensorAvg <= 0)
                rBOT_State = BCK;
            else
                rBOT_State = TURN90;
            break;
        case rTURN:
            //when the left sensor doesn't hang from the edge then go forward
            if (leftSensorAvg <= 0)
                rBOT_State = rTURN;
            else {
                rBOT_State = FWD;
            }
            break;
        case lTURN:
            //when the right sensor doesn't hang from the edge then go forward
            if (rightSensorAvg <= 0) {
                rBOT_State = lTURN;
            }
            else {
                rBOT_State = FWD;
            }
            break;
        case TURN90:
            if (frontSensorAvg <= 0)
                rBOT_State = TURN90;
            else
                rBOT_State = FWD;
            break;
        default:
            break;
    }
}

void Bluetooth_Init()
{
  b_state = B_WAIT;
}

void Bluetooth_Tick()
{
  //Actions
  switch(b_state)
  {
    case B_WAIT:
      if (Serial.available())
      {
        input = Serial.read();
      }
      break;

    case B_FORWARD:
      if (frontSensorAvg == 0)
      {
        input = ' '; //when it goes back to wait, it will go to halt
      }
      else if (frontSensorAvg > 0)
      {
        forward(); 
      }
      break;

    case B_LEFT:
      if (leftSensorAvg== 0)
      {
        input = ' ';
      }
      else if (leftSensorAvg > 0)
      {
        turnLeftAmt(tick45);
        input = ' ';
      }
      break;

    case B_REVERSE:
      turnLeftAmt(tick180); //turn around and then stop
      input = ' ';
      break;

    case B_RIGHT:
      if (rightSensorAvg == 0)
      {
        input = ' ';
      }
      else if (rightSensorAvg > 0)
      {
        turnRightAmt(tick45);
        input = ' ';
      }
      break;

    case B_HALT:
      halt();
      break;

    default:
      break;
  }
  
  //Transitions
  switch(b_state)
  {
    case B_WAIT:
        if (input == 'w') //forward
        {
          b_state = B_FORWARD;
        }
        else if (input == 'a') //left
        {
          b_state = B_LEFT;
        }
        else if (input == 's') //reverse
        {
          b_state = B_REVERSE;
        }
        else if (input == 'd') //right
        {
          b_state = B_RIGHT;
        }
        else //halt
        {
          b_state = B_HALT;
        }
      break;

    case B_FORWARD:
      b_state = B_WAIT;
      break;

    case B_LEFT:
      b_state = B_WAIT;
      break;

    case B_REVERSE:
      b_state = B_WAIT;
      break;

    case B_RIGHT:
      b_state = B_WAIT;
      break;

    case B_HALT:
      b_state = B_WAIT;
      break;

    default:
      break;
  }
}

#endif
