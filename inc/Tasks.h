#ifndef Tasks_h
#define Tasks_h

#include <Arduino.h>
#include <NewPing.h>
#include <Motors.h>
#include <Servo.h>

#define interruptR 19
#define interruptL 18
#define frontTRIG 9
#define frontECHO 22
#define rightTRIG 10 //was 10 v1
#define rightECHO 26 //was 26 v1
#define leftTRIG 3 //was 3 v1
#define leftECHO 24 //was 24 v1
#define fMAX_DIS 4
#define sideMAX_DIS 10

//variable declarations
extern int turnTick;
//extern int frontSensorAvg;

Servo myservo;

NewPing frontPing(frontTRIG, frontECHO, fMAX_DIS);
NewPing rightPing(rightTRIG, rightECHO, sideMAX_DIS);
NewPing leftPing(leftTRIG, leftECHO, sideMAX_DIS);

enum PINGState { INIT_1, PING_1, PING_2, PING_3 } PING_State;
enum rBOTState { INIT_2, FWD, BCK, rTURN, lTURN, TURN90, R_HALT } rBOT_State;
enum FWDState {INIT_6, FWD2} FWD_State;
enum LawnState {INIT_7, FWD3, BCK2, rTURN2, lTURN2, HALT, MOVEUP} Lawn_State;
enum ServoState {INIT_8, UP, DOWN, IDLE} Servo_State;
enum UIState {INIT_9, UI, END} UI_State;
enum BluetoothState {B_INIT, B_WAIT, B_MOVE} b_state;


//variable definitions(definitions should usually be done in the source file)
int tick90 = 160; //116 for 90 // 160 for 90
int tick45 = 60; //58 for 45
int frontTick = 90;
int backTick = 60;
int frontSensorAvg = 0;
int rightSensorAvg = 0;
int leftSensorAvg = 0;
int leftFlag = 0;
int rightFlag = 0;
int prevFlag = 0;
int haltcount = 0;
int angle = 0;
int targetMAX = 70;
int targetMIN = 75;
int flagHi = 0;
int flagRand = 0;
extern int flagLawn = 0;
int bluetoothFlag = 0; //bluetooth control off
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
            //Serial.print("Front: ");
            //Serial.println(frontSensorAvg);
            /*
            Serial.print("Right: ");
            Serial.println(rightSensorAvg);
            Serial.print("Left: ");
            Serial.println(leftSensorAvg);
             */
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

void Servo_Init() {
    
    myservo.attach(44);  // attaches the servo on pin 44 to the servo object
    angle = 80;
    myservo.write(angle);
    Servo_State = INIT_8;
}

void Servo_Tick() {
    static int counter = 0;
    //Actions
    switch(Servo_State) {
        case INIT_8:
            break;
        case UP:
            myservo.write(angle);
            angle--;
            break;
        case DOWN:
            myservo.write(angle);
            angle++;
            break;
        case IDLE:
            myservo.write(80);
            break;
        default:
            break;
    }
    
    //Transitions
    switch(Servo_State) {
        case INIT_8:
            if(flagHi == 1) {
                Servo_State = UP;
            }
            break;
        case UP:
            if (angle > targetMAX) {
                Servo_State = UP;
            }
            else {
                angle = 0;
                Servo_State = DOWN;
            }
            break;
        case DOWN:
            if(counter < 2) {
                if (angle < targetMIN) {
                    Servo_State = DOWN;
                }
                else if (angle > targetMIN){
                    angle = 0;
                    counter++;
                    Servo_State = UP;
                }
            }
            else {
                Servo_State = IDLE;
                counter = 0;
                flagHi = 0;
            }
            break;
        case IDLE:
            if(flagHi == 1) {
                Servo_State = UP;
            }
            else
                Servo_State = IDLE;
            break;
        default:
            break;
    }
}

void UI_Init() {
    UI_State = INIT_9;
}

void UI_Tick() {
    //Actions
    switch(UI_State) {
        case INIT_9:
            break;
        case UI:
            if(frontSensorAvg <= 0) {
                flagHi = 1;
                flagRand = 0;
                flagLawn = 0;
            }
            else if (leftSensorAvg <= 0) {
                flagRand = 1;
                flagHi = 0;
                flagLawn = 0;
            }
            else if (rightSensorAvg <= 0) {
                flagLawn = 1;
                flagHi = 0;
                flagRand = 0;
            }
            break;
        case END:
            break;
        default:
            break;
    }
    
    //Transitions
    switch(UI_State) {
        case INIT_9:
            UI_State = UI;
            break;
        case UI:
            if(flagRand == 1 || flagLawn == 1) {
                UI_State = END;
            }
            break;
        case END:
            UI_State = END;
            break;
        default:
            break;
    }

}

void rBOT_Init() {
    
    AFMS.begin(); //initialize the shield with default freq of 1.6 KHz
    
    //initialize motor speed
    motor1->setSpeed(100); //DC Motor 1
    motor2->setSpeed(100); //DC Motor 2
    //motor3->setSpeed(175); //DC Motor 3
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
            //Serial.println("FWD");
            forward();
            break;
        case BCK:
            //Serial.println("BCK");
            reverse();
            break;
        case rTURN:
            //Serial.println("rTURN");
            reverseAmt(backTick);
            turnRightAmt(tick45);
            break;
        case lTURN:
            //Serial.println("lTURN");
            reverseAmt(backTick);
            turnLeftAmt(tick45);
            break;
        case TURN90:
            turnRightAmt(tick90);
            break;
        case R_HALT:
            break;
        default:
            break;
    }
    
    //Transitions
    switch(rBOT_State) {
        case INIT_2:
            if(flagRand == 1)
                rBOT_State = FWD;
            else
                rBOT_State = INIT_2;
            break;
        case FWD:
            if (bluetoothFlag == 0) {
                //only the front sensor is extended out -> reverse
                if (frontSensorAvg <= 0 && rightSensorAvg > 0 && leftSensorAvg > 0) {
                    //halt();
                    rBOT_State = BCK;
                }
                //all of the sensors are extended out -> reverse
                else if(frontSensorAvg <= 0 && rightSensorAvg <= 0 && leftSensorAvg <= 0) {
                    rBOT_State = BCK;
                }
                else if (rightSensorAvg <= 0 && frontSensorAvg <= 0){
                    rBOT_State = BCK;
                }
                else if (leftSensorAvg <= 0 && frontSensorAvg <= 0) {
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
            }
            else if (bluetoothFlag == 1) {
                rBOT_State = R_HALT;
            }
            break;
        case BCK:
            if (bluetoothFlag == 0) {
                //when the front sensor doesn't hang from the edge then turn right
                if (frontSensorAvg <= 0)
                    rBOT_State = BCK;
                else
                    rBOT_State = TURN90;
            }
            else if (bluetoothFlag == 1) {
                rBOT_State = R_HALT;
            }
            break;
        case rTURN:
            if (bluetoothFlag == 0) {
                //when the left sensor doesn't hang from the edge then go forward
                if (leftSensorAvg <= 0)
                    rBOT_State = rTURN;
                else {
                    rBOT_State = FWD;
                }
                rBOT_State = FWD;
            }
            else if (bluetoothFlag == 1) {
                rBOT_State = R_HALT;
            }
            break;
        case lTURN:
            if (bluetoothFlag == 0) {
                //when the right sensor doesn't hang from the edge then go forward
                if (rightSensorAvg <= 0) {
                    rBOT_State = lTURN;
                }
                else {
                    rBOT_State = FWD;
                }
                //rBOT_State = FWD;
            }
            else if (bluetoothFlag == 1) {
                rBOT_State = R_HALT;
            }
            break;
        case TURN90:
            if (bluetoothFlag == 0) {
                if (frontSensorAvg <= 0)
                    rBOT_State = TURN90;
                else
                    rBOT_State = FWD;
            }
            else if (bluetoothFlag == 1) {
                rBOT_State = R_HALT;
            }
            break;
        case R_HALT:
            if (bluetoothFlag == 1) { //if using bluetooth control, stop doing pattern
                rBOT_State = R_HALT;
            }
            else if (bluetoothFlag == 0) { //if bluetooth control is off, do pattern
                rBOT_State = INIT_2;
            }
            break;
        default:
            break;
    }
}

void Bluetooth_Init()
{
    b_state = B_INIT;
}

void Bluetooth_Tick()
{
    //Transitions
    switch(b_state)
    {
        case B_INIT:
            b_state = B_WAIT;
            break;
            
        case B_WAIT:
            if (Serial.available())
            {
                input = Serial.read();
                Serial.println(input);
                
                if (input == 'b')
                {
                    Serial.println("Bluetooth Control On");
                    bluetoothFlag = 1;
                }
                else
                {
                    Serial.println("Bluetooth Control Off");
                    bluetoothFlag = 0;
                }
            }
            
            if (bluetoothFlag == 0) b_state = B_WAIT;
            else if (bluetoothFlag == 1) b_state = B_MOVE;
            break;
            
        case B_MOVE:
            if (bluetoothFlag == 1) b_state = B_MOVE;
            else if (bluetoothFlag == 0) b_state = B_WAIT;
            break;
            
        default:
            break;
    }
    
    //Actions
    switch(b_state)
    {
        case B_INIT:
            break;
            
        case B_WAIT:
            break;
            
        case B_MOVE:
            if (Serial.available())
            {
                input = Serial.read();
            }
            
            Serial.println(input);
            
            if (input == 'w') //forward
            {
                if (frontSensorAvg <= 0)
                {
                    input = ' '; //when it goes back to wait, it will go to halt
                }
                else
                {
                    Serial.println("Forward");
                    forward();
                }
            }
            else if (input == 'a') //left
            {
                if (leftSensorAvg <= 0)
                {
                    input = ' ';
                }
                else
                {
                    Serial.println("Left");
                    turnLeftAmt(tick45);
                    input = ' ';
                }
            }
            else if (input == 's') //reverse
            {
                Serial.println("Reverse");
                reverseAmt(backTick);
                input = ' ';
            }
            else if (input == 'd') //right
            {
                if (rightSensorAvg == 0)
                {
                    input = ' ';
                }
                else
                {
                    Serial.println("Right");
                    turnRightAmt(tick45);
                    input = ' ';
                }
            }
            else if (input == 'r') //bluetooth off
            {
                Serial.println("Bluetooth Control Off");
                bluetoothFlag = 0;
                input = ' ';
            }
            else
            {
                Serial.println("Halt");
                halt();
            }
            
            break;
            
        default:
            break;
    }
}

FWD_Init() {
    
    AFMS.begin(); //initialize the shield with default freq of 1.6 KHz
    
    //initialize motor speed
    motor1->setSpeed(125); //DC Motor 1
    motor2->setSpeed(125); //DC Motor 2
    
    Servo myservo;  // create servo object to control a servo
    // twelve servo objects can be created on most boards
    
    myservo.attach(44);  // attaches the servo on pin 44 to the servo object
    myservo.write(100);  //Angle
    
    FWD_State = INIT_6;
}

FWD_Tick() {
    
    //Actions
    switch(FWD_State) {
        case INIT_6:
            break;
        case FWD2:
            forward();
            break;
        default:
            break;
    }
    
    //Transitions
    switch(FWD_State) {
        case INIT_6:
            FWD_State = FWD2;
            break;
        case FWD2:
            FWD_State = FWD2;
            break;
        default:
            break;
    }
    
}

Lawn_Init() {
    myservo.attach(44);  // attaches the servo on pin 44 to the servo object
    
    AFMS.begin(); //initialize the shield with default freq of 1.6 KHz
    
    //initialize motor speed
    motor1->setSpeed(LFULL_SPEED); //Left Motor
    motor2->setSpeed(RFULL_SPEED); //Right Motor

    attachInterrupt(digitalPinToInterrupt(interruptR), rightEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(interruptL), leftEncoder, CHANGE);
    
    Lawn_State = INIT_7;
    leftFlag = 1;
    rightFlag = 0;
}

Lawn_Tick() {
    //Actions
    switch(Lawn_State) {
        case INIT_7:
            break;
        case FWD3:
            forward();
            break;
        case BCK2:
            reverse();
            break;
        case lTURN2:
            turnLeftAmt(tick90);
            break;
        case rTURN2:
            turnRightAmt(tick90);
            break;
        case MOVEUP:
            fwdAmt(frontTick);
            break;
        case HALT:
            //myservo.write(0);  //Angle
            halt();
            haltcount++;
            break;
        default:
            break;
    }
    
    //Transitions
    switch(Lawn_State) {
        case INIT_7:
            if (flagLawn == 1)
                Lawn_State = FWD3;
            else
                Lawn_State = INIT_7;
            break;
        case FWD3:
            //There is no panel
            if(frontSensorAvg <= 0) {
                Lawn_State = HALT;
                prevFlag = 1;
            }
            else
                Lawn_State = FWD3;
            break;
        case HALT:
            if (haltcount >= 5) {
                haltcount = 0; //reset halt counter
                if(prevFlag == 1) {
                    Lawn_State = BCK2;
                }
                else if(prevFlag == 2) {
                    if(leftFlag == 1 && rightFlag == 0) {
                        Lawn_State = lTURN2;
                        prevFlag = 3;
                    }
                    else if(leftFlag == 0 && rightFlag == 1) {
                        Lawn_State = rTURN2;
                        prevFlag = 3;
                    }
                }
                else if(prevFlag == 3) {
                    //check frontSensorHere to see if it reached the end
                    Lawn_State = MOVEUP;
                    prevFlag = 4;
                }
                else if(prevFlag == 4) {
                    if(leftFlag == 1 && rightFlag == 0) {
                        Lawn_State = lTURN2;
                        leftFlag = 0;
                        rightFlag = 1;
                        prevFlag = 5;
                    }
                    else if(leftFlag == 0 && rightFlag == 1) {
                        Lawn_State = rTURN2;
                        leftFlag = 1;
                        rightFlag = 0;
                        prevFlag = 5;
                    }
                }
                else if(prevFlag == 5) {
                    Lawn_State = FWD3;
                }
                    
            }
            else {
                Lawn_State = HALT;
            }
            break;
        case lTURN2:
            Lawn_State = HALT;
            break;
        case rTURN2:
            Lawn_State = HALT;
        case MOVEUP:
            Lawn_State = HALT;
            break;
        case BCK2:
            if(frontSensorAvg <= 0) {
                Lawn_State = BCK2;
            }
            else {
                Lawn_State = HALT;
                prevFlag = 2;
            }
            break;
        default:
            break;
    }
}


#endif
