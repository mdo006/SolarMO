#include <NewPing.h>
#include <Adafruit_MotorShield.h>

#define leftTRIG 3
#define leftECHO 24
#define frontTRIG 9
#define frontECHO 22
#define rightTRIG 10
#define rightECHO 26
#define interruptR 19
#define interruptL 18
#define MAX_DIS 500
#define MAX_DIS_LEFT 8
#define MAX_DIS_FRONT 6 //distance from sensor to solar panel
#define MAX_DIS_RIGHT 8

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motor1 = AFMS.getMotor(1); //M1 (DC Motor 1)
Adafruit_DCMotor *motor2 = AFMS.getMotor(3); //M3 (DC Motor 2)
//Adafruit_DCMotor *motor3 = AFMS.getMotor(3); //M3 (DC Motor for Brush)

//Ultrasonic Sensor Objects
NewPing leftPing(leftTRIG, leftECHO, MAX_DIS_LEFT);
NewPing frontPing(frontTRIG, frontECHO, MAX_DIS_FRONT);
NewPing rightPing(rightTRIG, rightECHO, MAX_DIS_RIGHT);

long avgFrontSensorReading;
long frontSensorReading1;
long frontSensorReading2;
long frontSensorReading3;

long avgRightSensorReading;
long rightSensorReading1;
long rightSensorReading2;
long rightSensorReading3;

long avgLeftSensorReading;
long leftSensorReading1;
long leftSensorReading2;
long leftSensorReading3;


long turnTick = 100;
volatile long rightTick = 0;
volatile long leftTick = 0;

char input = ' '; //for bluetooth control

void forward() 
{
  motor1->run(FORWARD);
  motor2->run(FORWARD);
}

void reverse() 
{
  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
}

void halt() 
{
  motor1->run(RELEASE);
  motor2->run(RELEASE);
}

void leftTurn() 
{
  motor1->run(FORWARD);
  motor2->run(BACKWARD);

}

void rightTurn() 
{
  motor1->run(BACKWARD);
  motor2->run(FORWARD);
}

void turnRight45() 
{
  long prevTickR = rightTick;
  long prevTickL = leftTick;

  while(rightTick - prevTickR <= turnTick || leftTick - prevTickL <= turnTick) {
    rightTurn();
  }
  halt();
}

void turnLeft45() 
{
  long prevTickR = rightTick;
  long prevTickL = leftTick;

  while(rightTick - prevTickR <= turnTick || leftTick - prevTickL <= turnTick) {
    leftTurn();
  }
  halt();
}

void rightEncoder() 
{
  rightTick++;
}

void leftEncoder() 
{
  leftTick++;
}

long avgFrontSensorReadings() 
{
  frontSensorReading1 = frontPing.ping_cm();
  delay(50);
  frontSensorReading2 = frontPing.ping_cm();
  delay(50);
  frontSensorReading3 = frontPing.ping_cm();
  delay(50);

  return (frontSensorReading1 + frontSensorReading2 + frontSensorReading3) / 3;
}

long avgRightSensorReadings() 
{
  rightSensorReading1 = rightPing.ping_cm();
  delay(50);
  rightSensorReading2 = rightPing.ping_cm();
  delay(50);
  rightSensorReading3 = rightPing.ping_cm();
  delay(50);

  return (rightSensorReading1 + rightSensorReading2 + rightSensorReading3) / 3;
}

long avgLeftSensorReadings() 
{
  leftSensorReading1 = leftPing.ping_cm();
  delay(50);
  leftSensorReading2 = leftPing.ping_cm();
  delay(50);
  leftSensorReading3 = leftPing.ping_cm();
  delay(50);

  return (leftSensorReading1 + leftSensorReading2 + leftSensorReading3) / 3;
}

void outputFrontSensorReadings()
{
  Serial.print("Front Sensor Reading 1: ");
  Serial.println(frontSensorReading1);
  Serial.print("Front Sensor Reading 2: ");
  Serial.println(frontSensorReading2);
  Serial.print("Front Sensor Reading 3: ");
  Serial.println(frontSensorReading3);
  Serial.print("Average Front Sensor Reading: ");
  Serial.println(avgFrontSensorReadings());
}

void traversePanel_1() //traversePanel algorithm 1st try (no edge cases to detect edges)
{
  avgFrontSensorReading = avgFrontSensorReadings();
  
  if (avgFrontSensorReading < 20) //change avgFrontSensorReading to frontPing.ping_cm(); (Gian Version)
  { 
    forward();
  }
  else 
  {
    halt();
    delay(500);
    reverse();
    delay(500);
    turnLeft45();
  }
}

void traversePanel()
{
  avgFrontSensorReading = avgFrontSensorReadings();
  avgRightSensorReading = avgRightSensorReadings();
  avgLeftSensorReading = avgLeftSensorReadings();
  
  Serial.println(avgFrontSensorReading);
  if (avgFrontSensorReading > 0) 
  {
    if (avgRightSensorReading == 0) //make sure it doesn't get too close to the right else
    {
      halt();
      delay(500);
      reverse();
      delay(500);
      Serial.println("Turn left away from the edge");
      turnLeft45();
    }
    else if (avgLeftSensorReading == 0) //make sure it doesn't get too close to the left else
    {
      halt();
      delay(500);
      reverse();
      delay(500);
      Serial.println("Turn right away from the edge");
      turnRight45();
    }
    Serial.println("forward");
    forward();
  }
  else if (avgFrontSensorReading == 0) //no panel
  {
    Serial.println("halt"); 
    halt();
    delay(500);
    reverse();
    delay(500);
    Serial.println("Turn Left");
    turnLeft45();
  }
}

void manualMode() 
{
  //if (Serial2.available()) 
  if (Serial2.available())
  {
    //input = Serial2.read();
    input = Serial2.read();

    if (input == 'w') {
      forward();  
    }
    else if (input == 'a'){
      turnLeft45();
    }
    else if (input == 'd'){
      turnRight45();
    }
    else if (input == 's'){
      reverse();
    }
    else{
      halt();
    }
  }
}

void setup() 
{
  // put your setup code here, to run once:
  //Serial2.begin(9600);
  Serial.begin(9600);
  
  AFMS.begin(); //initialize the shield with default freq of 1.6 KHz

  pinMode(frontTRIG, OUTPUT);
  pinMode(frontECHO, INPUT);
  
  //initialize motor speed
  motor1->setSpeed(125); //DC Motor 1
  motor2->setSpeed(125); //DC Motor 2
  
  attachInterrupt(digitalPinToInterrupt(interruptR), rightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptL), leftEncoder, CHANGE);
}

void loop() 
{
  traversePanel();
  delay(50);

  /*Serial.print("Front Sensor: ");
  Serial.println(frontPing.ping_cm());
  delay(100);
  Serial.print("Right Sensor: ");
  Serial.println(rightPing.ping_cm());
  delay(100);
  Serial.print("Left Sensor: ");
  Serial.println(leftPing.ping_cm());
  delay(100);*/
}
