#include <Adafruit_MotorShield.h>

int trigPin1 = 9; //front sensor
int echoPin1 = 22; 
int trigPin2 = 3; //right sensor
int echoPin2 = 24;
int trigPin3 = 10; //left sensor
int echoPin3 = 26 ;

int firstTurn = 0; //right = 0, left = 1
int prevTurn = 0; //right = 0, left = 1

long randNum = 0;
int ranSeed = 1002;

long counter = 0;
long oneSec = 0;

long duration1;
long duration2;
long duration3;

long frontSensorReading;
long rightSensorReading;
long leftSensorReading;

long frontSensorReading1, frontSensorReading2, frontSensorReading3;
long rightSensorReading1, rightSensorReading2, rightSensorReading3;
long leftSensorReading1, leftSensorReading2, leftSensorReading3;

long avgFrontSensorReading;
long avgRightSensorReading;
long avgLeftSensorReading;

int cornersDetected = 0;
int numIterations = 0;

int encoderLeft = 18; //interrupt pin
int encoderRight = 19; //interrupt pin

volatile long tickRight = 0;
volatile long tickLeft = 0;
volatile long turnRight = 0;
volatile long turnLeft = 0;

int turn90_ticks = 50;

byte turning = LOW;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motor1 = AFMS.getMotor(1); //M1 (DC Motor 1)
Adafruit_DCMotor *motor2 = AFMS.getMotor(3); //M3 (DC Motor 2)
//Adafruit_DCMotor *motor3 = AFMS.getMotor(3); //M3 (DC Motor for Brush)

void riseRight() //ISR
{
  if (turning == HIGH) 
  {
    turnRight++;
  }
  else if (turning == LOW) 
  {
    tickRight++;
  }
}

void riseLeft() //ISR
{
  if (turning == HIGH) 
  {
    turnLeft++;
  }
  else if (turning == LOW) 
  {
    tickLeft++;
  }
}

void resetTurn()
{
  turnLeft = 0;
  turnRight = 0;
}

void resetTick()
{
  tickLeft = 0;
  tickRight = 0;
}

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

void rightTurn()
{
  motor1->run(FORWARD);
  motor2->run(BACKWARD);
}

void leftTurn()
{
  motor1->run(BACKWARD);
  motor2->run(FORWARD);
}

//turn right 90 degrees
void rightTurn90(int tickL, int tickR)
{
  turning = HIGH;
  resetTurn();

  while(turnLeft < tickL || turnRight < tickR)
  {
    rightTurn();
  }
  
  turning = LOW;
  resetTurn();
  resetTick();
}

//turn left 90 degrees
void leftTurn90(int tickL,int tickR)
{
  turning = HIGH;
  resetTurn();

  while(turnLeft < tickL || turnRight < tickR)
  {
    leftTurn();
  }
  
  turning = LOW;
  resetTurn();
  resetTick();
}

void halt()
{
  motor1->run(RELEASE);
  motor2->run(RELEASE);
}

void brushMotor()
{
  
}

void servoArm()
{
   
}

//get initial sensor reading to find orientation of the robot
void initialSensorReading()
{
  avgFrontSensorReading = avgFrontSensorReadings();
  avgRightSensorReading = avgRightSensorReadings();
  avgLeftSensorReading = avgLeftSensorReadings();
  
  if (avgLeftSensorReading > avgFrontSensorReading) 
  {
    //initial position is either bottom left or top right
    firstTurn = 0; //right
  }
  else if (avgRightSensorReading > avgFrontSensorReading) 
  {
    //initial position is either top left or bottom right
    firstTurn = 1; //left
  }
}

//get readings from ultrasonic sensors
long frontSensorInput()
{
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  duration1 = pulseIn(echoPin1, HIGH); //read input from sensor
  frontSensorReading = (duration1 / 2) / 29.1;

  return frontSensorReading;
}
long rightSensorInput()
{
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  duration2 = pulseIn(echoPin2, HIGH); //read input from sensor
  rightSensorReading = (duration2 / 2) / 29.1;
  
  return rightSensorReading;
}

long leftSensorInput()
{
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);

  duration3 = pulseIn(echoPin3, HIGH); //read input from sensor
  leftSensorReading = (duration3 / 2) / 29.1;

  return leftSensorReading;
}

long avgFrontSensorReadings()
{
  frontSensorReading1 = frontSensorInput();
  frontSensorReading2 = frontSensorInput();
  frontSensorReading3 = frontSensorInput();
  
  return (frontSensorReading1 + frontSensorReading2 + frontSensorReading3) / 3;
}

long avgRightSensorReadings()
{
  rightSensorReading1 = rightSensorInput();
  rightSensorReading2 = rightSensorInput();
  rightSensorReading3 = rightSensorInput();

  return (rightSensorReading1 + rightSensorReading2 + rightSensorReading3) / 3;
}

long avgLeftSensorReadings()
{
  leftSensorReading1 = leftSensorInput();
  leftSensorReading2 = leftSensorInput();
  leftSensorReading3 = leftSensorInput();

  return (leftSensorReading1 + leftSensorReading2 + leftSensorReading3) / 3;
}

void outputAvgFrontSensorReadings()
{
  Serial.print("Front Sensor Reading 1: ");
  Serial.println(frontSensorReading1);
  Serial.print("Front Sensor Reading 2: ");
  Serial.println(frontSensorReading2);
  Serial.print("Front Sensor Reading 3: ");
  Serial.println(frontSensorReading3);
  Serial.print("Average Front Sensor Reading: ");
  Serial.println(avgFrontSensorReading);
}

void outputAvgRightSensorReadings()
{
  Serial.print("Right Sensor Reading 1: ");
  Serial.println(rightSensorReading1);
  Serial.print("Right Sensor Reading 2: ");
  Serial.println(rightSensorReading2);
  Serial.print("Right Sensor Reading 3: ");
  Serial.println(rightSensorReading3);
  Serial.print("Average Right Sensor Reading: ");
  Serial.println(avgRightSensorReading);
}

void outputAvgLeftSensorReadings()
{
  Serial.print("Left Sensor Reading 1: ");
  Serial.println(leftSensorReading1);
  Serial.print("Left Sensor Reading 2: ");
  Serial.println(leftSensorReading2);
  Serial.print("Left Sensor Reading 3: ");
  Serial.println(leftSensorReading3);
  Serial.print("Average Left Sensor Reading: ");
  Serial.println(avgLeftSensorReading);
}

void detectCorner() //if 3 corners are detected, the entire solar panel was traversed
{
  avgFrontSensorReading = avgFrontSensorReadings();
  avgRightSensorReading = avgRightSensorReadings();
  avgLeftSensorReading = avgLeftSensorReadings();

  if ((avgFrontSensorReading > 20 && avgLeftSensorReading > 20) || 
      (avgFrontSensorReading > 20 && avgRightSensorReading > 20))
  {
    cornersDetected = cornersDetected + 1;
  }

  if (cornersDetected == 3)
  {
    halt();
    delay(1000);
  }
}

void lawnmowerTest()
{
  avgFrontSensorReading = avgFrontSensorReadings();
  outputAvgFrontSensorReadings();
  
  if (avgFrontSensorReading >= 20)
  {
    halt();
  }
  else if (avgFrontSensorReading < 20)
  {
    forward();
  }
}

void lawnmowerPattern(long avgFrontSensorReading, long avgRightSensorReading, long avgLeftSensorReading) 
{
  if (avgFrontSensorReading >= 20)
  {
    halt();
    delay(500);

    //turn right or left
    if (prevTurn == 1 && avgRightSensorReading < 20) //turn right
    {
      //Serial.println("right");
      rightTurn();
      delay(500); //however long it takes to turn 90 degrees
      forward();
      delay(500);
      rightTurn();
      delay(500); //however long it takes to turn 90 degrees
      prevTurn = 0; //so it turns left o
      forward();
    }
    else if (prevTurn == 0 && avgLeftSensorReading < 20) //turn left
    {
      Serial.println("left");
      leftTurn();
      delay(500); //however long it takes to turn 90 degrees
      forward();
      delay(500);
      leftTurn();
      delay(500); //however long it takes to turn 90 degrees
      prevTurn = 1; //so it turns right on the next turn
      forward();
    }
  }
  else 
  {
    forward();
    Serial.println("forward");
    delay(1000);
  }
}

void randomPattern()
{
  avgFrontSensorReading = avgFrontSensorReadings();
  avgRightSensorReading = avgRightSensorReadings();
  avgLeftSensorReading = avgLeftSensorReadings();

  if (avgFrontSensorReading >= 20)
  {
    outputAvgFrontSensorReadings();
    Serial.print("halt"); 
    halt();
    delay(1000);
    Serial.print("reverse");
    reverse();
    delay(1000);
    
    //can turn right or left
    if (avgRightSensorReading < 20 && avgLeftSensorReading < 20)
    {
      randNum = random(100);
      if (randNum > 50)
      {
        Serial.print("left");
        leftTurn();
        delay(1000);
      }
      else if (randNum <= 50)
      {
        Serial.print("right");
        rightTurn();
        delay(1000);
      }
    }
    //can only turn left
    else if (avgRightSensorReading >= 20 && avgLeftSensorReading < 20)
    {
      Serial.print("left");
      leftTurn();
      delay(1000);
    }
    //can only turn right
    else if (avgLeftSensorReading >= 20 && avgRightSensorReading < 20)
    {
      Serial.print("right");
      rightTurn();
      delay(1000);
    }
  }
  else if (avgFrontSensorReading < 20)
  {
    outputAvgFrontSensorReadings();
    Serial.print("forward");
    forward();
  }
}

void manualMode()
{
  
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  //random seed
  randomSeed(ranSeed); 

  //input and output for ultrasonic sensor
  pinMode(trigPin1, OUTPUT); //front sensor
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); //right sensor
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT); //left sensor
  pinMode(echoPin3, INPUT);
  
  AFMS.begin(); //initialize the shield with default freq of 1.6 KHz

  //initialize motor speed
  motor1->setSpeed(75); //DC Motor 1
  motor2->setSpeed(75); //DC Motor 2
  //motor3->setSpeed(100); //DC Motor 3 for Brush

  attachInterrupt(digitalPinToInterrupt(encoderLeft), riseLeft, RISING); //RISING - to trigger when pin goes from low to high
  attachInterrupt(digitalPinToInterrupt(encoderRight), riseRight, RISING);
  
  //initialSensorReading();
}

void loop() 
{ 
  randomPattern();
  
  /*detectCorner();

  if (cornersDetected < 3)
  {
    lawnmowerPattern();
  }
  else if (cornersDetected == 3) //traversed panel one time
  {
    halt();
    delay(1000);
    //uturn
    numIterations = numIterations + 1;
    cornersDetected = 0;
  }*/
}
