#include <NewPing.h>
#include <Adafruit_MotorShield.h>

#define leftTRIG 10
#define leftECHO 26
#define frontTRIG 9
#define frontECHO 22
#define rightTRIG 3
#define rightECHO 24
#define interruptR 19
#define interruptL 18
//#define MAX_DIS 500
//#define MAX_DIS_FRONT 6 //distance from sensor to solar panel

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motor1 = AFMS.getMotor(1); //M1 (DC Motor 1)
Adafruit_DCMotor *motor2 = AFMS.getMotor(3); //M3 (DC Motor 2)
//Adafruit_DCMotor *motor3 = AFMS.getMotor(3); //M3 (DC Motor for Brush)

//Ultrasonic Sensor Objects
NewPing leftPing(leftTRIG, leftECHO, MAX_DIS);
NewPing frontPing(frontTRIG, frontECHO, MAX_DIS);
NewPing rightPing(rightTRIG, rightECHO, MAX_DIS);

long avgFrontSensorReading;
long frontSensorReading1;
long frontSensorReading2;
long frontSensorReading3;

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

void outputFrontSensorReadings()
{
  Serial2.print("Front Sensor Reading 1: ");
  Serial2.println(frontSensorReading1);
  Serial2.print("Front Sensor Reading 2: ");
  Serial2.println(frontSensorReading2);
  Serial2.print("Front Sensor Reading 3: ");
  Serial2.println(frontSensorReading3);
  Serial2.print("Average Front Sensor Reading: ");
  Serial2.println(avgFrontSensorReadings());
}

void traversePane_1() 
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

void traversePanel_2()
{
  avgFrontSensorReading = avgFrontSensorReadings();

  if (avgFrontSensorReading > 0) 
  {
    forward();
  }
  else if (avgFrontSensorReading == 0) // No panel
  {
    halt();
    delay(500);
    reverse();
    delay(500);
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
  Serial2.begin(9600);
  
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
  // put your main code here, to run repeatedly:
  //turnRight45();
  //delay(3000);
  //Serial.print("Front Sensor: ");
  //Serial.print(frontPing.ping_cm());
  //Serial.println("cm");
  //delay(50);
 
  //forward();
  //traversePanel();
  
  Serial2.println(avgFrontSensorReadings());
  delay(50);
  //outputFrontSensorReadings();
}
