#include <Arduino_FreeRTOS.h>
#include <Tasks.h>
#include <PIDTask.h>

void TaskPing(void *pvParameters);
void TaskBotRandom(void *pvParameters);
void TaskGetSpeed(void *pvParameters);
void TaskPID(void *pvParameters);
void TaskSetSpeed(void *pvParameters);
void TaskLawn(void *pvParameters);
void TaskBluetooth(void *pvParameters);
void TaskServo(void *pvParameters);
void TaskUI(void *pvParameters);

void setup() {

xTaskCreate (
  TaskPing
    , (const portCHAR *) "Ping"
    , 128 //Stack size
    , NULL
    , 2 //Priority
    , NULL);
xTaskCreate (
  TaskUI
    , (const portCHAR *) "UI"
    , 128 //Stack size
    , NULL
    , 1 //Priority
    , NULL);
    
xTaskCreate (
  TaskBotRandom
    , (const portCHAR *) "Bot"
    , 128 //Stack size
    , NULL
    , 1 //Priority
    , NULL);

xTaskCreate (
  TaskServo
    , (const portCHAR *) "Servo"
    , 128 //Stack size
    , NULL
    , 1 //Priority
    , NULL);

xTaskCreate (
  TaskGetSpeed
    , (const portCHAR *) "Bot"
    , 128 //Stack size
    , NULL
    , 2 //Priority
    , NULL);

xTaskCreate (
  TaskPID
    , (const portCHAR *) "Bot"
    , 128 //Stack size
    , NULL
    , 2 //Priority
    , NULL);

xTaskCreate (
  TaskSetSpeed
    , (const portCHAR *) "Bot"
    , 128 //Stack size
    , NULL
    , 2 //Priority
    , NULL);
 
xTaskCreate (
  TaskLawn
    , (const portCHAR *) "Ping"
    , 128 //Stack size
    , NULL
    , 2 //Priority
    , NULL);

xTaskCreate (
  TaskBluetooth
    , (const portCHAR *) "Bluetoth"
    , 128 //Stack size
    , NULL
    , 2 //Priority
    , NULL);

}


void loop() {
}

void TaskPing(void *pvParameters) //This is a task
{
  (void) pvParameters;

  PING_Init();

  for (;;) {
    PING_Tick();
    vTaskDelay(1); //portTICK_PERIOD_MS
  }
}


void TaskBotRandom(void *pvParameters) //This is a task
{
  (void) pvParameters;

  rBOT_Init();

  for (;;) {
    rBOT_Tick();
    vTaskDelay(4); //portTICK_PERIOD_MS
  }
}


void TaskServo(void *pvParameters) //This is a task
{
  (void) pvParameters;

  Servo_Init();

  for (;;) {
    Servo_Tick();
    vTaskDelay(1); //portTICK_PERIOD_MS
  }
}

void TaskUI(void *pvParameters) //This is a task
{
  (void) pvParameters;

  UI_Init();

  for (;;) {
    UI_Tick();
    vTaskDelay(4); //portTICK_PERIOD_MS
  }
}

void TaskGetSpeed(void *pvParameters) //This is a task
{
  (void) pvParameters;

  getSpeed_Init();

  for (;;) {
    getSpeed_Tick();
    vTaskDelay(2); //portTICK_PERIOD_MS
  }
}

void TaskPID(void *pvParameters) //This is a task
{
  (void) pvParameters;

  PID_Init();

  for (;;) {
    PID_Tick();
    vTaskDelay(2); //portTICK_PERIOD_MS
  }
}

void TaskSetSpeed(void *pvParameters) //This is a task
{
  (void) pvParameters;

  setSpeed_Init();

  for (;;) {
    setSpeed_Tick();
    vTaskDelay(2); //portTICK_PERIOD_MS
  }
  
}

void TaskLawn(void *pvParameters) //This is a task
{
  (void) pvParameters;

  Lawn_Init();

  for (;;) {
    Lawn_Tick();
    vTaskDelay(4); //portTICK_PERIOD_MS
  }
  
}

void TaskBluetooth(void *pvParameters) //This is a task
{
  (void) pvParameters;

  Bluetooth_Init();

  for (;;) {
    Bluetooth_Tick();
    vTaskDelay(4); //portTICK_PERIOD_MS
  }
  
}
