#include <Arduino_FreeRTOS.h>
#include <Tasks.h>

void TaskPing(void *pvParameters);
void TaskBotRandom(void *pvParameters);
//void TaskPID(void *pvParameters);

void setup() {

xTaskCreate (
  TaskPing
    , (const portCHAR *) "Ping"
    , 128 //Stack size
    , NULL
    , 2 //Priority
    , NULL);

xTaskCreate (
  TaskBotRandom
    , (const portCHAR *) "Bot"
    , 128 //Stack size
    , NULL
    , 1 //Priority
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
