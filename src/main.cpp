#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOSVariant.h>
#include <heap_3.c>
#include <task.h>
#include <queue.h>
#include <semphr.h>

// define pin out
// LED pins, On-board LED is pin 6
#define ONBOAD_LED_PIN LED_BUILTIN
#define LED_1 1
#define LED_2 2
#define LED_3 3

// Motor pins
#define MOTOR_PWM_PIN_1 10
#define MOTOR_PWM_PIN_2 9
#define MOTOR_DIR_PIN_1 7
#define MOTOR_DIR_PIN_2 8

// IR Array pins
#define IR_1 A0
#define IR_2 A1
#define IR_3 A2
#define IR_4 A3
#define IR_5 A4
#define IR_6 A5
#define IR_7 A6
#define IR_8 A7
#define IR_ON 0

// Button pins
#define KEY_2 4
#define KEY_1 5

// State machine states
typedef enum {
  IDLE,
  STRATE,
  CURVE,
  SLOW_ZONE,
  FALLEN_BRANCH
} volatile state_t;
state_t state = IDLE;

TaskHandle_t StateMachine;
void TaskStateMachine(void *pvParameters);

TaskHandle_t HandleKey1;
void TaskHandleKey1Func(void *pvParameters);
void handleKey1Interrupt();

TaskHandle_t HandleKey2;
void TaskHandleKey2Func(void *pvParameters);
void handleKey2Interrupt();

TaskHandle_t IdleBlink;
void TaskIdleBlink(void *pvParameters);

// Global array for IR values
int irValues[8];

void setup() {
  Serial.begin(9600);
  
  // Initialize LED pins
  pinMode(ONBOAD_LED_PIN, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);

  // Initialize Button pins and their interrupts
  pinMode(KEY_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KEY_1), handleKey1Interrupt, FALLING);
  
  pinMode(KEY_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KEY_2), handleKey2Interrupt, FALLING);
  
  // Create tasks
  // main tasks
  xTaskCreate(TaskStateMachine, "StateMachine", 256, NULL, 2, &StateMachine);

  xTaskCreate(TaskIdleBlink, "IdleBlink", 256, NULL, 10, &IdleBlink);
  // interrupt tasks
  xTaskCreate(TaskHandleKey1Func, "HandleKey1", 256, NULL, 1, &HandleKey1);
  xTaskCreate(TaskHandleKey2Func, "HandleKey2", 256, NULL, 1, &HandleKey1);

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();
}

void loop() {
  // Empty loop, tasks are executed by the scheduler
}

/*-----TASKS-----*/

void TaskStateMachine(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    switch (state) {
      case IDLE:
        // Do nothing
        vTaskResume(IdleBlink);
        break;
      case STRATE:
        // Do line following on straight
        vTaskSuspend(IdleBlink);
        break;
      case CURVE:
        // Do curve following on curve
        break;
      case SLOW_ZONE:
        // Do slow zone following on slow zone
        break;
      case FALLEN_BRANCH:
        // Do obsical avoidance
        break;        
      default:
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Callback function for key 1 interrupt, called by the ISR
void handleKey1Interrupt() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskResumeFromISR(HandleKey1, &xHigherPriorityTaskWoken);
}
// Task to handle key 1 press
void TaskHandleKey1Func(void *pvParameters) {
  (void) pvParameters;
  vTaskSuspend(NULL); // Suspend the task that is currently running
  printf("Key 1 pressed\n");
}

void TaskIdleBlink(void *pvParameters) {
  (void) pvParameters;
  
  while (1) {
    for (int i = 0; i < 2; i++) {
      digitalWrite(ONBOAD_LED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(200));
      digitalWrite(ONBOAD_LED_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    for (int i = 0; i < 2; i++) {
      digitalWrite(ONBOAD_LED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(ONBOAD_LED_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}
