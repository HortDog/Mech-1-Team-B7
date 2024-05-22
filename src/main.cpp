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
#define DashLeft A8
#define DashRight A9

volatile uint8_t irValues;
SemaphoreHandle_t irValuesMutex = NULL;

volatile int throttle = 0;
SemaphoreHandle_t throttleMutex = NULL;

volatile int direction = 0;
SemaphoreHandle_t directionMutex = NULL;

// Button pins
#define KEY_2 4
#define KEY_1 5
#define LimitSwitch 11

// State machine states
typedef enum {
  IDLE,
  STRAIGHT,
  CURVE,
  SLOW_ZONE,
  FALLEN_BRANCH
} volatile state_t;
state_t state = IDLE;

TaskHandle_t StateMachine;
void TaskStateMachine(void *pvParameters);

TaskHandle_t IdleBlink;
void TaskIdleBlink(void *pvParameters);

TaskHandle_t ReadIR;
void TaskReadIR(void *pvParameters);

TaskHandle_t MotorControl;
void TaskMotorControl(void *pvParameters);


void setup() {
  Serial.begin(9600);
  Serial.println("Serial communication started");

  // Semaphore for IR values
  irValuesMutex = xSemaphoreCreateMutex();
  xSemaphoreGive(irValuesMutex);

  // Initialize LED pins
  pinMode(ONBOAD_LED_PIN, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);

  // Initialize Button pins and their interrupts
  pinMode(KEY_1, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(KEY_1), handleKey1Interrupt, FALLING);
  
  pinMode(KEY_2, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(KEY_2), handleKey2Interrupt, FALLING);
  pinMode(LimitSwitch, INPUT_PULLUP);
  
  // Initialize Motor pins
  pinMode(MOTOR_PWM_PIN_1, OUTPUT);
  pinMode(MOTOR_PWM_PIN_2, OUTPUT);
  pinMode(MOTOR_DIR_PIN_1, OUTPUT);
  pinMode(MOTOR_DIR_PIN_2, OUTPUT);

  analogWrite(MOTOR_PWM_PIN_1, 0);
  analogWrite(MOTOR_PWM_PIN_2, 0);

  // Initialize IR Array pins
  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);
  pinMode(IR_3, INPUT);
  pinMode(IR_4, INPUT);
  pinMode(IR_5, INPUT);
  pinMode(IR_6, INPUT);
  pinMode(IR_7, INPUT);
  pinMode(IR_8, INPUT);
  pinMode(DashLeft, INPUT);
  pinMode(DashRight, INPUT);

  // Create tasks
  // main tasks
  xTaskCreate(TaskStateMachine, "StateMachine", 256, NULL, 10, &StateMachine);

  xTaskCreate(TaskIdleBlink, "IdleBlink", 256, NULL, 10, &IdleBlink);

  xTaskCreate(TaskReadIR, "ReadIR", 512, NULL, 10, &ReadIR);

  //xTaskCreate(TaskMotorControl, "MotorControl", 256, NULL, 10, &MotorControl);

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();
}

void loop() {
  // Empty loop, tasks are executed by the scheduler
  // Serial.println("ARRR");
}

/*-----TASKS-----*/

void TaskStateMachine(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    switch (state) {
      case IDLE:
        vTaskResume(IdleBlink);
        // LED_3 on if limit switch is pressed
        if (digitalRead(LimitSwitch) == LOW) {
          digitalWrite(LED_3, LOW);
          digitalWrite(LED_2, LOW);
          digitalWrite(LED_1, LOW);
        } else {
          digitalWrite(LED_3, HIGH);
          digitalWrite(LED_2, HIGH);
          digitalWrite(LED_1, HIGH);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        break;
      case STRAIGHT:
        // Do line following on straight
        //vTaskSuspend(IdleBlink);
        vTaskDelay(pdMS_TO_TICKS(10));
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
  }
}

void TaskMotorControl(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    uint8_t irValuesLocal;
    int directionLocal;
    int throttleLocal;
    //take semaphore and write to it, then release it
    if(xSemaphoreTake(irValuesMutex, 0) == pdTRUE) {
      irValuesLocal = irValues;
      xSemaphoreGive(irValuesMutex);
    }
    if(xSemaphoreTake(throttleMutex, 0) == pdTRUE) {
      throttleLocal = throttle;
      xSemaphoreGive(throttleMutex);
    }
    if (xSemaphoreTake(directionMutex, 0) == pdTRUE) {
      directionLocal = direction;
      xSemaphoreGive(directionMutex);
    }
    // Do motor control based on irValues
    // get the line vector from irValues uint8
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}

void TaskReadIR(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    uint8_t irValuesLocal;
    irValuesLocal = (digitalRead(IR_1) << 0) | (digitalRead(IR_2) << 1) | (digitalRead(IR_3) << 2) | (digitalRead(IR_4) << 3) | (digitalRead(IR_5) << 4) | (digitalRead(IR_6) << 5) | (digitalRead(IR_7) << 6) | (digitalRead(IR_8) << 7);
    //take semaphore and write to it, then release it
    if(xSemaphoreTake(irValuesMutex, 0) == pdTRUE) {
      irValues = irValuesLocal;
      xSemaphoreGive(irValuesMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
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
