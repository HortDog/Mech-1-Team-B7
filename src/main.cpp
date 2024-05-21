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

// Button pins
#define KEY_2 4
#define KEY_1 5
#define LimitSwitch 11

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
  xTaskCreate(TaskStateMachine, "StateMachine", 256, NULL, 20, &StateMachine);

  //xTaskCreate(TaskIdleBlink, "IdleBlink", 256, NULL, 10, &IdleBlink);

 // xTaskCreate(TaskReadIR, "ReadIR", 512, NULL, 30, &ReadIR);

  //xTaskCreate(TaskMotorControl, "MotorControl", 256, NULL, 40, &MotorControl);
  // interrupt tasks
  //xTaskCreate(TaskHandleKey1Func, "HandleKey1", 256, NULL, 1, &HandleKey1);
  //xTaskCreate(TaskHandleKey2Func, "HandleKey2", 256, NULL, 1, &HandleKey1);

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
        //vTaskResume(IdleBlink);
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
      case STRATE:
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
    //take semaphore and write to it, then release it
    if(xSemaphoreTake(irValuesMutex, 0) == pdTRUE) {
      irValuesLocal = irValues;
      xSemaphoreGive(irValuesMutex);
    }
    // Do motor control based on irValues
    // get the line vector from irValues uint8
    int line_vector = 0;
    for (int i = 0; i < 8; i++) {
      // if the IR sensor is on the line, add the weight to the line vector
      if (irValuesLocal & (1 << i)) {
        line_vector += i;
      }
    }
    printf("Line vector: %d\n", line_vector);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Callback function for key 1 interrupt, called by the ISR
void handleKey1Interrupt() {
  //BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  //vTaskResumeFromISR(HandleKey1, &xHigherPriorityTaskWoken);
}
// Task to handle key 1 press
void TaskHandleKey1Func(void *pvParameters) {
  (void) pvParameters;
  vTaskSuspend(NULL); // Suspend the task that is currently running
  printf("Key 1 pressed\n");
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
