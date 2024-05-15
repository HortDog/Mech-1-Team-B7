#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOSVariant.h>
#include <heap_3.c>

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
  LINE_FOLLOW
} volatile state_t;

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

  // Initialize Button pins
  pinMode(KEY_1, INPUT);
  pinMode(KEY_2, INPUT);

  // Create tasks
  xTaskCreate(TaskIdleBlink, "IdleBlink", 256, NULL, 10, NULL);

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();
}

void loop() {
  // Empty loop, tasks are executed by the scheduler
}

/*-----TASKS-----*/

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
