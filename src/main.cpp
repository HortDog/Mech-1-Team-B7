#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

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

// Global array for IR values
int irValues[8];

void readIRSensorsTask(void *pvParameters) {
  while (1) {
    // Read IR sensors
    irValues[0] = digitalRead(IR_1);
    irValues[1] = digitalRead(IR_2);
    irValues[2] = digitalRead(IR_3);
    irValues[3] = digitalRead(IR_4);
    irValues[4] = digitalRead(IR_5);
    irValues[5] = digitalRead(IR_6);
    irValues[6] = digitalRead(IR_7);
    irValues[7] = digitalRead(IR_8);

    // Process the sensor readings
    // ...

    // Delay before next reading
    vTaskDelay(pdMS_TO_TICKS(10)); // 100ms delay
  }
}

void printIRArrayTask(void *pvParameters) {
  while (1) {
    // Print IR array values
    for (int i = 0; i < 8; i++) {
      Serial.print("IR ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(irValues[i]);
    }

    // Delay before next printing
    vTaskDelay(pdMS_TO_TICKS(400)); // 1 second delay
  }
}

void setup() {
  // Initialize IR sensors
  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);
  pinMode(IR_3, INPUT);
  pinMode(IR_4, INPUT);
  pinMode(IR_5, INPUT);
  pinMode(IR_6, INPUT);
  pinMode(IR_7, INPUT);
  pinMode(IR_8, INPUT);

  // Create the task for reading IR sensors
  xTaskCreate(readIRSensorsTask, "ReadIRSensorsTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  // Create the task for printing IR array values
  xTaskCreate(printIRArrayTask, "PrintIRArrayTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();
}

void loop() {
  // Empty loop, tasks are executed by the scheduler
}