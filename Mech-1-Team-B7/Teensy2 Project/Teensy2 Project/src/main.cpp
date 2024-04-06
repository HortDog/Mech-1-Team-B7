#include <Arduino.h>

// define pin out
// LED pins
#define ONBOAD_LED_PIN 13
#define LED_1 1
#define LED_2 2
#define LED_3 3

// Motor pins
#define MOTOR_PWM_PIN_1 10
#define MOTOR_PWM_PIN_2 9
#define MOTOR_DIR_PIN_1 7
#define MOTOR_DIR_PIN_2 8

// IR Array pins
#define IR_1 21
#define IR_2 20
#define IR_3 19
#define IR_4 18
#define IR_5 15
#define IR_6 14
#define IR_7 16
#define IR_8 17
#define IR_ON 0

// function prototypes
void LED_IDLE_BLIK();
float LINE_VECTOR();
void MOTOR_CONTROL(int motor, int speed, int direction);

void setup() {
  // set up on-board LED
  pinMode(ONBOAD_LED_PIN, OUTPUT);
  digitalWrite(ONBOAD_LED_PIN, HIGH);

  // set up external LEDs (LOW = off, HIGH = on)
  pinMode(LED_1, OUTPUT);
  digitalWrite(LED_1, LOW);
  pinMode(LED_2, OUTPUT);
  digitalWrite(LED_2, LOW);
  pinMode(LED_3, OUTPUT);
  digitalWrite(LED_3, LOW);

  // output to pin 10 and 9, pwm %0 duty cycle, stop motors, set direction to low (FIND OUT WHICH DIRECTION IS LOW AND HIGH FOR THE MOTOR)
  // Motor 1
  pinMode(MOTOR_DIR_PIN_1, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN_1, LOW);
  pinMode(MOTOR_PWM_PIN_1, OUTPUT);
  analogWrite(MOTOR_PWM_PIN_1, 0);
  // Motor 2
  pinMode(MOTOR_DIR_PIN_2, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN_2, LOW);
  pinMode(MOTOR_PWM_PIN_2, OUTPUT);
  analogWrite(MOTOR_PWM_PIN_2, 0);

  // set up IR Array
  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);
  pinMode(IR_3, INPUT);
  pinMode(IR_4, INPUT);
  pinMode(IR_5, INPUT);
  pinMode(IR_6, INPUT);
  pinMode(IR_7, INPUT);
  pinMode(IR_8, INPUT);
  // turn on IR array (LOW = ON, HIGH = OFF)
  pinMode(IR_ON, OUTPUT);
  digitalWrite(IR_ON, LOW);

  // set up serial communication
  Serial.begin(9600);
}


void loop() {
  // Run the LED_IDLE_BLIK function, which will blink the on-board LED in a specific pattern (2 long blinks, 2 short blinks...)
  LED_IDLE_BLIK();
}


// put function definitions here:

// This function will read the IR array and return a float value that represents the direction of the line
float LINE_VECTOR() {
  // read the IR array
}

void LED_IDLE_BLIK() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(ONBOAD_LED_PIN, HIGH);
    delay(200); // 200ms delay for fast blink
    digitalWrite(ONBOAD_LED_PIN, LOW);
    delay(500); // 800ms delay for slow blink
  }
  for (int i = 0; i < 2; i++) {
    digitalWrite(ONBOAD_LED_PIN, HIGH);
    delay(100); // 200ms delay for fast blink
    digitalWrite(ONBOAD_LED_PIN, LOW);
    delay(200); // 800ms delay for slow blink
  }
}