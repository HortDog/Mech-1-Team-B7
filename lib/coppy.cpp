#include <Arduino.h>

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

typedef enum {
  IDLE,
  STRAIGHT_LINE,
  LINE_FOLLOW,
  FULL_STOP
} state_t;

state_t state = IDLE;

// function prototypes
void BREAK_CHECK();
void CHECK_KEYS();
void LED_IDLE_BLIK();
void IDLE_TEST();
int LINE();
void MOTOR_CONTROL(int line_vector, int speed, int direction);

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

  // set up buttons (LOW = pressed, HIGH = not pressed)
  pinMode(KEY_1, INPUT_PULLUP);
  pinMode(KEY_2, INPUT_PULLUP);

  // output to pin 10 and 9, pwm %0 duty cycle, stop motors, set direction to low (FIND OUT WHICH DIRECTION IS LOW AND HIGH FOR THE MOTOR)
  // Motor 1 - Left motor, LOW = backward, HIGH = forward
  pinMode(MOTOR_DIR_PIN_1, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN_1, HIGH);
  pinMode(MOTOR_PWM_PIN_1, OUTPUT);
  analogWrite(MOTOR_PWM_PIN_1, 0);

  // Motor 2 - Right motor, LOW = forward, HIGH = backward
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
  // turn on IR array (LOW = OFF, HIGH = ON)
  pinMode(IR_ON, OUTPUT);
  digitalWrite(IR_ON, HIGH);

  // set up serial communication
  Serial.begin(9600);
}

void loop() {
  CHECK_KEYS();
  switch (state) {
    case IDLE:
      LED_IDLE_BLIK();
      break;
    case STRAIGHT_LINE:
      digitalWrite(LED_1, HIGH);
      digitalWrite(LED_2, LOW);
      digitalWrite(LED_3, HIGH);
      MOTOR_CONTROL(0, 50, 1);
      //delay(2500); // 3.5 second delay
      //state = IDLE;
      break;
    case LINE_FOLLOW:
      int ir_values[8];
      ir_values[0] = digitalRead(IR_1);
      ir_values[1] = digitalRead(IR_2);
      ir_values[2] = digitalRead(IR_3);
      ir_values[3] = digitalRead(IR_4);
      ir_values[4] = digitalRead(IR_5);
      ir_values[5] = digitalRead(IR_6);
      ir_values[6] = digitalRead(IR_7);
      ir_values[7] = digitalRead(IR_8);

      if ((ir_values[0] == 1 ) && (ir_values[1] == 1) && (ir_values[2] == 1) && (ir_values[3] == 1) && (ir_values[4] == 1) && (ir_values[5] == 1) && (ir_values[6] == 1) && (ir_values[7] == 1)){
        digitalWrite(LED_1, HIGH);
        digitalWrite(LED_2, HIGH);
        digitalWrite(LED_3, HIGH);
        MOTOR_CONTROL(0, 0, 1);
      }
      else {
        digitalWrite(LED_1, LOW);
        digitalWrite(LED_2, HIGH);
        digitalWrite(LED_3, HIGH);
        int line_vector = LINE();
        //Serial.println(line_vector);
        MOTOR_CONTROL(line_vector, 80, 1);
      }
      break;
    default:
      break;
  }
}

// put function definitions here:

void MOTOR_CONTROL(int line_vector, int speed, int direction) {
  // calculate the motor speed based on the line vector
  float M1 = 10 * line_vector + speed;
  float M2 = -10 * line_vector + speed;
  // clamp the speed value between 0 and 255
  int motor_speed_1 = max(0, min(M1, 255));
  int motor_speed_2 = max(0, min(M2, 255));
  //Serial.print("Motor 1: ");
  //Serial.print(motor_speed_1);
  //Serial.print(" Motor 2: ");
  //Serial.println(motor_speed_2);
  // set the motor direction

  // set the motor speed
  analogWrite(MOTOR_PWM_PIN_1, motor_speed_1);
  analogWrite(MOTOR_PWM_PIN_2, motor_speed_2);
}

// This function will read the IR array and return a float value that represents the direction of the line
int LINE() {
  int ir_values[8];
  ir_values[0] = digitalRead(IR_1);
  ir_values[1] = digitalRead(IR_2);
  ir_values[2] = digitalRead(IR_3);
  ir_values[3] = digitalRead(IR_4);
  ir_values[4] = digitalRead(IR_5);
  ir_values[5] = digitalRead(IR_6);
  ir_values[6] = digitalRead(IR_7);
  ir_values[7] = digitalRead(IR_8);

  // calculate the line vector
  int line_vector = ir_values[0] * -4 + ir_values[1] * -3 + ir_values[2] * -2 + ir_values[3] * -1 + ir_values[4] * 1 + ir_values[5] * 2 + ir_values[6] * 3 + ir_values[7] * 4;

  return line_vector;
}

void BREAK_CHECK() {
  int ir_values[8];
  ir_values[0] = digitalRead(IR_1);
  ir_values[1] = digitalRead(IR_2);
  ir_values[2] = digitalRead(IR_3);
  ir_values[3] = digitalRead(IR_4);
  ir_values[4] = digitalRead(IR_5);
  ir_values[5] = digitalRead(IR_6);
  ir_values[6] = digitalRead(IR_7);
  ir_values[7] = digitalRead(IR_8);

  if ((ir_values[0] == 0 ) && (ir_values[1] == 0) && (ir_values[2] == 0) && (ir_values[3] == 0) && (ir_values[4] == 0) && (ir_values[5] == 0) && (ir_values[6] == 0) && (ir_values[7] == 0)){
    state = FULL_STOP;
  }
  else {
    state = LINE_FOLLOW;
  }
}
// This function will check if the buttons are pressed and update the IDLE_STATE variable
void CHECK_KEYS() {
  if ((state != IDLE) && (state != STRAIGHT_LINE) && (digitalRead(KEY_2) == LOW)) {
    state = IDLE;
  }
  else if ((state == IDLE) && (digitalRead(KEY_2) == LOW)) {
    state = STRAIGHT_LINE;
    Serial.println("STRAIGHT LINE");
  }
  if ((state == IDLE) && (digitalRead(KEY_1) == LOW)) {
    state = LINE_FOLLOW;
  }
}

// This function will run the IDLE state and Led pattern
void LED_IDLE_BLIK() {
  // stop the motors
  analogWrite(MOTOR_PWM_PIN_1, 0);
  analogWrite(MOTOR_PWM_PIN_2, 0);
  // turn off the LEDs
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  digitalWrite(LED_3, LOW);
  // blink the on-board LED in a specific pattern
  for (int i = 0; i < 2; i++) {
    digitalWrite(ONBOAD_LED_PIN, HIGH);
    delay(200); // 200ms delay for fast blink
    digitalWrite(ONBOAD_LED_PIN, LOW);
    delay(500); // 500ms delay for slow blink
  }

  // run the IDLE_TEST function
  // add test print statements to the IDLE_TEST function, will print every 2 seconds
  IDLE_TEST();

  for (int i = 0; i < 2; i++) {
    digitalWrite(ONBOAD_LED_PIN, HIGH);
    delay(100); // 200ms delay for fast blink
    digitalWrite(ONBOAD_LED_PIN, LOW);
    delay(200); // 800ms delay for slow blink
  }
}

void IDLE_TEST() {
  // print the test message
  Serial.println("IDLE TEST");
  int value = analogRead(IR_1);
  Serial.println(value);
}