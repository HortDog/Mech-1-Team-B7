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
#define IR_1 21
#define IR_2 20
#define IR_3 19
#define IR_4 18
#define IR_5 15
#define IR_6 14
#define IR_7 16
#define IR_8 17
#define IR_ON 0

// Button pins
#define KEY_2 4
#define KEY_1 5

bool IDLE_STATE;

// function prototypes
void CHECK_KEYS();
void LED_IDLE_BLIK();
void IDLE_TEST();
int LINE();
void MOTOR_CONTROL(int line_vector, int speed, int direction);

void setup() {
  // set the IDLE_STATE to false
  IDLE_STATE = true;

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
  CHECK_KEYS();
  if (IDLE_STATE) {
    // Run the IDLE function
    // runn the LED_IDLE_BLIK function will delay the loop for 2 second
    LED_IDLE_BLIK();
  } else {
    digitalWrite(ONBOAD_LED_PIN, HIGH);
    // Line follow
    int line_vector = LINE();
    MOTOR_CONTROL(line_vector, 50, 1);
  }
}


// put function definitions here:

void MOTOR_CONTROL(int line_vector, int speed, int direction) {
  // calculate the motor speed based on the line vector
  float M1 = 2.14068 * line_vector + speed;
  float M2 = 0.467141 * line_vector + speed;
  // clamp the speed value between 0 and 255
  int motor_speed_1 = max(0, min(M1, 255));
  int motor_speed_2 = max(0, min(M2, 255));
  // set the motor direction

  // set the motor speed
  analogWrite(MOTOR_PWM_PIN_1, motor_speed_1);
  analogWrite(MOTOR_PWM_PIN_2, motor_speed_2);
}

// This function will read the IR array and return a float value that represents the direction of the line
int LINE() {
  // read the IR array
  int ir_1 = digitalRead(IR_1);
  int ir_2 = digitalRead(IR_2);
  int ir_3 = digitalRead(IR_3);
  int ir_4 = digitalRead(IR_4);
  int ir_5 = digitalRead(IR_5);
  int ir_6 = digitalRead(IR_6);
  int ir_7 = digitalRead(IR_7);
  int ir_8 = digitalRead(IR_8);

  // calculate the line vector (range: -4 to 4, -4 = hard left, 4 = hard right, 0 = center)
  int line_vector = (ir_1 * 4) + (ir_2 * 3) + (ir_3 * 2) + (ir_4 * 1) + (ir_5 * -1) + (ir_6 * -2) + (ir_7 * -3) + (ir_8 * -4);

  return line_vector;
}

// This function will check if the buttons are pressed and update the IDLE_STATE variable
void CHECK_KEYS() {
  // read the state of the buttons
  int key_1 = digitalRead(KEY_1);
  int key_2 = digitalRead(KEY_2);

  // check if the buttons are pressed
  if (key_1 == LOW) {
    IDLE_STATE = true;
  } else {
    if (key_2 == LOW) {
      IDLE_STATE = false;
    }
  }
}

// This function will run the IDLE state and Led pattern
void LED_IDLE_BLIK() {
  // stop the motors
  analogWrite(MOTOR_PWM_PIN_1, 0);
  analogWrite(MOTOR_PWM_PIN_2, 0);
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
}