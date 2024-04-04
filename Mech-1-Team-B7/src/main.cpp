#include <Arduino.h>

const int ledPin = 25; // LED connected to pin 25
const int inputPin = 20; // Pin to detect high signal

int previousState = LOW; // Store the previous state of the input pin

void print_gpio(int GP_pin) {
  pinMode(GP_pin, INPUT_PULLDOWN);

  int pin_state = digitalRead(GP_pin);
  
  Serial.print("GPIO - ");
  Serial.print(inputPin);
  Serial.print(" : ");
  Serial.print(pin_state);
  Serial.print("\n");

  delay(100);
}

void setup() {
  pinMode(inputPin, INPUT);
  gpio_disable_pulls(inputPin);

  pinMode(ledPin, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(0, OUTPUT);

  digitalWrite(2, HIGH);
  digitalWrite(1, HIGH);
  digitalWrite(0, HIGH);
  //digitalWrite(ledPin, HIGH); // Turn on LED

  Serial.begin(9600);
}

void loop() {
  print_gpio(20);

  int currentState = digitalRead(inputPin); // Read current state of the input pin
  // Check if the state has changed from LOW to HIGH (rising edge)
  //if (currentState == 1) {
  //  digitalWrite(ledPin, HIGH); // Turn on LED
  //}
  //else {
    //digitalWrite(ledPin, LOW); // Turn off LED
  //}
}
