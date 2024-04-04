#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200); // 200ms delay for fast blink
    digitalWrite(LED_BUILTIN, LOW);
    delay(500); // 800ms delay for slow blink
  }
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100); // 200ms delay for fast blink
    digitalWrite(LED_BUILTIN, LOW);
    delay(200); // 800ms delay for slow blink
  }
  
}