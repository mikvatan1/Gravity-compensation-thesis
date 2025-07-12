#include <Arduino.h>

// Simple Arduino Nano motor test
// Turns the motor on for 2 seconds, then stops

const int PWM_L = 5; // Motor driver LPWM (D5)
const int PWM_R = 3; // Motor driver RPWM (D3)

void setup() {
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
}

void loop() {
  // Turn motor forward at full speed
  analogWrite(PWM_L, 255);
  analogWrite(PWM_R, 0);
  delay(2000); // Run for 2 seconds

  // Stop motor
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
  while (true) {} // Stop loop
}
