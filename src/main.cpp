#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <AS5600.h> // Include the AS5600 library for easy sensor access
#include "PIDController.h"

#define AS5600_RAW_TO_DEGREES (360.0 / 4096.0)

float k_used = 1.97; //N/mm
float num_springs = 2; // Number of springs used in the scissor lift
float spoed = 2.0; // Spoed van de schroef in mm per rotatie 
float a_start = 100; // Handmatig ingestelde startwaarde van a in mm, komt overeen met de startpositie van het apparaat
float a_target = 0; // Doelwaarde van a in mm, berekend uit kracht

AS5600 sensor; // Create an instance of the AS5600 class to interact with the sensor

// BTS7960 motor driver pinnen
const int PWM_L = 5; // PWM voor IN1 (links)
const int PWM_R = 6; // PWM voor IN2 (rechts)

// Snelheid en richting
int motorSpeed = 150; // 0-255 (voorbeeldwaarde, kan je aanpassen)
bool motorForward = true; // true = vooruit, false = achteruit
// dit nog aanpassen, nu moet je true/false typen maar dit moet automatisch gaan

// --- Pin voor krachtsensor ---
const int forceSensorPin = A0; // Pas aan indien nodig

// --- Encoder variabelen ---
float lastAngle = 0;
float totalRotations = 0;

// --- Closed-loop control variabelen ---
float targetRotations = 0;
bool running = false;

// PID controller object
PIDController pid(50.0, 0.0, 0.0); // Kp, Ki, Kd

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  Serial.println("Initializing AS5600...");
  bool ok = sensor.begin();
  if (!ok) {
    Serial.println("Failed to initialize AS5600 sensor.");
  } else {
    Serial.println("AS5600 initialized.");
  }

  // Motor driver pinnen als output
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  // ...eventuele andere initialisaties...
}

void loop() {


  // --- Krachtsensor uitlezen ---

  int rawADC = analogRead(forceSensorPin);
  float voltage = (rawADC / 1023.0) * 5.0;
  float weightValue = (voltage / 5.0) * 15.0;
  float forceValue = weightValue * 9.81;
  float totalForce = forceValue * 6; // total force at the middle of stage 1 of the scissor lift


  // --- Bepaal doelrotaties op basis van verschil tussen a_target en a_start ---

  if (!running) { //Als het systeem NIET aan het draaien is
    a_target = totalForce/(k_used*num_springs); // a (mm), F = a*k
    float delta_a = a_target - a_start;
    if (fabs(delta_a) > 1) { // Alleen starten als delta_a groter is dan 1 mm
      targetRotations = delta_a / spoed;
      totalRotations = 0;
      lastAngle = sensor.readAngle() * AS5600_RAW_TO_DEGREES;
      motorForward = (delta_a >= 0); // Vooruit als delta positief, anders achteruit
      running = true; // motor begint met draaien
    } else {
      running = false; // Geen beweging nodig want delta a is te klein, dus motor uit
    }
  }


  // --- Motor aansturen met PID ---

  if (running) {
    float error = targetRotations - totalRotations; // Bereken de error tussen doel en huidige rotaties
    float output = pid.compute(targetRotations, totalRotations); // PID berekening, totalRotations is de gemeten rotary encoder waarde
    int pwm = constrain(abs(output), 0, 255); // Beperk de PWM waarde tussen 0 en 255
    if (fabs(totalRotations) < fabs(targetRotations)) {
      if (motorForward) { // Vooruit draaien
        analogWrite(PWM_L, pwm);
        analogWrite(PWM_R, 0);
      } else { // Achteruit draaien
        analogWrite(PWM_L, 0);
        analogWrite(PWM_R, pwm);
      }
    } else { // Doel bereikt, stop de motor
      analogWrite(PWM_L, 0);
      analogWrite(PWM_R, 0);
      running = false;
      pid.reset();
    }
  } else { // Motor uit als niet aan het draaien
    analogWrite(PWM_L, 0);
    analogWrite(PWM_R, 0);
  }


  // --- Rotary encoder uitlezen NA het draaien van de motor ---

  float angle = sensor.readAngle() * AS5600_RAW_TO_DEGREES;
  float deltaAngle = angle - lastAngle;
  if (deltaAngle > 180) deltaAngle -= 360;
  if (deltaAngle < -180) deltaAngle += 360;
  totalRotations += deltaAngle / 360.0;
  lastAngle = angle; // Update lastAngle voor de volgende iteratie

  // --- Na het bereiken van het doel, update a_start ---
  if (!running) {
    a_start = a_target;
  }

  // --- Seriële output ---
  Serial.print("Kracht: "); Serial.print(totalForce, 2); Serial.print(" N, ");
  Serial.print("Doel: "); Serial.print(targetRotations, 2); Serial.print(" rot, ");
  Serial.print("Gedraaid: "); Serial.print(totalRotations, 2); Serial.print(" rot, ");
  Serial.print("Angle: "); Serial.print(angle, 2); Serial.println(" deg");

  delay(50);
}