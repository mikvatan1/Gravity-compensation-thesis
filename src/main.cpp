#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <AS5600.h> 
#include "PIDController.h"
#include <Adafruit_NeoPixel.h>

#define AS5600_RAW_TO_DEGREES (360.0 / 4096.0)

float k_spring = 1.97; // [N/mm]
float num_springs = 2; // Number of springs used
float spoed = 2.0; // [mm] per rotation
float a_start = 0; // [mm] Manually set start value of a, corresponds to the actual start position of the device
float a_target = 0; // [mm] Starting value of a

// Motor driver pins (correct from main.cpp)
#define R_PWM 5 // D5
#define L_PWM 3 // D3
#define R_EN 8 // D8
#define L_EN 7 // D7

#define LOAD_CELL A0 // force sensor pin

AS5600 as5600; // Create an instance of the AS5600 class to interact with the sensor 
int currentPosition = 0;
float angle = 0.0; 

// Variables for rotation tracking
int previousPosition = 0;
long totalRotation = 0;
float rotationCounter = 0.0;  // Aantal volledige rotaties (met decimalen)
float totalAbsRotation = 0.0; // Sum of absolute steps, always increasing
bool firstReading = true;

// Closed-loop control variables
float targetRotations = 0;
bool running = false;



PIDController pid(2.3, 0.1, 0.0); // Kp, Ki, Kd - Optimized for 15kg max force

// P-term: (Should spike immediately with force and drop when no force is applied)
// Max force: 15kg = 150 N
// Max totalForce: 150 * 6 = 900 N
// Max a_target: 900/(1.97*2) = 227.4 mm
// Max targetRotations: 227.4/2.0 = 113.7 rotations
// Optimal Kp: 255/113.7 ≈ 2.2
// For your maximum expected ERROR, the PID output should be around 255 (max speed):

// I-term: (Should gradually build up during sustained error and flatten out when target is reached)
// Too low: System never quite reaches exact target (steady-state error remains)
// Too high: System becomes unstable, oscillates, or overshoots
// Good range to try: 0.05, 0.1, 0.2, 0.5

// D-term: (Should remain flat because the error difference is very small, and change when sudden spikes happen)
// Too low: System overshoots target, oscillates
// Too high: System becomes sluggish, amplifies noise
// Good starting values: 0.01, 0.05, 0.1


// Instellingen voor de WS2812B LED-strip (correct from main.cpp)
#define LED_PIN 4           // Data pin for WS2812B (connect to DIN of the strip)
#define NUM_LEDS 144        // Number of LEDs on the strip (adjust to your strip)
#define LED_BRIGHTNESS 5    // Brightness of the strip (correct from main.cpp)

// Create a NeoPixel strip object
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Set all LEDs on the strip to the same color (r, g, b)
void setStripColor(uint8_t r, uint8_t g, uint8_t b) {
  strip.clear(); // Clear the strip first (like in main.cpp)
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b)); // Set LED i to the desired color
  }
  strip.show(); // Send the set colors to the strip
}



// Function to calculate total rotation from AS5600
// void calculateRotation() {
//   if (firstReading) {
//        previousPosition = currentPosition;
//        firstReading = false;
//        return;
//   }
//    
//    // Robust calculation: always take the smallest difference, even with multiple turns per sample
//    int diff = (currentPosition - previousPosition + 4096) % 4096;
//    if (diff > 2048) diff -= 4096;
//    totalRotation += diff;
//    totalAbsRotation += fabs(diff) / 4096.0; // Add absolute value in rotations
//    totalDegrees = (totalRotation * 360.0) / 4096.0;
//    rotationCounter = (float)totalRotation / 4096.0;  // Float division for decimal places
//    previousPosition = currentPosition;
//}

unsigned long startMillis = 0;

void setup() {
  Serial.begin(9600); // Baud rate 9600
  Wire.begin(); // Initialize I2C

  as5600.begin(4);  // 4 = number of fast I2C transmissions
  as5600.setDirection(AS5600_CLOCK_WISE); // Set rotation direction (now matches physical direction)
  
  // Check if AS5600 is connected
  if (as5600.isConnected()) {
    Serial.println("AS5600 rotary encoder connected successfully!");
  } else {
    Serial.println("AS5600 not found - check I2C connections");
  }

  // Motor driver pins as output (correct pins from main.cpp)
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  
  // Enable motor driver
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  // Led strip initialization
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS); // Set brightness
  strip.show(); // Turn off all LEDs at startup
  startMillis = millis(); // Record start time
}


void loop() {

  // Stop after 20 seconds
  if (millis() - startMillis > 20000) {
    setStripColor(255, 255, 0); // Optional: set LED yellow to indicate stop
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    while (true) {
      // Halt execution
    }
  }

  currentPosition = as5600.rawAngle();  // 0-4095 (12-bit)


  if (firstReading) {
    previousPosition = currentPosition;
    firstReading = false;
  } else {
    int diff = (currentPosition - previousPosition + 4096) % 4096;
    if (diff > 2048) diff -= 4096;
    totalRotation += diff;
    totalAbsRotation += fabs(diff) / 4096.0; 
    rotationCounter = (float)totalRotation / 4096.0;
    previousPosition = currentPosition;
  }

  
  
  // Read the force sensor (Load Cell)
  int rawADC = analogRead(LOAD_CELL);
  float voltage = (rawADC * 5.0) / 1023.0; // Convert to voltage
  float force = voltage * 29.361;  // force (N) 
  float totalForce = force * 6; // Total force 
  
  float a_target = totalForce/(k_spring*num_springs); // a(mm), k(N/mm), F = a*k
  float a_actual = a_start + (rotationCounter * spoed); // Actual position of a
  float delta_a = a_target - a_actual;

  targetRotations = delta_a / spoed; // Convert mm to rotations

  float rotationError = targetRotations - rotationCounter;

  if (fabs(rotationError) > 2) {   // Absolute value of 1
    // Enable motor driver so it can move
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    float output = pid.compute(targetRotations, rotationCounter); // PID calculation 
    int pwm = constrain(abs(output), 50, 255); // PWM constraints

    setStripColor(255, 0, 0); // Led strip: red

    if (rotationError > 2) { // Need more rotations (extend)
      analogWrite(R_PWM, 0); // R_PWM = clockwise 
      analogWrite(L_PWM, pwm);
    } else if (rotationError < -2) { // Retract to correct overshoot
      analogWrite(R_PWM, pwm);
      analogWrite(L_PWM, 0); // L_PWM = counter-clockwise
    }

  } else { // Target reached
    Serial.print("Target reached!");
    setStripColor(0, 255, 0); // Led strip: green
    
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);

    // Reset rotation variables
    totalRotation = 0;
    rotationCounter = 0.0;
    totalAbsRotation = 0.0;
    previousPosition = currentPosition; // Fix: reset encoder reference

    pid.reset();
    a_start = a_actual;  // Update start position
    firstReading = true; // Reset first reading
  }

delay(5);

Serial.print("Force: ");
Serial.print(force, 2);
Serial.print(" | a_target: ");
Serial.print(a_target, 2);
Serial.print(" | a_actual: ");
Serial.print(a_actual, 2);
Serial.print(" | delta_a: ");
Serial.print(delta_a, 2);
Serial.print(" | totalAbsRotation: ");
Serial.print(totalAbsRotation, 2);
Serial.print(" | rotationError: ");
Serial.print(rotationError, 2);
Serial.print(" | rotationCounter: ");
Serial.println(rotationCounter, 4);


}







