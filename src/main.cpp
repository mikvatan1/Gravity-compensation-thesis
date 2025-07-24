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
float a_start = 100; // [mm] Manually set start value of a, corresponds to the actual start position of the device
float a_target = 0; // [mm] Starting value of a

AS5600 as5600; // Create an instance of the AS5600 class to interact with the sensor

// Motor driver pins (correct from main.cpp)
#define R_PWM 5 // D5
#define L_PWM 3 // D3
#define R_EN 8 // D8
#define L_EN 7 // D7

#define LOAD_CELL A0 // force sensor pin

// Motor direction
bool motorForward = true; // true = forward, false = backward, goes automatically

// Encoder variables 
int currentPosition = 0;
int previousPosition = 0;
long totalRotation = 0;
float totalDegrees = 0.0;
float rotationCounter = 0.0;
bool firstReading = true;

// Closed-loop control variables
float targetRotations = 0;
bool running = false;
unsigned long motorStartTime = 0; // Track when motor started for timeout
bool timeoutOccurred = false; // Prevent immediate restart after timeout
unsigned long timeoutLightTime = 0; // Track when yellow light started
bool yellowLightActive = false; // Track if yellow light is currently on

// PID controller object 
// tune your Kp, Ki, and Kd so that, 
// for the largest expected error, the output is close to 255, 
// and for small errors, the output is much less.

PIDController pid(1.7, 0.0, 0.0); // Kp, Ki, Kd

//Start with Ki and Kd at 0.
//Set Kp so that:
//Kp * 150 ≈ 255
//So, Kp ≈ 255 / 150 ≈ 1.7
//Test and adjust Kp, then slowly add Ki and Kd for better performance.

// Instellingen voor de WS2812B LED-strip (correct from main.cpp)
#define LED_PIN 4           // Data pin for WS2812B (connect to DIN of the strip)
#define NUM_LEDS 144        // Number of LEDs on the strip (adjust to your strip)
#define LED_BRIGHTNESS 5    // Brightness of the strip (correct from main.cpp)

// Create a NeoPixel strip object
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Function to calculate total rotation from AS5600 - exact same as test code
void calculateRotation() {
    if (firstReading) {
        previousPosition = currentPosition;
        firstReading = false;
        return;
    }
    
    int rawDifference = currentPosition - previousPosition;
    int difference = rawDifference;
    
    // Handle boundary crossings
    if (rawDifference > 2048) {
        difference = rawDifference - 4096;  // Crossed backward through 0
    } else if (rawDifference < -2048) {
        difference = rawDifference + 4096;  // Crossed forward through 4095
    }
    
    // Accumulate total rotation
    totalRotation += difference;
    totalDegrees = (totalRotation * 360.0) / 4096.0;
    rotationCounter = (float)totalRotation / 4096.0;  // Float division for decimal places
    
    previousPosition = currentPosition;
}




void setup() {
  Serial.begin(9600); // Baud rate 9600
  Wire.begin(); // Initialize I2C

  as5600.begin(4);  // 4 = number of fast I2C transmissions
  as5600.setDirection(AS5600_CLOCK_WISE); // Set rotation direction
  
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
}

// Set all LEDs on the strip to the same color (r, g, b)
void setStripColor(uint8_t r, uint8_t g, uint8_t b) {
  strip.clear(); // Clear the strip first (like in main.cpp)
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b)); // Set LED i to the desired color
  }
  strip.show(); // Send the set colors to the strip
}

void loop() {

  // Read force sensor
  int rawADC = analogRead(LOAD_CELL);
  float voltage = (rawADC * 5.0) / 1023.0; // Convert to voltage
  float force = voltage * 29.361;  // Convert voltage directly to force (N) using slope
  float totalForce = force * 6; // Total force at the middle of stage 1 of the scissor lift

  // If motor is not running
  if (!running) { 
    a_target = totalForce/(k_spring*num_springs); // a(mm), k(N/mm), F = a*k
    float delta_a = a_target - a_start;
    
    // Don't restart immediately after timeout - require force to change significantly
    if (timeoutOccurred) {
      
      // Turn off yellow light after 2 seconds - only once
      if (yellowLightActive && millis() - timeoutLightTime > 2000) {
        setStripColor(0, 255, 0); // Back to green after 2 seconds
        yellowLightActive = false; // Mark yellow light as turned off
      }
      
      if (fabs(delta_a) < 1) { // Reset timeout flag when force is low
        timeoutOccurred = false;
        yellowLightActive = false; // Reset yellow light flag too
      }
      return; // Skip motor start logic during cooldown
    }
    
    if (fabs(delta_a) > 5) {  // Only adjust if a_delta is bigger than 5 mm
      targetRotations = delta_a / spoed;
      totalRotation = 0;
      rotationCounter = 0.0;
      currentPosition = as5600.rawAngle();
      previousPosition = currentPosition;
      firstReading = true;
      motorForward = (delta_a >= 0); // True/false , later used for rotation direction
      running = true; // Motor starts rotating
      motorStartTime = millis(); // Record start time for timeout
    } else {
      running = false; // delta_a is too small, motor off
    }
  }


  // If motor is running, adjust with PID
  if (running) {
    // Check for 5-second timeout
    if (millis() - motorStartTime > 5000) {
      // Serial.println(" | TIMEOUT: 5 seconds elapsed - stopping motor for safety!");
      analogWrite(R_PWM, 0);
      analogWrite(L_PWM, 0);
      running = false;
      pid.reset();
      timeoutOccurred = true; // Set timeout flag to prevent immediate restart
      timeoutLightTime = millis(); // Record when yellow light started
      yellowLightActive = true; // Mark that yellow light is now active
      setStripColor(255, 255, 0); // Yellow = timeout warning
      
      // Stop program after timeout
      while(true) {
        delay(1000); // Infinite loop - stops program
      }
    }
    
    setStripColor(255, 0, 0); // Led strip: red
    float output = pid.compute(targetRotations, rotationCounter); // PID calculation using rotationCounter from test code
    int pwm = constrain(abs(output), 50, 255); // Minimum PWM of 50 like in test code
    
    // DEBUG: Print PID values
    // Serial.print(" | PID output: "); Serial.print(output, 2);
    // Serial.print(" | PWM: "); Serial.print(pwm);
    // Serial.print(" | abs(totalRot): "); Serial.print(fabs(totalRotations), 3);
    // Serial.print(" | abs(targetRot): "); Serial.print(fabs(targetRotations), 3);
    // Serial.print(" | Runtime: "); Serial.print((millis() - motorStartTime) / 1000.0, 1); Serial.print("s");
    
    if (fabs(rotationCounter) < fabs(targetRotations)) {
      // Serial.print(" | Status: MOVING");
      if (motorForward) { // Motor moves forward
        analogWrite(R_PWM, pwm); // Use correct pin names from main.cpp
        analogWrite(L_PWM, 0);
      } else { // Motor moves backward
        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, pwm);
      }
    } else { // Target reached, stop the motor
      analogWrite(R_PWM, 0);
      analogWrite(L_PWM, 0);
      running = false;
      pid.reset();
      timeoutOccurred = false; // Clear timeout flag on successful completion
      
      // Stop program after successful completion
      while(true) {
        delay(1000); // Infinite loop - stops program
      }
    }
    
    // Fast sampling when motor is running - like in your working test code
    delay(5);  // 5ms delay for fast encoder reading during motor operation
  } else { // Motor off when not running
    // Only set green if not in timeout cooldown
    if (!timeoutOccurred) {
      setStripColor(0, 255, 0); // Led strip: green
    }
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    
    // Use same fast sampling even when motor is off for consistent encoder reading
    delay(5);  // Same 5ms delay as when running
  }


  // Rotary encoder reading - read every loop like in test code
  currentPosition = as5600.rawAngle();  // 0-4095 (12-bit)
  calculateRotation();

  // After reaching the goal, update a_start
  if (!running) {
    a_start = a_target;
  }

  // Serial output - DEBUG: Print all values to see what's happening
  // Serial.print("Raw ADC: "); Serial.print(rawADC);
  // Serial.print(" | Voltage: "); Serial.print(voltage, 3); Serial.print("V");
  Serial.print(" | Force: "); Serial.print(force, 2); Serial.print("N");
  // Serial.print(" | Total Force: "); Serial.print(totalForce, 2); Serial.print("N");
  // Serial.print(" | a_target: "); Serial.print(a_target, 2); Serial.print("mm");
  // Serial.print(" | delta_a: "); Serial.print(a_target - a_start, 2); Serial.print("mm");
  // Serial.print(" | Running: "); Serial.print(running ? "YES" : "NO");
  // Serial.print(" | Target rot: "); Serial.print(targetRotations, 2);
  Serial.print(" | Rotations: "); Serial.print(rotationCounter, 2);
  // Serial.print(" | Angle: "); Serial.print(angle, 2); Serial.println("°");

  // Remove the delay from here since it's now handled in the motor control section
}


