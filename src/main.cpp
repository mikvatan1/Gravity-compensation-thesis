#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <AS5600.h> 
#include "PIDController.h"
#include <Adafruit_NeoPixel.h>

#define R_PWM 5 // D5
#define L_PWM 3 // D3
#define R_EN 8 // D8
#define L_EN 7 // D7
#define LOAD_CELL A0 // force sensor pin
#define AS5600_RAW_TO_DEGREES (360.0 / 4096.0)

#define LED_PIN 4           // Data pin for WS2812B (connect to DIN of the strip)
#define NUM_LEDS 144        // Number of LEDs on the strip (adjust to your strip)
#define LED_BRIGHTNESS 5    // Brightness of the strip 

float k_spring = 1.97; // [N/mm]
float num_springs = 2; // Number of springs used
float spoed = 2.0; // [mm] per rotation
float a_start = 0; // [mm] Manually set start value of a, corresponds to the actual start position of the device
float a_target = 0; // [mm] Starting value of a
float angle = 0.0; 

// Pre-calculated constants for ultra-fast math
const float ADC_TO_FORCE = (5.0 / 1023.0) * 29.361 * 6; // Combined: voltage conversion * calibration * total force multiplier
const float FORCE_TO_TARGET = 1.0 / (k_spring * num_springs); // 1/(1.97*2) = 0.253807
const float ROTATION_MULTIPLIER = 1.0 / 4096.0; // Pre-calculated 1/4096 

// Rotation tracking variables
bool firstReading = true;
int previousPosition = 0;
int currentPosition = 0;
long totalRotation = 0;
float totalAbsRotation = 0.0;
float rotationCounter = 0.0;
float targetRotations = 0;

bool running = false;
bool lastMotorState = false; // Track motor state 


Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

AS5600 as5600; // Create an instance of the AS5600 class to interact with the sensor 

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


// Set all LEDs on the strip to the same color
void setStripColor(uint8_t r, uint8_t g, uint8_t b) {
  strip.clear(); // Clear the strip first 
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b)); // Set LED i to the desired color
  }
  strip.show(); 
}

unsigned long startMillis = 0;
unsigned long lastPrintTime = 0; 
unsigned long loopStartTime = 0; 
unsigned long maxLoopTime = 0; 
unsigned long minLoopTime = 999999; 

void setup() {
  Serial.begin(9600); // Baud rate
  Wire.begin(); // Initialize I2C 
  Wire.setClock(400000); // Reliable I2C speed for AS5600 (400kHz)
  as5600.begin(4);  // 4 = number of fast I2C transmissions
  as5600.setDirection(AS5600_CLOCK_WISE); // Set rotation direction (now matches physical direction)
  
  // Ultra-fast ADC settings (prescaler 32 = 500kHz)
  ADCSRA &= ~(bit(ADPS0) | bit(ADPS1) | bit(ADPS2)); // Clear prescaler bits
  ADCSRA |= bit(ADPS2) | bit(ADPS0);                  // Set prescaler to 32 (500kHz)
  
  // Motor driver pins as output
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  // Led strip initialization
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.show();

  Serial.println("Ultra-fast mode enabled");
  startMillis = millis();
}

void loop() {
  loopStartTime = micros(); 
  
  // Stop after 20 seconds
  if (millis() - startMillis > 20000) {
    Serial.println("Stopping after 20 seconds...");
    setStripColor(255, 255, 0); // Optional: set LED yellow to indicate stop
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    while (true) {
      // Stop loop
    }

  }
  currentPosition = as5600.rawAngle();  // 0-4095
  
  // Original rotation tracking logic that worked
  if (firstReading) {
    previousPosition = currentPosition;
    firstReading = false;
  } else {
    // Robust calculation: always take the smallest difference, even with multiple turns per sample
    int diff = (currentPosition - previousPosition + 4096) % 4096;
    if (diff > 2048) diff -= 4096;
    totalRotation += diff;
    totalAbsRotation += abs(diff) / 4096.0;  
    rotationCounter = (float)totalRotation / 4096.0;
    previousPosition = currentPosition;
  }

  // Ultra-fast math - single calculation chain
  int rawADC = analogRead(LOAD_CELL); 
  float force = rawADC * ADC_TO_FORCE; // Single multiplication instead of 4 operations
  float a_target = force * FORCE_TO_TARGET; // Single multiplication instead of division
  float a_actual = a_start + (rotationCounter * spoed);
  float error_a = a_target - a_actual;

  if (fabs(error_a) > 5) {

    // Enable motor driver
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    // Update LEDs only on state change
    if (!lastMotorState) {
      setStripColor(255, 0, 0); // Led strip: red
      lastMotorState = true;
    }

    float output = pid.compute(a_target, a_actual); // PID calculation 
    int pwm = constrain(abs(output), 30, 75); // PWM constraints

    if (error_a > 5) { 
      analogWrite(R_PWM, 0); // R_PWM corresponds to clockwise in reality
      analogWrite(L_PWM, pwm);
    } else if (error_a < -5) { // 
      analogWrite(R_PWM, pwm);
      analogWrite(L_PWM, 0); // L_PWM = counter-clockwise
    }

  } else { 
    Serial.print("Target reached!");
    Serial.println();
    
    if (lastMotorState) {
      setStripColor(0, 255, 0); // Led strip: green
      lastMotorState = false;
    }

    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);

    // Don't reset rotation tracking - keep accumulating position
    // Only reset PID to avoid wind-up
    pid.reset();
  }

// Track loop time every loop
unsigned long currentLoopTime = micros() - loopStartTime;
if (currentLoopTime > maxLoopTime) maxLoopTime = currentLoopTime;
if (currentLoopTime < minLoopTime) minLoopTime = currentLoopTime;

// Serial output only every 100ms to reduce loop time
if (millis() - lastPrintTime > 100) {
  float timestamp = millis() / 1000.0;
  float control = pid.getOutput();
  float p = pid.getPTerm();
  float i = pid.getITerm();
  float d = pid.getDTerm();
  float position = a_actual;
  float target = a_target;

  Serial.print(timestamp, 2); Serial.print(",");
  Serial.print(error_a, 2); Serial.print(",");
  Serial.print(control, 2); Serial.print(",");
  Serial.print(position, 2); Serial.print(",");
  Serial.print(target, 2); Serial.print(",");
  Serial.print(p, 2); Serial.print(",");
  Serial.print(i, 2); Serial.print(",");
  Serial.println(d, 2);

  Serial.print("Force: ");
  Serial.print(force, 2);
  Serial.print(" | a_target: ");
  Serial.print(a_target, 2);
  Serial.print(" | a_actual: ");
  Serial.print(a_actual, 2);
  Serial.print(" | error_a: ");
  Serial.print(error_a, 2);
  Serial.print(" | rot_Counter: ");
  Serial.print(rotationCounter, 2);
  Serial.print(" | tot_AbsRot_: ");
  Serial.print(totalAbsRotation, 2);
  Serial.print(" | rawPos: ");
  Serial.print(currentPosition);
  Serial.print(" | totalRot: ");
  Serial.print(totalRotation);
  Serial.print(" | Min/Max/Current LoopTime (μs): ");
  Serial.print(minLoopTime); Serial.print("/");
  Serial.print(maxLoopTime); Serial.print("/");
  Serial.print(currentLoopTime);
  Serial.println();
  
  // Reset min/max after printing
  minLoopTime = 999999;
  maxLoopTime = 0;
  lastPrintTime = millis();
}

}







