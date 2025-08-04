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
long displayRotation = 0; // Separate display variable that can be reset
float rotationCounter = 0.0;
float targetRotations = 0;

// ADC optimization - skip every other reading
static bool skipADC = false;
static float lastForce = 0.0;
static float lastATarget = 0.0;
static bool firstADCRead = true;

bool running = false;
bool lastMotorState = false; // Track motor state for LED updates
bool ledUpdatePending = false; // Flag for pending LED update
uint8_t pendingR = 0, pendingG = 0, pendingB = 0; // Pending LED colors


Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

AS5600 as5600; // Create an instance of the AS5600 class to interact with the sensor 

PIDController pid(1.5, 0.8, 0.15); // sOptimized tuning for better PWM range utilization

// Optimized tuning for force control system:
// P-term: 1.5 - balanced response without extreme outputs
// I-term: 0.25 - prevent integral windup while eliminating steady-state error  
// D-term: 0.15 - good stability without noise amplification
// P-term: (Should spike immediately with force and drop when no force is applied)
// Max force: 15kg = 150 N
// Max totalForce: 150 * 6 = 900 N
// Max a_target: 900/(1.97*2) = 110 mm
// Max targetRotations: 110/2.0 = 55 rotations
// Optimal Kp: 50/55 ≈ 0.91
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

// Set every 10th LED to show motor status
void setMotorStatusLEDs(uint8_t r, uint8_t g, uint8_t b) {
  strip.clear(); // Clear the strip first
  for (int i = 9; i < NUM_LEDS; i += 10) { // Every 10th LED (10, 20, 30, etc.)
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

// Fast LED update - just set the flag, don't update immediately
void requestMotorStatusLEDs(uint8_t r, uint8_t g, uint8_t b) {
  pendingR = r;
  pendingG = g;
  pendingB = b;
  ledUpdatePending = true;
}

unsigned long startMillis = 0;
unsigned long lastPrintTime = 0; 
unsigned long loopStartTime = 0; 
unsigned long maxLoopTime = 0; 
unsigned long minLoopTime = 999999; 

void setup() {
  Serial.begin(9600); // Baud rate
  Wire.begin(); // Initialize I2C 
  Wire.setClock(800000); // 800kHz - try again with better timeout
  Wire.setTimeout(75); // More conservative timeout than before
  as5600.begin(4);  // Keep original 4 for reliable I2C transmissions
  as5600.setDirection(AS5600_CLOCK_WISE); // Set rotation direction (now matches physical direction)
  
  // Ultra-fast ADC settings (prescaler 32 = 500kHz)
  ADCSRA &= ~(bit(ADPS0) | bit(ADPS1) | bit(ADPS2)); // Clear prescaler bits
  ADCSRA |= bit(ADPS2) | bit(ADPS0);                  // Set prescaler to 32 (500kHz)
  
  // Motor driver pins as output - DISABLE FIRST to prevent startup motor movement
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  
  // Immediately disable motor to prevent startup movement
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);

  // Led strip initialization
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.show();

  Serial.println("Ultra-fast mode enabled");
  
  // Wait for AS5600 to stabilize and systems to initialize
  delay(500); // 500ms delay to prevent startup motor movement
  
  startMillis = millis();
}

void loop() {
  loopStartTime = micros(); 
  unsigned long t1, t2, t3, t4, t5, t6;
  
  // Stop after 20 seconds
  if (millis() - startMillis > 20000) {
    //Serial.println("Stopping after 20 seconds...");
    //setStripColor(255, 255, 0); // Optional: set LED yellow to indicate stop
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    while (true) {
      // Stop loop
    }

  }
  
  t1 = micros();
  currentPosition = as5600.rawAngle();  // 0-4095
  t2 = micros();
  
  // Original rotation tracking logic that worked
  if (firstReading) {
    previousPosition = currentPosition;
    firstReading = false;
  } else {
    // Ultra-optimized rotation tracking
    int diff = currentPosition - previousPosition;
    if (diff > 2048) diff -= 4096;
    else if (diff < -2048) diff += 4096;
    totalRotation += diff; // Keep accumulating for actual position control
    displayRotation += diff; // Also accumulate for display (can be reset)
    rotationCounter = totalRotation * ROTATION_MULTIPLIER; // Use total for control
    previousPosition = currentPosition;
  }
  
  t3 = micros();

  // Ultra-fast math - skip ADC reading every other loop (but always read first time)
  float force, a_target;
  if (!skipADC || firstADCRead) {
    int rawADC = analogRead(LOAD_CELL); 
    t4 = micros();
    force = rawADC * ADC_TO_FORCE;
    a_target = force * FORCE_TO_TARGET;
    lastForce = force;
    lastATarget = a_target;
    firstADCRead = false;
  } else {
    t4 = micros(); // No ADC read, so t4 = t3
    force = lastForce;
    a_target = lastATarget;
  }
  skipADC = !skipADC; // Toggle for next loop
  
  float a_actual = a_start + (rotationCounter * spoed);
  float error_a = a_target - a_actual;
  t5 = micros();

  if (fabs(error_a) > 1.5) {  // Reduced deadband to 1.5mm for better precision

    // Enable motor driver
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    // Update LEDs only on state change - motor turning (RED)
    if (!lastMotorState) {
      requestMotorStatusLEDs(255, 0, 0); // Request red LEDs (non-blocking)
      lastMotorState = true;
    }

    float output = pid.compute(a_target, a_actual); // PID calculation 
    
    // Improved adaptive PWM mapping based on diagnostic data
    int pwm;
    float abs_output = abs(output);
    if (abs_output < 5) {
      // Very small errors - minimal but sufficient torque
      pwm = map(abs_output, 0, 5, 40, 50);  // 40-50 PWM for very small errors
    } else if (abs_output < 15) {
      // Small to medium errors - precision control
      pwm = map(abs_output, 5, 15, 50, 65);  // 50-65 PWM for small-medium errors  
    } else if (abs_output < 40) {
      // Medium to large errors - higher torque
      pwm = map(abs_output, 15, 40, 65, 75);  // 65-75 PWM for medium-large errors
    } else {
      // Very large errors - maximum torque
      pwm = 80;  // Maximum PWM for very large errors
    }

    if (error_a > 1.5) {  // Updated threshold to match new deadband
      analogWrite(R_PWM, 0); // R_PWM corresponds to clockwise in reality
      analogWrite(L_PWM, pwm);
    } else if (error_a < -1.5) {  // Updated threshold to match new deadband
      analogWrite(R_PWM, pwm);
      analogWrite(L_PWM, 0); // L_PWM = counter-clockwise
    }

  } else { 
    // Update LEDs only on state change - motor stopped (GREEN)
    if (lastMotorState) {
      requestMotorStatusLEDs(0, 255, 0); // Request green LEDs (non-blocking)
      lastMotorState = false;
    }

    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);

    // Don't reset rotation tracking - keep accumulating position
    // Only reset PID to avoid wind-up
    pid.reset();
    
    // Reset display rotation for visual reference (target reached)
    displayRotation = 0;
  }
  
  t6 = micros();

// Track loop time every loop
unsigned long currentLoopTime = micros() - loopStartTime;
if (currentLoopTime > maxLoopTime) maxLoopTime = currentLoopTime;
if (currentLoopTime < minLoopTime) minLoopTime = currentLoopTime;

// Performance analysis - print timing breakdown every 2 seconds
/*
static unsigned long lastTimingPrint = 0;
if (millis() - lastTimingPrint > 2000) {
  Serial.print("TIMING BREAKDOWN (μs): ");
  Serial.print("AS5600: "); Serial.print(t2-t1);
  Serial.print(" | Rotation: "); Serial.print(t3-t2);
  Serial.print(" | ADC: "); Serial.print(t4-t3);
  Serial.print(" | Math: "); Serial.print(t5-t4);
  Serial.print(" | Control: "); Serial.print(t6-t5);
  Serial.print(" | TOTAL: "); Serial.print(minLoopTime);
  Serial.print("/"); Serial.print(maxLoopTime);
  Serial.print(" | AS5600 RAW: "); Serial.print(currentPosition);
  Serial.print(" | ROT_COUNTER: "); Serial.print(rotationCounter, 3);
  Serial.println();
  lastTimingPrint = millis();
}
*/

// Serial output only every 1000ms to reduce loop time further
if (millis() - lastPrintTime > 1000) {
  // Handle pending LED updates here (outside critical loop timing)
  if (ledUpdatePending) {
    setMotorStatusLEDs(pendingR, pendingG, pendingB);
    ledUpdatePending = false;
  }
  
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

  /*
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
  Serial.print(" | totalRot: ");
  Serial.print(displayRotation * ROTATION_MULTIPLIER, 2); // Convert to actual rotations
  Serial.print(" | Current LoopTime (μs): ");
  Serial.print(currentLoopTime);
  Serial.println();
  */

  // Reset min/max after printing
  minLoopTime = 999999;
  maxLoopTime = 0;
  lastPrintTime = millis();
}

}







