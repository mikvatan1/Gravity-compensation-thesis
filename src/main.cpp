#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <AS5600.h> 
#include "PIDController.h"
#include <Adafruit_NeoPixel.h>

#define LOAD_CELL A0

#define R_PWM 5 // D5
#define L_PWM 3 // D3
#define R_EN 8 // D8
#define L_EN 7 // D7
#define LOAD_CELL A0 // force sensor pin
#define AS5600_RAW_TO_DEGREES (360.0 / 4096.0)

#define LED_PIN 4           // Data pin for WS2812B (connect to DIN of the strip)
#define NUM_LEDS 144        // Number of LEDs on the strip (adjust to your strip)
#define LED_BRIGHTNESS 5    // Brightness of the strip


// Rotation tracking
bool firstReading = true;
int previousPosition = 0;
long totalRotation = 0;
float rotationCounter = 0.0;

// PID reset delay variables
static unsigned long timeInDeadband = 0;
static bool inDeadband = false;
const unsigned long RESET_DELAY_MS = 2000; // Wait 2 seconds before resetting PID

static bool skipADC = false;
static float lastForce = 0.0;
static float lastATarget = 0.0;
static float lastDetectedLoad = 0.0;
static bool firstADCRead = true;

// Force filtering variables
static float filteredForce = 0.0;
static bool firstForceRead = true;
const float FORCE_FILTER_ALPHA = 0.05; // Lower = more filtering, 0.1-0.3 is good range

// Position filtering variables
float filteredRotation = 0.0;
bool firstPosRead = true;
const float POS_FILTER_ALPHA = 0.05;

bool running = false;
bool lastMotorState = false; // Track motor state for LED updates
int currentMotorDirection = 0; // Track motor direction: 1=R_PWM(CW), -1=L_PWM(CCW), 0=stopped
bool ledUpdatePending = false; // Flag for pending LED update
uint8_t pendingR = 0, pendingG = 0, pendingB = 0; // Pending LED colors


Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

PIDController pid(4.0, 1.8, 0.80); 

// a_max = 150mm a_min 95mm
// error a_max = 55mm
// pwm range: 100-200
//
// therefore Kp*e_max should be 200 at start
// Kp = 3, e_max is 55mm

AS5600 as5600;

float k_spring = 1.97; // [N/mm]
float num_springs = 2; // Number of springs used
float spoed = 2.0; // [mm] per rotation
float a_start = 80.0; // [mm] Start value, only set at startup
float a_target = 0; // [mm] Starting value of a
float angle = 0.0; 
float own_weight = 2.0; // [kg]
float own_force = own_weight * 9.81*6; // [N] at intersection

const float ADC_TO_FORCE = ((5.0 / 1023.0) * 29.361 * 6); // Combined: voltage conversion * calibration * total force multiplier
const float ADC_TO_LOAD = ((5.0 / 1023.0) * 29.361);
const float FORCE_TO_TARGET = 1.0 / (k_spring * num_springs); // 1/(1.97*2) = 0.253807
const float ROTATION_MULTIPLIER = 1.0 / 4096.0; // Pre-calculated 1/4096 



unsigned long lastPrintTime = 0;
unsigned long startMillis = 0;
bool returnToStart = false; // Flag to trigger return to a_start
bool finished = false; // Flag to stop the loop after returning




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


//// setup ////


void setup() {
  // FIRST PRIORITY: Disable motor driver immediately to prevent startup movement
  // Set pins as outputs and immediately disable them
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, LOW);    // Disable right motor driver
  digitalWrite(L_EN, LOW);    // Disable left motor driver
  
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  analogWrite(R_PWM, 0);      // Ensure PWM is 0
  analogWrite(L_PWM, 0);      // Ensure PWM is 0
  
  // Small delay to ensure motor drivers are fully disabled
  delay(100);
  
  Serial.begin(115200);
  Wire.begin();
  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);


  // Led strip initialization
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.show();
  
  // Initialize timing variables
  startMillis = millis();
  lastPrintTime = startMillis - 1000; // Set lastPrintTime so first print happens immediately

  pid.reset();
}






//// Loop ////




void loop() {

  unsigned long loopStart = millis(); // Changed to millis() for consistency


  // After 10 seconds, trigger return to start
  if (!returnToStart && !finished && (millis() - startMillis > 10000)) {
    Serial.println("Returning to start position...");
    returnToStart = true;
  }

  // If finished, stop everything and halt
  if (finished) {
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    Serial.println("END");
    while (true) {
      //stop the loop
    }
  }

  // Update LEDs if a change was requested
  if (ledUpdatePending) {
    setMotorStatusLEDs(pendingR, pendingG, pendingB);
    ledUpdatePending = false;
  }

    
// --- Read position and calculate rotations ---
int currentPosition = as5600.rawAngle();

// Invert the angle to match your expected rotation direction
currentPosition = 4095 - currentPosition;

if (firstReading) {
  previousPosition = currentPosition;
  firstReading = false;
} else {
  int diff = currentPosition - previousPosition;
  if (diff > 2048) diff -= 4096;
  else if (diff < -2048) diff += 4096;

  totalRotation += diff;
  rotationCounter = totalRotation / 4096.0; // raw, unfiltered rotations
  previousPosition = currentPosition;

  // --- Apply low-pass filter (EMA) on rotation ---
  if (firstPosRead) {
    filteredRotation = rotationCounter;
    firstPosRead = false;
  } else {
    filteredRotation = (POS_FILTER_ALPHA * rotationCounter) +
                       ((1.0 - POS_FILTER_ALPHA) * filteredRotation);
  }
}

  
  // Read force with ADC optimization and filtering 
  float force, a_target, detectedLoad;  
  if (!skipADC || firstADCRead) { 
    int rawADC = analogRead(LOAD_CELL); 
    detectedLoad = rawADC * ADC_TO_LOAD; // top load [N]
    float rawForce = (rawADC * ADC_TO_FORCE) + own_force; // (raw load cell value + own weight) * conversion = total force at intersection
    
    // Apply exponential moving average filter to reduce noise
    if (firstForceRead) {
      filteredForce = rawForce; // Initialize with first reading
      firstForceRead = false;
    } else {
      filteredForce = (FORCE_FILTER_ALPHA * rawForce) + ((1.0 - FORCE_FILTER_ALPHA) * filteredForce);
    }
    
    force = filteredForce; // total force at intersection (top load and own weight at intersection)
    a_target = force * FORCE_TO_TARGET; // a = F/k
    lastForce = force;
    lastATarget = a_target;
    lastDetectedLoad = detectedLoad;
    firstADCRead = false;
  } else {
    force = lastForce;
    a_target = lastATarget;
    detectedLoad = lastDetectedLoad;
  }
  skipADC = !skipADC; // Toggle for next loop

  

  float a_actual = a_start + (filteredRotation * spoed);
  float error_a = a_target - a_actual;

  // If returning to start, override a_target
  if (returnToStart) {
    error_a = a_start - a_actual;
    a_target = a_start;
  }



  // If returning to start and within 2 mm, finish
  if (returnToStart && fabs(a_actual - a_start) < 2.0) {
    finished = true;
  }

  if (fabs(error_a) > 2) {
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    // Update LEDs only on state change - motor turning (RED)
    if (!lastMotorState) {
      requestMotorStatusLEDs(255, 0, 0); // Request red LEDs (non-blocking)
      lastMotorState = true;
    }

    float output = pid.compute(a_target, a_actual); // PID calculation 
    int pwm = constrain(abs(output), 100, 200); // Reduced PWM range for smoother operation

    if (error_a > 2) {  
      analogWrite(R_PWM, pwm); // R_PWM = clockwise (a_actual goes up)
      analogWrite(L_PWM, 0);
    } else if (error_a < -2) { 
      analogWrite(R_PWM, 0);
      analogWrite(L_PWM, pwm); // L_PWM = counter-clockwise (a_actual goes down)
    }

  } else {
    // System is in deadband (error < 5mm)

    // Update LEDs only on state change - motor stopped (GREEN)
    if (lastMotorState) {
      requestMotorStatusLEDs(0, 255, 0); // Request green LEDs (non-blocking)
      lastMotorState = false;
    }

    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);

    // Delayed PID reset logic
    if (!inDeadband) {
      // Just entered deadband - start the timer
      timeInDeadband = millis();
      inDeadband = true;
    } else {
      // Check if we've been in deadband long enough
      if (millis() - timeInDeadband > RESET_DELAY_MS) {
        pid.reset(); // Reset after delay
        timeInDeadband = millis(); // Reset timer to prevent repeated resets
      }
    }
  }

  // Reset deadband tracking if we exit the deadband
  if (fabs(error_a) > 5 && inDeadband) {
    inDeadband = false;
  }

  // Print every 100 milliseconds
  if (millis() - lastPrintTime > 100) {
    // Handle pending LED updates here (outside critical loop timing)
    if (ledUpdatePending) {
      setMotorStatusLEDs(pendingR, pendingG, pendingB);
      ledUpdatePending = false;
    }

    
    float timestamp = (millis() - startMillis) / 1000.0;
    float control = pid.getOutput();
    float p = pid.getPTerm();
    float i = pid.getITerm();
    float d = pid.getDTerm();
    float position = a_actual;
    float target = a_target;

    // Optimized serial output - fewer function calls
    Serial.print(timestamp, 1); Serial.print(",");
    Serial.print(error_a, 1); Serial.print(",");
    Serial.print(control, 1); Serial.print(",");
    Serial.print(position, 1); Serial.print(",");
    Serial.print(target, 1); Serial.print(",");
    Serial.print(detectedLoad, 1); Serial.print(",");
    Serial.print(p, 1); Serial.print(",");
    Serial.print(i, 1); Serial.print(",");
    Serial.println(d, 1);

    lastPrintTime = millis();
    
    unsigned long loopDuration = lastPrintTime - loopStart;

    Serial.print("a_target:");
    Serial.print(a_target);
    Serial.print("a_actual:");
    Serial.print(a_actual);
    Serial.print("Detected_load:");
    Serial.print(detectedLoad/9.81);
    Serial.print("Looptime:");
    Serial.print(loopDuration);
    Serial.println(" ms");

    
    
  }

}