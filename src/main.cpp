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


static bool skipADC = false;
static float lastForce = 0.0;
static float lastATarget = 0.0;
static bool firstADCRead = true;

bool running = false;
bool lastMotorState = false; // Track motor state for LED updates
int currentMotorDirection = 0; // Track motor direction: 1=R_PWM(CW), -1=L_PWM(CCW), 0=stopped
bool ledUpdatePending = false; // Flag for pending LED update
uint8_t pendingR = 0, pendingG = 0, pendingB = 0; // Pending LED colors


Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

PIDController pid(1.0, 0.1, 0.05); // create your PID object with tunings

AS5600 as5600;

float k_spring = 1.97; // [N/mm]
float num_springs = 2; // Number of springs used
float spoed = 2.0; // [mm] per rotation
float a_start = 0; // [mm] Manually set start value of a, corresponds to the actual start position of the device
float a_target = 0; // [mm] Starting value of a
float angle = 0.0; 

const float ADC_TO_FORCE = (5.0 / 1023.0) * 29.361 * 6; // Combined: voltage conversion * calibration * total force multiplier
const float FORCE_TO_TARGET = 1.0 / (k_spring * num_springs); // 1/(1.97*2) = 0.253807
const float ROTATION_MULTIPLIER = 1.0 / 4096.0; // Pre-calculated 1/4096 


unsigned long lastPrintTime = 0;



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





void setup() {
  Serial.begin(9600);
  Wire.begin();
  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);
  
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
}

void loop() {

  unsigned long loopStart = micros();

  // Update LEDs if a change was requested
  if (ledUpdatePending) {
    setMotorStatusLEDs(pendingR, pendingG, pendingB);
    ledUpdatePending = false;
  }
    
  // Read force
  int rawADC = analogRead(LOAD_CELL);
  // Read position and calculate rotations
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
    rotationCounter = totalRotation / 4096.0;
    previousPosition = currentPosition;
  }
  

  float force, a_target;
  if (!skipADC || firstADCRead) {
    int rawADC = analogRead(LOAD_CELL); 
    //t4 = micros();
    force = rawADC * ADC_TO_FORCE;
    a_target = force * FORCE_TO_TARGET;
    lastForce = force;
    lastATarget = a_target;
    firstADCRead = false;
  } else {
  //t4 = micros(); // No ADC read, so t4 = t3
    force = lastForce;
    a_target = lastATarget;
  }
  skipADC = !skipADC; // Toggle for next loop
  
  float a_actual = a_start + (rotationCounter * spoed);
  float error_a = a_target - a_actual;





  if (fabs(error_a) > 5) {  // Reduced deadband to 2mm for better precision

    // Enable motor driver
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    // Update LEDs only on state change - motor turning (RED)
    if (!lastMotorState) {
      requestMotorStatusLEDs(255, 0, 0); // Request red LEDs (non-blocking)
      lastMotorState = true;
    }

    float output = pid.compute(a_target, a_actual); // PID calculation 
    int pwm = constrain(abs(output), 50, 100); // Reduced PWM range for smoother operation

    if (error_a > 5) {  
      analogWrite(R_PWM, pwm); // R_PWM = clockwise (a_actual goes up)
      analogWrite(L_PWM, 0);

    } else if (error_a < -5) { 
      analogWrite(R_PWM, 0);
      analogWrite(L_PWM, pwm); // L_PWM = counter-clockwise (a_actual goes down)

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
    }


  // Print every second
  if (millis() - lastPrintTime >= 1000) {
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
    Serial.print(" | raw_angle: ");
    Serial.print(currentPosition);
    Serial.println();
    */
    float control = pid.compute(a_target, a_actual);

    // Print values space-separated (Arduino IDE Serial Plotter expects this)
    Serial.print(pid.getPTerm()); Serial.print(" ");
    Serial.print(pid.getITerm()); Serial.print(" ");
    Serial.print(pid.getDTerm()); Serial.print(" ");
    Serial.println(control);
    
    
    unsigned long loopTime = micros() - loopStart; 
    Serial.print("Loop time (µs): ");
    Serial.println(loopTime);
    
    lastPrintTime = millis();
  }
}


