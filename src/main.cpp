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
float totalDegrees = 0.0;  // Totale rotatie in graden
float rotationCounter = 0.0;  // Aantal volledige rotaties (met decimalen)
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


void loop() {
  // Read force sensor
  int rawADC = analogRead(LOAD_CELL);
  float voltage = (rawADC * 5.0) / 1023.0; // Convert to voltage
  float force = voltage * 29.361;  // Convert voltage directly to force (N) using slope
  float totalForce = force * 6; // Total force at the middle of stage 1 of the scissor lift
  
  // calculate target a and delta_a
  a_target = totalForce/(k_spring*num_springs); // a(mm), k(N/mm), F = a*k
  float delta_a = a_target - a_start; 
  
  // Calculate target rotations from delta_a
  targetRotations = delta_a / spoed; // Convert mm to rotations

  // Read AS5600 position
  currentPosition = as5600.rawAngle();  // 0-4095 (12-bit)
  angle = currentPosition * (360.0/4096.0);  // Convert to degrees using already read value

  // Calculate total rotation
  calculateRotation();

  // Print readings with rotation info every 10 readings to avoid serial overflow
  static int printCounter = 0;
  if (printCounter % 10 == 0) {
    Serial.print("Raw ADC: ");
    Serial.print(rawADC);
    Serial.print(" | Voltage: ");
    Serial.print(voltage, 3);
    Serial.print("V | Force: ");
    Serial.print(force, 2);
    Serial.print(" N | Angle: ");
    Serial.print(angle, 1);
    Serial.print("° | Total Rotation: ");
    Serial.print(totalDegrees, 1);
    Serial.print("° | Rotations: ");
    Serial.println(rotationCounter, 2);  // Show 2 decimal places
  }
  printCounter++;
    

  // Motor control based on rotation error only
  float rotationError = targetRotations - rotationCounter;
  
  if (fabs(rotationError) > 1) {    // Only adjust if rotation error > 1
    float output = pid.compute(targetRotations, rotationCounter); // PID calculation using rotationCounter
    int pwm = constrain(abs(output), 50, 255); // Minimum PWM of 50 for motor operation

    setStripColor(255, 0, 0); // Led strip: red
    running = true;

    if (rotationError > 1) { // Need more rotations (extend)
      analogWrite(R_PWM, pwm); 
      analogWrite(L_PWM, 0);
    } else if (rotationError < -1) { // Need fewer rotations (retract to correct overshoot)
      analogWrite(R_PWM, 0);
      analogWrite(L_PWM, pwm);
    }

  } else { // Target reached, stop the motor
    setStripColor(0, 255, 0); // Led strip: green
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    if (running) {  // Only reset PID when transitioning from running to stopped
      pid.reset();
      a_start = a_target;  // Update start position when target is reached
      // Reset rotation counter to match new reference point
      totalRotation = 0;
      rotationCounter = 0.0;
      totalDegrees = 0.0;
      running = false;
    }
  }

  // Faster sampling when motor is active
  if (running) {
    delay(10);  // Fast updates when controlling motor
  } else {
    delay(20);  // Normal rate when idle
  }
  
  // Stop after 20 seconds
  static unsigned long startTime = millis();
  if (millis() - startTime > 20000) {  // 20 seconds
    Serial.println("20 seconds elapsed - stopping measurements...");
    
    // STOP MOTOR FIRST!
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    
    // Turn off LED strip (using clear like working code)
    strip.clear();
    strip.show();
    
    while(true) {
      // Infinite loop - stops everything
      delay(1000);
    }
  }

  // Serial output - DEBUG: Print all values to see what's happening
  // Serial.print("Raw ADC: "); Serial.print(rawADC);
  // Serial.print(" | Voltage: "); Serial.print(voltage, 3); Serial.print("V");
  Serial.print(" | Force: "); Serial.print(force, 2); Serial.print("N");
  // Serial.print(" | Total Force: "); Serial.print(totalForce, 2); Serial.print("N");
  Serial.print(" | a_target: "); Serial.print(a_target, 2); Serial.print("mm");
  Serial.print(" | delta_a: "); Serial.print(delta_a, 2); Serial.print("mm");
  // Serial.print(" | Running: "); Serial.print(running ? "YES" : "NO");
  Serial.print(" | Target rot: "); Serial.print(targetRotations, 2);
  Serial.print(" | Rotations: "); Serial.print(rotationCounter, 2);
  Serial.println(); // Add newline for readable output

}












