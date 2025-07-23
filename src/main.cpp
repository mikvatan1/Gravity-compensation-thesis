#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <Adafruit_NeoPixel.h>

// WS2812B LED strip settings
#define LED_PIN 4           // Data pin for WS2812B (connect to DIN of the strip)
#define NUM_LEDS 144          // Start with just 50 LEDs for testing
#define LED_BRIGHTNESS 5    // 0-255 

// Create NeoPixel strip object
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Motor driver pins
#define R_PWM 5 // D5
#define L_PWM 3 // D3
#define R_EN 8 // D8
#define L_EN 7 // D7

#define LOAD_CELL A0 // force sensor pin

// I2C pins (Arduino Nano default)
#define SDA_PIN A4  // Data line
#define SCL_PIN A5  // Clock line

// Variables for load cell readings
int loadCellValue = 0;
float voltage = 0.0;
float force = 0.0;

// AS5600 rotary encoder
AS5600 as5600;
int currentPosition = 0;
float angle = 0.0;

// Variables voor rotatie tracking
int previousPosition = 0;
long totalRotation = 0;
float totalDegrees = 0.0;  // Totale rotatie in graden
float rotationCounter = 0.0;  // Aantal volledige rotaties (met decimalen)
bool firstReading = true;

// Function to set all LEDs on the strip to the same color (r, g, b)
void setStripColor(uint8_t r, uint8_t g, uint8_t b) {
    // Clear the strip first (like in working code)
    strip.clear();
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
    Serial.begin(9600);
    Wire.begin(); // Initialize I2C
    
    Serial.println("=== CODE VERSION 3.0 - NO FLAGS ===");
    Serial.println("Setup starting...");
    
    // Initialize LED strip with the working sequence
    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.clear();
    strip.show();

    // Initialize AS5600
    as5600.begin(4);  // 4 = number of fast I2C transmissions
    as5600.setDirection(AS5600_CLOCK_WISE); // Set rotation direction
    
    // Configureer alle motor pinnen als output
    pinMode(R_PWM, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);

    // Analog pins are input by default, no pinMode needed
    
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    
    // Check if AS5600 is connected
    if (as5600.isConnected()) {
        Serial.println("AS5600 rotary encoder connected successfully!");
    } else {
        Serial.println("AS5600 not found - check I2C connections");
    }
}

void loop() {
    // Read load cell from A0
    loadCellValue = analogRead(LOAD_CELL); // raw ADC reading (0-1023)
    voltage = (loadCellValue * 5.0) / 1023.0; // Convert to voltage (0-1023 maps to 0-5V)
    force = voltage * 29.361;  // Convert voltage directly to force using slope
    
    // Read AS5600 position
    currentPosition = as5600.rawAngle();  // 0-4095 (12-bit)
    angle = as5600.rawAngle() * (360.0/4096.0);  // Convert to degrees
    
    // Calculate total rotation
    calculateRotation();

    // Print readings with rotation info every 10 readings to avoid serial overflow
    static int printCounter = 0;
    if (printCounter % 10 == 0) {
        Serial.print("Raw ADC: ");
        Serial.print(loadCellValue);
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

    // Check if force is above 5N to start motor
    if (force > 5) {
     
        setStripColor(255, 0, 0);  // Set LED strip to red (motor running)
        
        // Motor run with fast AS5600 sampling
        analogWrite(R_PWM, 50);  // 0-256
        analogWrite(L_PWM, 0);
        
        // Fast sampling during motor operation
        unsigned long motorStartTime = millis();
        while (millis() - motorStartTime < 1000) {
            currentPosition = as5600.rawAngle();
            calculateRotation();
            delay(5);  // Very fast sampling (5ms)
        }

        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, 0);
        delay(1000);

        analogWrite(R_PWM, 50);
        analogWrite(L_PWM, 0);  // 0-256
        
        // Fast sampling during second motor run
        motorStartTime = millis();
        while (millis() - motorStartTime < 1000) {
            currentPosition = as5600.rawAngle();
            calculateRotation();
            delay(5);  // Very fast sampling (5ms)
        }

        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, 0);
        
        // Set LED strip to green (motor stopped)
        setStripColor(0, 255, 0);

    }
    
    delay(20);  // Faster reading - 20ms instead of 100ms
    
    // Stop after 20 seconds
    static unsigned long startTime = millis();
    if (millis() - startTime > 20000) {  // 20 seconds
        Serial.println("20 seconds elapsed - stopping measurements...");
        
        // Turn off LED strip (using clear like working code)
        strip.clear();
        strip.show();
        
        while(true) {
            // Infinite loop - stops everything
            delay(1000);
        }
    }
}



