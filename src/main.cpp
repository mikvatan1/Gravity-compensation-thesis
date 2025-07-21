#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <Adafruit_NeoPixel.h>

// WS2812B LED strip settings
#define LED_PIN 4           // Data pin for WS2812B (connect to DIN of the strip)
#define NUM_LEDS 50          // Start with just 50 LEDs for testing
#define LED_BRIGHTNESS 64    // Lower brightness for testing

// Create NeoPixel strip object
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Motor driver pins
#define R_PWM 5 // D5
#define L_PWM 3 // D3
#define R_EN 8 // D8
#define L_EN 7 // D7

#define LOAD_CELL A0 // force sensor pin

// Variables for load cell readings
int loadCellValue = 0;
float voltage = 0.0;
float force = 0.0;
bool motorTriggered = false;  // Flag to prevent multiple motor runs

// AS5600 rotary encoder
AS5600 as5600;
int currentPosition = 0;
float angle = 0.0;

// Function to set all LEDs on the strip to the same color (r, g, b)
void setStripColor(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(r, g, b)); // Set LED i to the desired color
    }
    strip.show(); // Send the set colors to the strip
}









void setup() {
    Serial.begin(9600);
    Wire.begin(); // Initialize I2C
    strip.begin(); // Start the strip
    strip.setBrightness(LED_BRIGHTNESS); // Set brightness (for all colors)
    strip.show(); // Clear any previous data
    
    Serial.println("Setting LED to RED...");
    setStripColor(255, 0, 0);        // Start with red (motor stopped)
    delay(2000);
    
    Serial.println("Testing LED colors...");
    Serial.println("GREEN for 2 seconds...");
    setStripColor(0, 255, 0);        // Green
    delay(2000);
    
    Serial.println("BLUE for 2 seconds...");
    setStripColor(0, 0, 255);        // Blue
    delay(2000);
    
    Serial.println("Back to RED (standby)...");
    setStripColor(255, 0, 0);        // Back to red
    
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

    Serial.println("Motor, Load Cell and AS5600 Test Started");
    Serial.println("Waiting for force > 10N to start motor...");
    
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
    
    // Convert to voltage (0-1023 maps to 0-5V)
    voltage = (loadCellValue * 5.0) / 1023.0; //ADC = 1023 → voltage = (1023 × 5) ÷ 1023 = 5V
    
    // Convert voltage directly to force using slope
    force = voltage * 29.361; 
    
    // Read AS5600 position
    currentPosition = as5600.rawAngle();  // 0-4095 (12-bit)
    angle = as5600.rawAngle() * (360.0/4096.0);  // Convert to degrees

    // Print readings with LED status
    Serial.print("Raw ADC: ");
    Serial.print(loadCellValue);
    Serial.print(" | Voltage: ");
    Serial.print(voltage, 3);
    Serial.print("V | Force: ");
    Serial.print(force, 2);
    Serial.print(" N | Position: ");
    Serial.print(currentPosition);
    Serial.print(" | Angle: ");
    Serial.print(angle, 2);
    Serial.print("° | Motor triggered: ");
    Serial.println(motorTriggered ? "YES" : "NO");
    
    // Check if force is above 10N to start motor (only once)
    if (force > 10.0 && !motorTriggered) {
        motorTriggered = true;  // Set flag to prevent multiple runs
        Serial.println("Force > 10N detected! Starting motor...");
        
        // Set LED strip to green (motor running)
        setStripColor(0, 255, 0);
        
        // Test motor sequence
        analogWrite(R_PWM, 128);  // 50% speed forward
        analogWrite(L_PWM, 0);
        delay(1000);

        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, 0);
        delay(1000);

        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, 128);  // 50% speed reverse
        delay(1000);

        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, 0);
        
        // Set LED strip to red (motor stopped)
        setStripColor(255, 0, 0);
        
        Serial.println("Motor test completed, waiting for next trigger...");
    }
    
    // Reset flag when force drops below 5N (hysteresis)
    if (force < 5.0) {
        motorTriggered = false;
    }
    
    delay(100);  // Read every 100ms
    
    // Stop after 20 seconds
    static unsigned long startTime = millis();
    if (millis() - startTime > 20000) {  // 20 seconds
        Serial.println("20 seconds elapsed - stopping measurements...");
        
        // Turn off LED strip
        setStripColor(0, 0, 0);  // Set all LEDs to black (off)
        
        while(true) {
            // Infinite loop - stops everything
            delay(1000);
        }
    }
}



