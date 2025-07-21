#include <Arduino.h>

// Motor pins
#define R_PWM 5 // D5
#define L_PWM 3 // D3
#define R_EN 8 // D8
#define L_EN 7 // D7

// Load cell signal conditioner pin
#define LOAD_CELL A0

// Variables for load cell readings
int loadCellValue = 0;
float voltage = 0.0;
float force = 0.0;
bool motorTriggered = false;  // Flag to prevent multiple motor runs

void setup() {
    Serial.begin(9600);
    
    // Configureer alle motor pinnen als output
    pinMode(R_PWM, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);

    // Analog pins are input by default, no pinMode needed
    
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    Serial.println("Motor and Load Cell Test Started");
    Serial.println("Waiting for force > 10N to start motor...");
}

void loop() {
    // Read load cell from A0
    loadCellValue = analogRead(LOAD_CELL); // raw ADC reading (0-1023)
    
    // Convert to voltage (0-1023 maps to 0-5V)
    voltage = (loadCellValue * 5.0) / 1023.0; //ADC = 1023 → voltage = (1023 × 5) ÷ 1023 = 5V
    
    // Convert voltage directly to force using slope
    force = voltage * 29.361; 
    
    // Print readings
    Serial.print("Raw ADC: ");
    Serial.print(loadCellValue);
    Serial.print(" | Voltage: ");
    Serial.print(voltage, 3);
    Serial.print("V | Force: ");
    Serial.print(force, 2);
    Serial.println(" Newtons");
    
    // Check if force is above 10N to start motor (only once)
    if (force > 10.0 && !motorTriggered) {
        motorTriggered = true;  // Set flag to prevent multiple runs
        Serial.println("Force > 10N detected! Starting motor...");
        
        // Test motor sequence
        analogWrite(R_PWM, 128);  // 50% speed forward
        analogWrite(L_PWM, 0);
        delay(3000);

        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, 0);
        delay(1000);

        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, 128);  // 50% speed reverse
        delay(3000);

        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, 0);
        
        Serial.println("Motor test completed, waiting for next trigger...");
    }
    
    // Reset flag when force drops below 5N (hysteresis)
    if (force < 5.0) {
        motorTriggered = false;
    }
    
    delay(100);  // Read every 100ms
    
    // Stop after 10 seconds
    static unsigned long startTime = millis();
    if (millis() - startTime > 10000) {  // 10 seconds
        Serial.println("10 seconds elapsed - stopping measurements...");
        while(true) {
            // Infinite loop - stops everything
            delay(1000);
        }
    }
}



