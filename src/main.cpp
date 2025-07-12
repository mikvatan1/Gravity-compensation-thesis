#include <Arduino.h>

// Simpele test voor motor
const int LPWM = 5; // D5
const int RPWM = 3; // D3
void test_motor() {
    Serial.begin(9600);
    Serial.println("Motor test gestart");
    pinMode(LPWM, OUTPUT);
    pinMode(RPWM, OUTPUT);
    analogWrite(LPWM, 128); // Halve snelheid vooruit
    digitalWrite(RPWM, LOW);
    Serial.println("Motor draait 2 seconden op halve snelheid vooruit...");
    delay(2000);
    digitalWrite(LPWM, LOW); // Motor uit
    digitalWrite(RPWM, LOW);
    Serial.println("Motor gestopt");
}

void setup() {
    test_motor();
}

void loop() {
    // Optioneel: herhaal test of laat leeg
}
