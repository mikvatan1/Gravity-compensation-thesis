#include <Arduino.h>

// Simpele test voor motor
const int RPWM = 5; // D5
const int LPWM = 3; // D3
const int R_EN = 8; // D8
const int L_EN = 7; // D7
void test_motor() {
    Serial.begin(9600);
    Serial.println("Motor test gestart");
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);

    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

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
