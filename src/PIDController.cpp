#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    integral = 0;
    lastError = 0;

    pTerm = 0;
    iTerm = 0;
    dTerm = 0;
    output = 0;
}

void PIDController::reset() {
    integral = 0;
    lastError = 0;
    pTerm = 0;
    iTerm = 0;
    dTerm = 0;
    output = 0;
}

void PIDController::setTunings(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

float PIDController::compute(float setpoint, float measured) {
    float error = setpoint - measured;
    integral += error;
    
    // Integral windup prevention - limit integral to reasonable range
    if (integral > 100.0) integral = 100.0;
    if (integral < -100.0) integral = -100.0;
    
    float derivative = error - lastError;

    pTerm = Kp * error;
    iTerm = Ki * integral;
    dTerm = Kd * derivative;

    output = pTerm + iTerm + dTerm;
    lastError = error;

    return output;
}

// Getters
float PIDController::getPTerm() const { return pTerm; }
float PIDController::getITerm() const { return iTerm; }
float PIDController::getDTerm() const { return dTerm; }
float PIDController::getOutput() const { return output; }