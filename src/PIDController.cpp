#include "PIDController.h"
#include <Arduino.h>  // For millis() function

PIDController::PIDController(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    integral = 0;
    lastError = 0;
    lastTime = 0;  // Initialize timing
    lastDerivative = 0;  // Initialize derivative filter

    pTerm = 0;
    iTerm = 0;
    dTerm = 0;
    output = 0;
}

void PIDController::reset() {
    integral = 0;
    lastError = 0;
    lastTime = 0;  // Reset timing
    lastDerivative = 0;  // Reset derivative filter
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
    unsigned long currentTime = millis();
    float error = setpoint - measured;
    
    // Calculate time difference in seconds
    float dt = 0.001;  // Default dt for first call or if time hasn't changed
    if (lastTime != 0) {
        dt = (currentTime - lastTime) / 1000.0;  // Convert ms to seconds
        if (dt <= 0) dt = 0.001;  // Prevent division by zero
    }
    
    integral += error * dt;  // Proper integral: Σ(error × dt)
    
    // Integral windup prevention - limit integral to reasonable range
    // Changed limits to allow for more gradual buildup with Ki=0.2
    //if (integral > 50.0) integral = 50.0;
    //if (integral < -50.0) integral = -50.0;
    
    float derivative = 0;
    if (lastTime != 0) {
        float rawDerivative = (error - lastError) / dt;  // Raw derivative calculation
        
        // Apply simple low-pass filter to smooth D-term (reduces noise)
        float alpha = 0.7;  // Filter strength (0.1 = heavy filtering, 0.9 = light filtering)
        derivative = alpha * rawDerivative + (1.0 - alpha) * lastDerivative;
        lastDerivative = derivative;
        
        // Alternative: Derivative on measurement (more stable)
        // derivative = -(measured - lastMeasured) / dt;  // Uncomment if still noisy
    }

    pTerm = Kp * error;
    iTerm = Ki * integral;
    dTerm = Kd * derivative;

    output = pTerm + iTerm + dTerm;
    
    lastError = error;
    lastTime = currentTime;

    return output;
}

// Getters
float PIDController::getPTerm() const { return pTerm; }
float PIDController::getITerm() const { return iTerm; }
float PIDController::getDTerm() const { return dTerm; }
float PIDController::getOutput() const { return output; }