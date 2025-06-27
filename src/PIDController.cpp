#include "PIDController.h"

// Constructor: initializes PID gains and internal state
PIDController::PIDController(float kp, float ki, float kd) {
    Kp = kp;         // Proportional gain
    Ki = ki;         // Integral gain
    Kd = kd;         // Derivative gain
    integral = 0;    // Integral term accumulator
    lastError = 0;   // Last error value for derivative calculation
}

// Resets the integral and last error (useful when stopping/restarting control)
void PIDController::reset() {
    integral = 0;
    lastError = 0;
}

// Allows you to change the PID gains at runtime
void PIDController::setTunings(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

// Main PID calculation function
float PIDController::compute(float setpoint, float measured) {
    float error = setpoint - measured; // Calculate error
    integral += error;                 // Accumulate the integral term
    float derivative = error - lastError; // Calculate the derivative term
    float output = Kp * error + Ki * integral + Kd * derivative; // PID formula
    lastError = error;                 // Store error for next derivative calculation
    return output;                     // Return the control output
}
