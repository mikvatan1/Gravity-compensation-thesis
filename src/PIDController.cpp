#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    integral = 0;
    lastError = 0;
}

void PIDController::reset() {
    integral = 0;
    lastError = 0;
}

void PIDController::setTunings(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

float PIDController::compute(float setpoint, float measured) {
    float error = setpoint - measured;
    integral += error;
    float derivative = error - lastError;
    float output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;
    return output;
}
