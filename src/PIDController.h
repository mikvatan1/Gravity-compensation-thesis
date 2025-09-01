#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    void reset();
    float compute(float setpoint, float measured);
    void setTunings(float kp, float ki, float kd);

    // New: Get for plotting
    float getPTerm() const;
    float getITerm() const;
    float getDTerm() const;
    float getOutput() const;

private:
    float Kp, Ki, Kd;
    float integral;
    float lastError;
    unsigned long lastTime;  // For derivative calculation
    float lastDerivative;    // For derivative filtering

    // Store components
    float pTerm;
    float iTerm;
    float dTerm;
    float output;
};

#endif // PIDCONTROLLER_H