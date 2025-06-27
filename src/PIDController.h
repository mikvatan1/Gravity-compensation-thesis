#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    void reset();
    float compute(float setpoint, float measured);
    void setTunings(float kp, float ki, float kd);
private:
    float Kp, Ki, Kd;
    float integral;
    float lastError;
};

#endif // PIDCONTROLLER_H
