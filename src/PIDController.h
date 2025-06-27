#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

// PIDController class defines the interface and internal variables for a PID controller
class PIDController {
public:
    // Constructor: initialize PID gains
    PIDController(float kp, float ki, float kd);
    // Reset the internal state (integral and last error)
    void reset();
    // Compute the PID output based on setpoint and measured value
    float compute(float setpoint, float measured);
    // Change the PID gains at runtime
    void setTunings(float kp, float ki, float kd);
private:
    float Kp, Ki, Kd;      // PID gain parameters
    float integral;        // Accumulated integral term
    float lastError;       // Last error value for derivative calculation
};

#endif // PIDCONTROLLER_H
