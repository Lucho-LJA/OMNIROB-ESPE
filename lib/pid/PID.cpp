#include "Arduino.h"
#include "PID.h"

PID::PID(float kp, float ki, float kd, int16_t delta_t, int16_t max_pwm_):
    kp_(kp),
    ki_(ki),
    kd_(kd),
    delta_t_(delta_t),
    max_pwm_(max_pwm)

{
}

int PID::compute(float setpoint, float measured_value)
{
    double error;
    int pid;

    //setpoint is constrained between min and max to prevent pid from having too much error
    error = setpoint - measured_value;
    integral_ += error*delta_t_/1000;
    if (integral_>max_pwm_)
    {
        integral_=max_pwm_
    }else if(integral_<-max_pwm_)
    {
        integral_=-max_pwm_;
    }
    derivative_ = (error - prev_error_)*1000/delta_t_;

    pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
    if (pid> pwm_max_)
    {
        pid=pwm_max_;
    }else if(pid<0)
    {
        pid=0;
    }
    prev_error_ = error;

    return pid;
}

void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
