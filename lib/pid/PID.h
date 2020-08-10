#ifndef PID_H
    #define PID_H


    #include "Arduino.h"

    class PID
    {
        public:
            PID( float kp, float ki, float kd, int16_t delta_t, int16_t max_pwm);
            int compute(float setpoint, float measured_value);
            void updateConstants(float kp, float ki, float kd);

            private:
            float kp_;
            float ki_;
            float kd_;
            double integral_;
            double derivative_;
            double prev_error_;
            int16_t delta_t_;
            int16_t max_pwm_;
    };

#endif
