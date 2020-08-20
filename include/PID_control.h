#ifndef PID_H
    #define PID_H

    float integral_[4]={0,0,0,0};
    float derivative_[4]={0,0,0,0};
    float prev_error_[4]={0,0,0,0};

    void PIDcompute(int pwm_max_)
    {
        float error[4];

        //SET_motor is constrained between min and max to prevent pid from having too much error
        error[0] = SET_motor[0] - RPM_motor[0];
        error[1] = SET_motor[1] - RPM_motor[1];
        error[2] = SET_motor[2] - RPM_motor[2];
        error[3] = SET_motor[3] - RPM_motor[3];


        integral_[0] += error[0]*dt_board/1000;
        integral_[1] += error[1]*dt_board/1000;
        integral_[2] += error[2]*dt_board/1000;
        integral_[3] += error[3]*dt_board/1000;


        if (integral_[0]>pwm_max_)
        {
            integral_[0]=pwm_max_;
        }else if(integral_[0]<-pwm_max_)
        {
            integral_[0]=-pwm_max_;
        }


        if (integral_[1]>pwm_max_)
        {
            integral_[1]=pwm_max_;
        }else if(integral_[1]<-pwm_max_)
        {
            integral_[1]=-pwm_max_;
        }

        if (integral_[2]>pwm_max_)
        {
            integral_[2]=pwm_max_;
        }else if(integral_[2]<-pwm_max_)
        {
            integral_[2]=-pwm_max_;
        }

        if (integral_[3]>pwm_max_)
        {
            integral_[3]=pwm_max_;
        }else if(integral_[3]<-pwm_max_)
        {
            integral_[3]=-pwm_max_;
        }


        derivative_[0] = (error[0] - prev_error_[0])*1000/dt_board;
        derivative_[1] = (error[1] - prev_error_[1])*1000/dt_board;
        derivative_[2] = (error[2] - prev_error_[2])*1000/dt_board;
        derivative_[3] = (error[3] - prev_error_[3])*1000/dt_board;


        PWM_motor[0] = (kp_motor[0] * error[0]) + (ki_motor[0] * integral_[0]) + (kd_motor[0] * derivative_[0]);
        PWM_motor[1] = (kp_motor[1] * error[1]) + (ki_motor[1] * integral_[1]) + (kd_motor[1] * derivative_[1]);
        PWM_motor[2] = (kp_motor[2] * error[2]) + (ki_motor[2] * integral_[2]) + (kd_motor[2] * derivative_[2]);
        PWM_motor[3] = (kp_motor[3] * error[3]) + (ki_motor[3] * integral_[3]) + (kd_motor[3] * derivative_[3]);


        if (PWM_motor[0]> pwm_max_)
        {
            PWM_motor[0]=pwm_max_;
        }else if(PWM_motor[0]<0)
        {
            PWM_motor[0]=0;
        }

        if (PWM_motor[1]> pwm_max_)
        {
            PWM_motor[1]=pwm_max_;
        }else if(PWM_motor[1]<0)
        {
            PWM_motor[1]=0;
        }

        if (PWM_motor[2]> pwm_max_)
        {
            PWM_motor[2]=pwm_max_;
        }else if(PWM_motor[2]<0)
        {
            PWM_motor[2]=0;
        }

        if (PWM_motor[3]> pwm_max_)
        {
            PWM_motor[3]=pwm_max_;
        }else if(PWM_motor[3]<0)
        {
            PWM_motor[3]=0;
        }

        prev_error_[0] = error[0];
        prev_error_[1] = error[1];
        prev_error_[2] = error[2];
        prev_error_[3] = error[3];
    }

#endif
