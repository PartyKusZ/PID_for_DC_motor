#include "pid.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, int16_t max_output, float anti_windup) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->anti_windup = anti_windup;
    pid->last_error = 0;
    pid->integral = 0;
}
void pid_reset(pid_t *pid) {
    pid->last_error = 0;
    pid->integral = 0;
}

int16_t pid_calculate(pid_t *pid, float setpoint, float process_var){
    float error = setpoint - process_var;
    pid->integral += (error * 0.01f);
    float derivative = (error - pid->last_error) / 0.01f;
    pid->last_error = error;
    
    if(pid->integral > pid->anti_windup) {
        pid->integral = pid->anti_windup;
    }

    if(pid->integral < -pid->anti_windup) {
        pid->integral = -pid->anti_windup;
    }

    int16_t output = (pid->kp * error) + (pid->ki * pid->integral)  + (pid->kd * derivative);
    if(output > pid->max_output) {
        output = pid->max_output;
    }
    if(output < -pid->max_output) {
        output = -pid->max_output;
    }
    return output;
}




