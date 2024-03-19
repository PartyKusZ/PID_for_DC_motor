#ifndef INC_PID_H_
#define INC_PID_H_
#include <stdint.h>

typedef struct {

    float kp;
    float ki;
    float kd;
    float last_error;
    float integral;
    float anti_windup;
    int16_t max_output;

}pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, int16_t max_output, float anti_windup);
void pid_reset(pid_t *pid);
int16_t pid_calculate(pid_t *pid, float setpoint, float process_var);


#endif /* INC_PID_H_ */
