/*
 * pid.h
 *
 *  Created on: Nov 15, 2025
 *      Author: ASUS
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
// PID Controller Structure
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float dt;
    float output_min, output_max;
    float integral_max;
    float dead_zone;
    uint8_t min_pwm_threshold;
} PID_Controller;
extern PID_Controller angle_pid;
extern volatile float target_counts;
extern volatile float error;
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float sample_time_ms);
void PID_Reset(PID_Controller *pid);
float PID_Compute(PID_Controller *pid, float setpoint, float measured);
void PID_MoveToAngle(float target_shaft_angle);
void StopMotor(void);


#endif /* INC_PID_H_ */
