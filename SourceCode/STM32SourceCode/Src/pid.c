/*
 * pid.c
 *
 *  Created on: Nov 15, 2025
 *      Author: ASUS
 */

#include "pid.h"
#include "main.h"

PID_Controller angle_pid;
volatile float target_counts = 0;
volatile float error = 0;
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float sample_time_ms) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->dt = sample_time_ms / 1000.0f;

    pid->integral = 0;
    pid->prev_error = 0;

    pid->output_min = -100.0;      // Percentage: -100% to +100%
    pid->output_max = 100.0;
    pid->integral_max = 100.0;

    pid->dead_zone = 10;          // ±10 encoder counts
    pid->min_pwm_threshold = 15;   // 15% minimum PWM
}

void PID_Reset(PID_Controller *pid) {
    pid->integral = 0;
    pid->prev_error = 0;
}

float PID_Compute(PID_Controller *pid, float setpoint, float measured) {
    float error = setpoint - measured;

    // Dead zone check
    if (fabs(error) < pid->dead_zone) {
        pid->integral = 0;
        return 0;
    }

    // Proportional term
    float p_term = pid->Kp * error;

    // Integral term
    pid->integral += error * pid->dt;

    // Anti-windup
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }

    float i_term = pid->Ki * pid->integral;

    // Derivative term
    float derivative = (error - pid->prev_error) / pid->dt;
    float d_term = pid->Kd * derivative;

    pid->prev_error = error;

    // Calculate output
    float output = p_term + i_term + d_term;

    // Clamp output
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    // Apply minimum PWM threshold
    if (output != 0 && fabs(output) < pid->min_pwm_threshold) {
        output = (output > 0) ? pid->min_pwm_threshold : -pid->min_pwm_threshold;
    }

    return output;
}

void PID_MoveToAngle(float target_shaft_angle) {
    // Reset encoder counter to 0
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // Calculate target as signed value
    target_counts = (target_shaft_angle * GEAR_RATIO * COUNTS_PER_REV) / 360.0f;

//    sprintf(msg, "\r\n=== PID Movement ===\r\n");
//    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//    sprintf(msg, "Target: %.2f deg (%.0f counts)\r\n", target_shaft_angle, target_counts);
//    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    PID_Reset(&angle_pid);

    uint32_t timeout_cycles = 1000;
    uint32_t cycle_count = 0;
    uint32_t settled_count = 0;
    uint32_t settled_threshold = 30;

    uint32_t last_tick = HAL_GetTick();

    while(1) {
        // Wait for next sample time
        while((HAL_GetTick() - last_tick) < (uint32_t)(angle_pid.dt * 1000)) {
            // Wait
        }
        last_tick = HAL_GetTick();

        // Read encoder position directly as signed value
        // This handles wraparound automatically (0xFFFFFFFF becomes -1)
        int32_t delta = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

        // Error is target minus current position
        error = target_counts - (float)delta;

        // Get PID output (in percentage 0-100%)
        float pwm_output = PID_Compute(&angle_pid, target_counts, (float)delta);

        // Check if settled
        if (fabs(error) <= angle_pid.dead_zone) {
            settled_count++;
            if (settled_count >= settled_threshold) {
                sprintf(msg, "Target reached! Error: %.1f counts\r\n", error);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                break;
            }
        } else {
            settled_count = 0;
        }

        // Timeout check
        if (cycle_count++ > timeout_cycles) {
//            sprintf(msg, "TIMEOUT! Error: %.1f counts\r\n", error);
//            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            break;
        }

        // Motor control based on PWM output
        if (fabs(pwm_output) > 0.5) {
            // Convert percentage (0-100) to PWM value (0-999)
            uint16_t pwm_value = (uint16_t)((fabs(pwm_output) / 100.0f) * 999.0f);

            // Ensure we don't exceed ARR
            if (pwm_value > 999) pwm_value = 999;

            if (pwm_output > 0) {
                // Forward direction
                HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
            } else {
                // Reverse direction
                HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
            }

            TIM1->CCR1 = pwm_value;
        } else {
            StopMotor();
        }

        // Debug output every 25 cycles
        if (cycle_count % 25 == 0) {
//            sprintf(msg, "C:%lu Delta:%ld E:%.1f PWM:%.1f\r\n",
//                    cycle_count, delta, error, pwm_output);
//            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        }
    }

    // Stop motor
    StopMotor();
    HAL_Delay(500);

    // Final position
    int32_t final_delta = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    //DisplayResults(target_shaft_angle, final_delta);
}

void StopMotor(void) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    TIM1->CCR1 = 0;
}
