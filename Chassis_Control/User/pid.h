#ifndef __PID_H__
#define __PID_H__

#include "main.h"

typedef struct{
	float Kp_v;
    float Ki_v;
    float Kd_v;
    float target_velocity;
	float current_velocity;
    float error;
    float error_difference;
    float error_last;
    float MAX_I;
    float Kp_v_out;
    float Ki_v_out;
    float Kd_v_out;
    float PID_velocity_out;
} pid_v;

void pid_velocity_init(pid_v * pid_velocity, float Kp, float Ki, float Kd, float MAX);
float pid_velocity_control(pid_v * pid_velocity, float current_speed_form, float target_speed_form);

#endif
