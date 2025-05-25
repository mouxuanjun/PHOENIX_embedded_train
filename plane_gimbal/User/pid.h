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

typedef struct{
	float Kp_a;
    float Ki_a;
    float Kd_a;
	float target_angle;	
    float current_angle;	
	float error;
    float error_last;
    float MAX_I;
    float limit;
    float Kp_a_out;
    float Ki_a_out;
    float Kd_a_out;
    float PID_angle_out;
	float zero_pro;
} pid_a;

void pid_velocity_init(pid_v * pid_velocity, float Kp, float Ki, float Kd, float MAX);
void pid_angle_init(pid_a * pid_angle, float Kp, float Ki, float Kd, float MAX, float limit_form, float zero_pro_form);
float pid_velocity_control(pid_v * pid_velocity, float current_speed_form, float target_speed_form);
float pid_angle_control(pid_a * pid_angle, float current_angle_form, float target_angle_form);
void Limit(pid_a * pid_angle, float H, float L);

#endif
