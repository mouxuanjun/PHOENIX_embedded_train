#ifndef __PID_H__
#define __PID_H__

#include "main.h"
#include "GM6020.h"
#include "math.h"

typedef struct{
    float target_speed;
    float error;
    float error_last;
    float PID_velocity_out;
    float pid_Kp;
    float pid_Ki;
    float pid_Kd;
    float MAX_I;
	float current_speed;
	float Kp_v;
    float Ki_v;
    float Kd_v;
} pid_v;

typedef struct{
	float target_angle;	
    float current_angle;	
	float Kp_a;
    float Ki_a;
    float Kd_a;
	float error;
    float error_last;
    float MAX_I;
    float pid_Kp;
    float pid_Ki;
    float pid_Kd;
    float PID_angle_out;	
} pid_a;

extern pid_v pid_velocity;
extern pid_a pid_angle;
extern rc_t rc_6020;
extern int cnt;

void pid_velocity_init(pid_v * pid_velocity, float Kp, float Ki, float Kd, int16_t MAX);
void pid_angle_init(pid_a * pid_angle, float Kp, float Ki, float Kd, int16_t MAX);
float first_order_filter(float current_filter, float last_filter);
float pid_velocity_control(pid_v * pid_velocity, float current_speed_form, float target_speed_form);
float pid_angle_control(pid_a * pid_angle, float current_angle_form, float target_angle_form);

#endif
