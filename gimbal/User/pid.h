#ifndef __PID_H_
#define __PID_H_
#include "main.h"
#include "stm32f4xx.h"
typedef struct _pid_struct_t
{
    float kp;
    float ki;
    float kd;
    float i_max;
    float out_max;

    float ref[2];
    float fdb;
    float err[2];

    float p_out;
    float i_out;
    float d_out;
    float output;
}pid_struct_t;

void pid_init(pid_struct_t *pid,
    float kp,
    float ki,
    float kd,
    float i_max,
    float out_max);
    void motor_PID_init(void);
    float PID_Calc_Angle(pid_struct_t *pid, float ref, float fdb,float angle_max,float i_out);
    float PID_Calc_Speed(pid_struct_t *pid, float ref, float fdb);
    float Limit_Min_Max(float value,float min,float max);
    void pid_Protect(pid_struct_t *pid,float angle_max);

#endif

