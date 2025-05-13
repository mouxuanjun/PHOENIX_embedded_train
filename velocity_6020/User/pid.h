#ifndef __PID_H__
#define __PID_H__

#include "main.h"
#include "GM6020.h"

typedef struct{
    int16_t target_speed;
    int16_t error;
    int16_t sum_error;
    int16_t error_difference;
    int16_t error_last;
    int16_t error_out;
    int16_t PID_out;
	  double Kp;
    double Ki;
    double Kd;
} pid_t;

//记着赋值！！！！

extern pid_t pid;
extern rc_t rc_6020;

void pid_init(void);
int16_t pid_control(pid_t * pid, rc_t * rc_6020);

#endif
