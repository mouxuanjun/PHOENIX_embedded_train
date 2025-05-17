#ifndef __PID_H_
#define __PID_H_

#include "main.h"
#include "can.h"
#include "stm32f4xx.h"
#include "dvc_dji_gm6020.h"


typedef struct
{   
    float P;
    float I;
	  float D;
	  float F;//前馈
	  float integral;  // 积分项
    float last_error;// 上次误差
    float last_target;// 前馈用上次目标
	  float derivative;//微分

}PID;


float position_PID(float target, float current,PID *PosePID);
float velocity_PID(float target, float current,PID *VelPID);
void init_sine_generator(float initial_phase_rad);
float generate_sine_target(void);
#endif
