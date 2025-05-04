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

}PID;


float position_PID(float target, float current);
float velocity_PID(float target, float current);
void init_sine_generator(float initial_phase_rad);
float generate_sine_target(void);
#endif
