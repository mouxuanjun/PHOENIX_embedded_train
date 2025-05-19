#ifndef __PID_H_
#define __PID_H_

#include "main.h"
#include "stm32f4xx.h"

typedef struct
{
  float kp;
  float ki;
  float kd;
  float kf;

  float i_max;
  float out_max;
  
  float ref[2]; //…Ë∂®
  float fdb; //∑¥¿°
  float err[2];

  float p_out;
  float i_out;
  float d_out;
  float f_out;
  float output;
}PID_struct_t;

void PID_init(PID_struct_t *PID,float kp,float ki,float kd,float kf,float i_max,float out_max);
float PID_Calc_Angle(PID_struct_t *PID, float ref, float fdb,float angle_max,float i_out);
float PID_Calc_Speed(PID_struct_t *PID, float ref, float fdb);
float Limit_Min_Max(float value,float min,float max);

#endif


