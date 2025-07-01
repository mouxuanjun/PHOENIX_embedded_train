#include "pid.h"
pid_struct_t pid_motor_speed;
pid_struct_t pid_motor_angle;
float Limit_Min_Max(float value,float min,float max);
void pid_init(pid_struct_t *pid,
    float kp,
    float ki,
    float kd,
    float i_max,
    float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_max = i_max;
    pid->out_max = out_max;
}
void pid_Protect(pid_struct_t *pid,float angle_max)
{ 
  float half_angle = angle_max/2;
	if(pid->ref[0] - pid ->fdb > half_angle)
	{
		pid->fdb+=angle_max;
	}
	else if(pid->ref[0] - pid->fdb < -half_angle)
	{
		pid->fdb-=angle_max;
	}
}

float PID_Calc_Angle(pid_struct_t *pid, float ref, float fdb,float angle_max,float i_out)//PID运算函数（目标，实际）
{
  pid->ref[0] = ref;
  pid->fdb = fdb;
	pid_Protect(pid,angle_max);

  pid->err[0] = pid->ref[0] - pid->fdb;

  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  pid->i_out  = Limit_Min_Max(pid->i_out, - pid->i_max, pid->i_max);
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  pid->output = Limit_Min_Max(pid->output, -pid->out_max, pid->out_max);

  pid->err[1] = pid->err[0];
  pid->ref[1] = pid->ref[0];
  return pid->output;
}
float PID_Calc_Speed(pid_struct_t *pid, float ref, float fdb)
{
    pid->ref[0] = ref;
    pid->fdb = fdb;

    pid->err[0] = pid->ref[0] - pid->fdb;

    pid->p_out  = pid->kp * pid->err[0];
    pid->i_out += pid->ki * pid->err[0];
    pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
    pid->i_out  = Limit_Min_Max(pid->i_out, - pid->i_max, pid->i_max);
  
    pid->output = pid->p_out + pid->i_out + pid->d_out;
    pid->output = Limit_Min_Max(pid->output, -pid->out_max, pid->out_max);

    pid->ref[1] = pid->ref[0];
    pid->err[1] = pid->err[0];

  return pid->output;
}
float Limit_Min_Max(float value,float min,float max)
{
	if(value<min)
		return min;
	else if(value>max)
		return max;
	else return value;
}
