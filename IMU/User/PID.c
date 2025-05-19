#include "PID.h"

/**************************几个PID除了过零保护外没有任何区别**********************************/
/********************************************************************************************
*角度环-KI给0
*速度环-KD给0
*先调速度环，等速度环能实现跳变跟随再调角度。
********************************************************************************************/

float Limit_Min_Max(float value,float min,float max);

/**
 * @brief PID数组初始化
 * @param PID PID数组
 * @param kp 
 * @param ki 
 * @param kd 
 * @param i_max 
 * @param out_max 
 */
void PID_init(PID_struct_t *PID,
              float kp,
              float ki,
              float kd,
              float kf,
              float i_max,
              float out_max)//PID初始化函数
{
  PID->kp      = kp;
  PID->ki      = ki;
  PID->kd      = kd;
  PID->kf      = kf;
  PID->i_max   = i_max;//积分限幅
  PID->out_max = out_max;//输出限幅
}

/**
 * @brief PID过零保护
 * @param pid PID结构体
 * @param angle_max 角度上限
 */
void PID_Protect(PID_struct_t *pid,float angle_max)
{
  float half_angle = angle_max/2;
	if(pid->ref[0] - pid->fdb > half_angle)
	{
		pid->fdb+=angle_max;
	}
	else if(pid->ref[0] - pid->fdb < -half_angle)
	{
		pid->fdb-=angle_max;
	}
}

/**
 * @brief PID角度环计算函数
 * @param PID PID结构体
 * @param ref 设定值
 * @param fdb 实际值
 * @param angle_max 角度上限
 * @param i_out 积分分离参数（为0时无效）
 * @return PID计算结果
 */
float PID_Calc_Angle(PID_struct_t *PID, float ref, float fdb,float angle_max,float i_out)//PID运算函数（目标，实际）
{
  PID->ref[0] = ref;
  PID->fdb = fdb;
  PID->f_out = PID->kf * (PID->ref[0] - PID->ref[1]);
	PID_Protect(PID,angle_max);//过零保护

  PID->err[0] = PID->ref[0] - PID->fdb;

  PID->p_out  = PID->kp * PID->err[0];
  PID->i_out += PID->ki * PID->err[0];
  PID->d_out  = PID->kd * (PID->err[0] - PID->err[1]);
  PID->i_out=Limit_Min_Max(PID->i_out, -PID->i_max, PID->i_max);
  
  PID->output = PID->p_out + PID->i_out + PID->d_out + PID->f_out;
  PID->output=Limit_Min_Max(PID->output, -PID->out_max, PID->out_max);

  PID->err[1] = PID->err[0];
  PID->ref[1] = PID->ref[0];
  return PID->output;
}

/**
 * @brief 速度环PID
 * @param PID PID结构体
 * @param ref 设定值
 * @param fdb 实际值
 * @return PID计算结果
 */
float PID_Calc_Speed(PID_struct_t *PID, float ref, float fdb)
{
  PID->ref[0] = ref;
  PID->fdb = fdb;

  PID->f_out = PID->kf * (PID->ref[0] - PID->ref[1]);

  PID->err[0] = PID->f_out + PID->ref[0] - PID->fdb;

  PID->p_out  = PID->kp * PID->err[0];
  PID->i_out += PID->ki * PID->err[0];
  PID->d_out  = PID->kd * (PID->err[0] - PID->err[1]);
  PID->i_out=Limit_Min_Max(PID->i_out, -PID->i_max, PID->i_max);
  
  PID->output = PID->p_out + PID->i_out + PID->d_out;
  PID->output=Limit_Min_Max(PID->output, -PID->out_max, PID->out_max);

  PID->ref[1] = PID->ref[0];
  PID->err[1] = PID->err[0];

  return PID->output;
}

/**
 * @brief 限制一个整数变量 value 在指定的最小值 min 和最大值 max 之间
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 
 */
float Limit_Min_Max(float value,float min,float max)
{
	if(value<min)
		return min;
	else if(value>max)
		return max;
	else return value;
}

