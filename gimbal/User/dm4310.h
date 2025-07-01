#ifndef __DM4310_H__
#define __DM4310_H__
#include "main.h"
#include "stm32f4xx_hal.h"
#include "pid.h"
#include <math.h>
#include "dr16.h"
#define MOTOR1  0x01
#define MOTOR2  0x02
#define MOTOR3  0x03
#define P_MIN   -12.5
#define P_MAX   12.5
#define V_MIN   -45
#define V_MAX   45
#define KP_MIN  0
#define KP_MAX  500
#define KD_MIN  0
#define KD_MAX  5
#define T_MIN   -18
#define T_MAX   18
typedef struct
{
	int error;
  float p_int;
  float v_int;
  float t_int;
  float position;
  float velocity;
  float torque;
	float   Set_Speed;
  float   Set_angle;
	
	pid_struct_t Speed_pid;
  pid_struct_t Angle_pid;
}Motor_t;
typedef struct
{
	CAN_TxHeaderTypeDef hdr;
	uint8_t tx_data[8];
}CAN_TxPacketTypeDef;
typedef struct
{
	CAN_RxHeaderTypeDef hdr;
	uint8_t rx_data[8];
}CAN_RxPacketTypeDef;
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
extern CAN_HandleTypeDef hcan1;
void MIT_task(void const * argument);
extern rc_info_t rc_ctrl;
#endif
