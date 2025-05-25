#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "main.h" 
#include "can.h"
#include "GM6020.h"
#include "DM4310.h"
#include "GM3508.h"

#define P_MIN -12.5
#define P_MAX 12.5
#define V_MIN -45
#define V_MAX 45
#define T_MIN -18
#define T_MAX 18

extern uint8_t target_Torque[8];

void Filter_Init(void);
void GM6020_Control(int16_t motor);
void DM4310_Control(uint16_t id, float angle, float velocity, float Kp, float Kd, float torque);
void GM3508_Control(int16_t motor1, int16_t motor2);

#endif 
