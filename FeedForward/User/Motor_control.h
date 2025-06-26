#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "GM6020.h"
#include "PID.h"
#include "freeRTOS.h"
#include "VOFT_Uartx.h"
#include "cmsis_os.h"
#include <math.h>

extern Moto_GM6020_t GM6020;
extern Moto_GM6020_t GM6020_pitch;
void Send_GM6020_Motor_Message(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
// CAN相关定义
#define CAN_CHASSIS_ALL_ID 0x1FF    // 标识符的ID
#define CHASSIS_CAN hcan1           // 主CAN总线
#define CHASSIS_CAN2 hcan2          // 备用CAN总线

#endif

