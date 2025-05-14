#ifndef __BSP_CAN_H_
#define __BSP_CAN_H_

#define Motor_1_ID 0x205
#define Motor_2_ID 0x206


#include "main.h"
#include "can.h"
#include "stm32f4xx.h"
#include "dvc_dji_gm6020.h"



void CAN_Filter_Init(void);

#endif
