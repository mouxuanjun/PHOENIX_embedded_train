#ifndef DR_CAN_H
#define DR_CAN_H
#include "main.h"
#include "can.h"
#include "stm32f4xx.h"
#include "DM4310.h"


void CAN1_Filter_Init(void);
void CAN2_Filter_Init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
#endif


