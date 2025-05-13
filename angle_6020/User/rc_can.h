#ifndef __RC_CAN_H__
#define __RC_CAN_H__

#include "main.h"

void Filter_Init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void GM6020_Control(int16_t motor1);

#endif
