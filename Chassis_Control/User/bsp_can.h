#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "main.h" 
#include "can.h"
#include "GM3508.h"

void Filter_Init(void);
void GM3508_Control(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif 
