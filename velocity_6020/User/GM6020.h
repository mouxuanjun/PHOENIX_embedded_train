#ifndef __GM6020_H__
#define __GM6020_H__

#include "main.h"

typedef struct{
	int16_t current_speed;
	int16_t current_Torque;
} rc_t;

extern uint8_t target_Torque[8];

void GM6020_RxData(uint32_t StdID, uint8_t rx_data[8]);

extern rc_t rc_6020;
#endif
