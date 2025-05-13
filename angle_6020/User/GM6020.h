#ifndef __GM6020_H__
#define __GM6020_H__

#include "main.h"

typedef struct{
	uint16_t current_angle;
	int16_t current_speed;
	int16_t current_Torque;
	uint16_t last_angle;
	int16_t last_speed;
} rc_t;

extern uint8_t target_Torque[8];
extern rc_t rc_6020;

void GM6020_RxData(uint32_t StdID, uint8_t rx_data[8]);

#endif
