#ifndef __GM3508_H__
#define __GM3508_H__

#include "main.h"
#include "pid.h"
#include "bsp_can.h"
#include "cmsis_os.h"

typedef struct {
    uint16_t current_angle;
	uint16_t real_angle;
    int16_t current_velocity;
    int16_t current_torque;

	pid_v GM3508_velocity;
} GM3508_st;

extern GM3508_st rc_3508[4];

void GM3508_RxData(uint32_t StdId, uint8_t rx_data[8]);

#endif
