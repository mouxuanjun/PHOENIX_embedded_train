#ifndef __GM6020_H__
#define __GM6020_H__

#include "main.h"
#include "pid.h"
#include "bsp_can.h"
#include "cmsis_os.h"

typedef struct {
    uint16_t current_angle;
    int16_t current_velocity;
    int16_t current_torque;
	
	pid_a GM6020_angle;	
	pid_v GM6020_velocity;
} GM6020_st;

extern GM6020_st rc_6020;


void GM6020_RxData(uint32_t StdId, uint8_t rx_data[8]);
void GM6020_task(void const * argument);

#endif
