#ifndef __DM4310_H__
#define __DM4310_H__

#include "main.h"
#include "pid.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "DBUS.h"

#define P_MIN -12.5
#define P_MAX 12.5
#define V_MIN -45
#define V_MAX 45
#define T_MIN -18
#define T_MAX 18

typedef struct {
    float error_code;
    float current_angle;
    float current_velocity;
    float current_torque;
	
	pid_a DM4310_angle;	
	pid_v DM4310_velocity;
} DM4310_st;

extern DM4310_st rc_4310;

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
void if_enable(void);
void DM4310_RxData(uint32_t StdId, uint8_t rx_data[8]);

#endif
