#ifndef __DBUS_H__
#define __DBUS_H__

#include "main.h"
#include "stdlib.h"
#include "string.h"
#include "Chassis.h"

typedef struct{
    int16_t ch[4];
    char s1, s2;
	int16_t wheel;
} rc_info_t;

extern rc_info_t rc_ctrl;

void RemoteDataProcess(rc_info_t *rc_ctrl, uint8_t rx_data[18]);
void Control(void);

#endif
