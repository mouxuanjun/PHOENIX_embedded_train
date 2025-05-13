#ifndef __DBUS_H__
#define __DBUS_H__

#include "main.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
    int16_t ch[2];
}rc_info_t;

extern rc_info_t rc_ctrl;
extern uint8_t rx_data[18];

void RemoteDataProcess(rc_info_t *rc_ctrl, uint8_t rx_data[18]);
void control(rc_info_t *rc_ctrl, float * target_angle_form);

#endif
