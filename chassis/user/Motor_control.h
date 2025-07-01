#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "GM6020.h"
#include "pid.h"
#include "freeRTOS.h"
#include "dri_uart.h"
#include "cmsis_os.h"
#include <math.h>
#include "dr16.h"

extern Moto_GM6020_t motor_pitch;  //0x205
extern Moto_GM6020_t motor_yaw;    //0x207

#endif

