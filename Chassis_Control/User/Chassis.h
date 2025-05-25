#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "main.h"
#include "math.h"
#include "DBUS.h"

typedef struct{
	float vx;
	float vy;
	float vw;
}chassis_st;

extern chassis_st chassis;

#endif
