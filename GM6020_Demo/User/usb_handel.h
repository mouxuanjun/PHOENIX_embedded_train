#ifndef __USB_HANDEL_H_
#define __USB_HANDEL_H_

#include "main.h"
#include "can.h"
#include "stm32f4xx.h"
#include "dvc_dji_gm6020.h"


typedef struct
{   
    float P;//设定速度
    float I;//设定角度
	  float D;

}PID;

#endif
