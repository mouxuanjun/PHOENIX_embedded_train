//
// Created by 29812 on 25-5-4.
//

#ifndef TASK_H
#define TASK_H
#include "cmsis_os.h"
#include "gpio.h"
#include "motor_dm.h"
#ifdef __cplusplus
extern "C" {
#endif

    void dm_task(void const * argument);
#ifdef __cplusplus
}
#endif

#endif //TASK_H
