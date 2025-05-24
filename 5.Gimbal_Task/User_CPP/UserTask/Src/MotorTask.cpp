//
// Created by 22560 on 25-4-22.
//

#include "MotorTask.h"
#include <cmath>
#include "Algorithm_Math.hpp"
#include "can.h"
#include "cmsis_os.h"
#include "Device_DM4310.hpp"
#include "freertos.h"
#include "task.h"
#include "Middleware_CAN.hpp"
#include "Device_GM6020.hpp"
#include "DR16_Remote.hpp"
#include "usart.h"

// #define DEV_GM6020
// #define DEV_DM4310

extern GM6020 GM6020_3;
extern GM6020_All GM6020_All1;
extern DM4310 DM4310_1;
extern Class_CAN CPP_CAN1;
extern RC_t RC;
extern uint8_t RC_Data[18];

float pitch = 0.0f;
float yaw = 2.0f;

void CPP_Motor_Task() {
    HAL_UART_Receive_DMA(&huart3, RC_Data, 18);

    //电机类Init
    GM6020_3.Init(3, &hcan1);
    GM6020_All1.Init(nullptr, nullptr, &GM6020_3, nullptr);
    DM4310_1.Init(DM_4310, 6, &hcan2, DM_CtrlMode_MIT);

    DM4310_1.Enable();

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1 / portTICK_PERIOD_MS; // 周期为1ms
    xLastWakeTime = xTaskGetTickCount();
    int cnt = 0;
    while (1) {
#ifdef DEV_GM6020
        //生成正弦波信号
        // cnt = (cnt == 500) ? 0 : cnt + 1;
        // P = 3 + 2.5 * sin(2 * PI * (1.0f / 500) * cnt);
        // GM6020_4.Ctrl_Angle(P);
        GM6020_4.Ctrl_Current(0.0f);
        GM6020_All1.Ctrl_Current();

#elifdef DEV_DM4310
        cnt = (cnt == 8000) ? 0 : cnt + 1;
        P = 3+ 3 * sin(2 * PI * (1.0f / 8000) * cnt);
        DM4310_1.Ctrl_Angle(P);
        // DM4340_2.Ctrl_SpeedPosition_Mode(0.0f,5.0f);
#endif
        if (RC.s1 == 3 && RC.s2 == 3) {
            pitch += 0.008f*(RC.ch1/660.0f);
            yaw -= 0.007f*(RC.ch0/660.0f);
            Math_Constrain(yaw,1.2f,3.2f);
            Math_Constrain(pitch,-0.35f,0.55f);
            GM6020_3.Ctrl_Angle(yaw);
            GM6020_All1.Ctrl_Current();
            DM4310_1.Ctrl_Angle(pitch+0.38f);
        }else if (RC.s1 == 3 && RC.s2 ==1) {
            // cnt = (cnt == 8000) ? 0 : cnt + 1;
            // pitch = 1 + 3 * sin(2 * PI * (1.0f / 8000) * cnt);
            // Math_Constrain(pitch,-1.5f,2.1f);
            // DM4310_1.Ctrl_Angle(pitch);
            pitch = 0.0f;
            yaw = 2.0f;
            DM4310_1.Ctrl_MIT_Mode(0.0f,0.0f,0.0f,0.0f,0.0f);
            GM6020_3.Ctrl_Current(0.0);
            GM6020_All1.Ctrl_Current();
        }else {
            pitch = 0.0f;
            yaw = 2.0f;
            DM4310_1.Ctrl_MIT_Mode(0.0f,0.0f,0.0f,0.0f,0.0f);
            GM6020_3.Ctrl_Current(0.0);
            GM6020_All1.Ctrl_Current();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

extern "C" {
void Motor_Task(void const* argument) {
    CPP_Motor_Task();
}
}
