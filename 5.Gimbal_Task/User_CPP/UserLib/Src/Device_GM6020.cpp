//
// Created by 22560 on 25-4-20.
//

#include "Device_GM6020.hpp"
#include "Device_GM6020.h"

#include "Algorithm_Math.hpp"
#include "Middleware_CAN.hpp"

extern Class_CAN CPP_CAN1;
extern Class_CAN CPP_CAN2;

void GM6020::Init(uint8_t id, CAN_HandleTypeDef* hcan) {
    CPP_motor_id = id;
    CPP_Motor_hcan = hcan;

    pid_speed.Init(speed, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                   0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                   0.0f, CPP_PID_DFirst_Enable);
    CPP_PID_Params_t GM6020_pid_speed = {
        30.0f, 1500.0f, 1000.0f, 10.0f,
        9000.0f, 23900.0f
    };
    pid_speed.Set_params(&GM6020_pid_speed);
    pid_speed.Set_DFirst(CPP_PID_DFirst_Enable);

    pid_angle.Init(GM6020_angle, 0.05f, 0.06f, 400.0f, 0.01f, 0.0f,
                   0.0f, 0.0f, 50.0f, 0.0f, 0.0f,
                   0.0f, CPP_PID_DFirst_Enable);
    CPP_PID_Params_t GM6020_pid_angle = {
        0.06f, 0.006f, 400.0f, 0.01f,
        1000.0f, 5000.0f
    };
    pid_angle.Set_params(&GM6020_pid_angle);
    pid_angle.Set_DFirst(CPP_PID_DFirst_Enable);
}

void GM6020::Update() {
    if (CPP_Motor_hcan->Instance == CAN1) {
        if (CPP_CAN1.CPP_CAN_Rx_CAN_id != CPP_motor_id + 0x204) return;
        CPP_motor_info.angle_ecd = (CPP_CAN1.CPP_CAN_RxBuffer[0] << 8) | CPP_CAN1.CPP_CAN_RxBuffer[1];
        CPP_motor_info.speed_rpm = (CPP_CAN1.CPP_CAN_RxBuffer[2] << 8) | CPP_CAN1.CPP_CAN_RxBuffer[3];
        CPP_motor_info.current_raw = (CPP_CAN1.CPP_CAN_RxBuffer[4] << 8) | CPP_CAN1.CPP_CAN_RxBuffer[5];
        CPP_motor_info.temperate = CPP_CAN1.CPP_CAN_RxBuffer[6];
    }
    else if (CPP_Motor_hcan->Instance == CAN2) {
        if (CPP_CAN2.CPP_CAN_Rx_CAN_id != CPP_motor_id + 0x204) return;
        CPP_motor_info.angle_ecd = (CPP_CAN2.CPP_CAN_RxBuffer[0] << 8) | CPP_CAN2.CPP_CAN_RxBuffer[1];
        CPP_motor_info.speed_rpm = (CPP_CAN2.CPP_CAN_RxBuffer[2] << 8) | CPP_CAN2.CPP_CAN_RxBuffer[3];
        CPP_motor_info.current_raw = (CPP_CAN2.CPP_CAN_RxBuffer[4] << 8) | CPP_CAN2.CPP_CAN_RxBuffer[5];
        CPP_motor_info.temperate = CPP_CAN2.CPP_CAN_RxBuffer[6];
    }
}

void GM6020::Ctrl_Angle(float raw_Angle) {
    int16_t Angle = (int16_t)(raw_Angle * 8192.0f / (2 * PI));
    if(raw_Angle == 0.0f) Angle = 0;
    int16_t Max = 8191;
    int16_t Min = 0;
    Math_Constrain(Angle, Min, Max);

    pid_angle.Set_Now(CPP_motor_info.angle_ecd);
    pid_angle.Set_Target(Angle);
    target_angle = Angle;
    pid_angle.CPP_TIM_Adjust_PeriodElapsedCallback();
    Ctrl_Speed(pid_angle.Get_Out());
}

void GM6020::Ctrl_Speed(int16_t Speed) {
    int16_t Max = 5000;
    int16_t Min = -5000;
    Math_Constrain(Speed, Min, Max);

    pid_speed.Set_Now(CPP_motor_info.speed_rpm);
    pid_speed.Set_Target(Speed);
    target_speed = Speed;
    pid_speed.CPP_TIM_Adjust_PeriodElapsedCallback();
    Ctrl_Current(pid_speed.Get_Out());
}

void GM6020::Ctrl_Current(int16_t current) {
    target_current = current;
}

void GM6020_All::Init(GM6020* GM6020_1, GM6020* GM6020_2,
                  GM6020* GM6020_3, GM6020* GM6020_4) {
    GM6020_1_Handle = GM6020_1;
    GM6020_2_Handle = GM6020_2;
    GM6020_3_Handle = GM6020_3;
    GM6020_4_Handle = GM6020_4;

    if (GM6020_1_Handle || GM6020_2_Handle || GM6020_3_Handle || GM6020_4_Handle) {
        if (GM6020_1_Handle && (!CAN_Handle || CAN_Handle == GM6020_1_Handle->CPP_Motor_hcan)) {
            CAN_Handle = GM6020_1_Handle->CPP_Motor_hcan;
        } else if (GM6020_1_Handle) {
            return;
        }
        if (GM6020_2_Handle && (!CAN_Handle || CAN_Handle == GM6020_2_Handle->CPP_Motor_hcan)) {
            CAN_Handle = GM6020_2_Handle->CPP_Motor_hcan;
        } else if (GM6020_2_Handle) {
            return;
        }
        if (GM6020_3_Handle && (!CAN_Handle || CAN_Handle == GM6020_3_Handle->CPP_Motor_hcan)) {
            CAN_Handle = GM6020_3_Handle->CPP_Motor_hcan;
        } else if (GM6020_3_Handle) {
            return;
        }
        if (GM6020_4_Handle && (!CAN_Handle || CAN_Handle == GM6020_4_Handle->CPP_Motor_hcan)) {
            CAN_Handle = GM6020_4_Handle->CPP_Motor_hcan;
        } else if (GM6020_4_Handle) {
            return;
        }
    }
}

void GM6020_All::Ctrl_Current() {
    uint8_t TxBuffer[8] = {0};
    if(GM6020_1_Handle != nullptr) {
        TxBuffer[0] = GM6020_1_Handle->target_current >> 8;
        TxBuffer[1] = GM6020_1_Handle->target_current;
    }else {
        TxBuffer[0] = 0;
        TxBuffer[1] = 0;
    }
    if(GM6020_2_Handle != nullptr) {
        TxBuffer[2] = GM6020_2_Handle->target_current >> 8;
        TxBuffer[3] = GM6020_2_Handle->target_current;
    }else {
        TxBuffer[2] = 0;
        TxBuffer[3] = 0;
    }
    if(GM6020_3_Handle != nullptr) {
        TxBuffer[4] = GM6020_3_Handle->target_current >> 8;
        TxBuffer[5] = GM6020_3_Handle->target_current;
    }else {
        TxBuffer[4] = 0;
        TxBuffer[5] = 0;
    }
    if(GM6020_4_Handle != nullptr) {
        TxBuffer[6] = GM6020_4_Handle->target_current >> 8;
        TxBuffer[7] = GM6020_4_Handle->target_current;
    }else {
        TxBuffer[6] = 0;
        TxBuffer[7] = 0;
    }
    CPP_CAN1.CPP_CAN_Tx_Data(TxBuffer, 0x1FF);
}

void GM6020_All::Update() {
    if (CAN_Handle->Instance == CAN1) {
        switch (CPP_CAN1.CPP_CAN_Rx_CAN_id) {
        case 0x205: GM6020_1_Handle->Update();
            break;
        case 0x206: GM6020_2_Handle->Update();
            break;
        case 0x207: GM6020_3_Handle->Update();
            break;
        case 0x208: GM6020_4_Handle->Update();
            break;
        }
    }
    else if (CAN_Handle->Instance == CAN2) {
        switch (CPP_CAN2.CPP_CAN_Rx_CAN_id) {
        case 0x205: GM6020_1_Handle->Update();
            break;
        case 0x206: GM6020_2_Handle->Update();
            break;
        case 0x207: GM6020_3_Handle->Update();
            break;
        case 0x208: GM6020_4_Handle->Update();
            break;
        }
    }
}

GM6020 GM6020_3;

GM6020_All GM6020_All1;
