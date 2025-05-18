//
// Created by 22560 on 25-4-20.
//

#ifndef DEVICE_GM6020_HPP
#define DEVICE_GM6020_HPP

#include "Algorithm_PID.hpp"
#include "main.h"

typedef struct {
    uint16_t angle_ecd;
    int16_t speed_rpm;
    int16_t current_raw;
    int16_t temperate;
} GM6020_t;

class GM6020 {
public:
    uint8_t CPP_motor_id;
    CAN_HandleTypeDef* CPP_Motor_hcan;
    GM6020_t CPP_motor_info;
    Class_PID pid_speed;
    Class_PID pid_angle;

    int16_t target_current;
    int16_t target_speed;
    float target_angle;

    void Init(uint8_t id, CAN_HandleTypeDef* hcan);
    void Update();
    void Ctrl_Current(int16_t current_raw);
    void Ctrl_Speed(int16_t Speed);
    void Ctrl_Angle(float raw_Angle);
};

class GM6020_All {
public:
    int16_t Currency_ALL[4] = {0, 0, 0, 0};

    void Init(GM6020* GM6020_1, GM6020* GM6020_2,
                  GM6020* GM6020_3, GM6020* GM6020_4);
    void Update();
    void Ctrl_Current();//电机对象仅值传递，这里是发报

protected:
    GM6020* GM6020_1_Handle = nullptr;
    GM6020* GM6020_2_Handle = nullptr;
    GM6020* GM6020_3_Handle = nullptr;
    GM6020* GM6020_4_Handle = nullptr;
    CAN_HandleTypeDef* CAN_Handle = nullptr;
};

#endif //DEVICE_GM6020_HPP
