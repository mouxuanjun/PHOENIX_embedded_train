//
// Created by 22560 on 25-4-25.
//

#ifndef DEVICE_DM4310_HPP
#define DEVICE_DM4310_HPP

#include "Algorithm_PID.hpp"
#include "main.h"

typedef enum {
    DM_CtrlMode_MIT = 0,
    DM_CtrlMode_SpeedPosition = 1,
    DM_CtrlMode_Speed = 2,
} DM_CtrlMode;

typedef enum {
    DM_4310 = 1,
    DM_4340 = 2,
} DM_Type;

typedef struct {
    float position = 0.0f;
    float position_raw = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    uint8_t Err = 0;
} DM4310_t;

typedef struct {
    float position = 0.0f, velocity = 0.0f, torque = 0.0f,
          Kp = 0.0f, Kd = 0.0f;
} DM_CtrlTarget;

class DM4310 {
public:
    DM_Type motor_type = DM_4310;
    uint8_t motor_id = -1;
    CAN_HandleTypeDef* CAN_Handle = nullptr;
    DM_CtrlMode ctrl_mode = DM_CtrlMode_MIT;
    DM4310_t motor_info;
    Class_PID pid_speed;
    Class_PID pid_angle;
    bool enabled = false;

    DM_CtrlTarget ctrl_target;

    void Init(DM_Type _motor_type, uint8_t _id, CAN_HandleTypeDef* _hcan, DM_CtrlMode _ctrl_mode);
    void Enable();
    void Disable();
    void Update();
    void Ctrl_MIT_Mode(float Kp, float Kd, float pos, float vel, float tor);
    void Ctrl_SpeedPosition_Mode(float pos, float vel);
    void Ctrl_Speed(float Speed);
    void Ctrl_Angle(float Angle);
};

#endif //DEVICE_DM4310_HPP
