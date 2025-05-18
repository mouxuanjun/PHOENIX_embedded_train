//
// Created by 22560 on 25-4-25.
//

#include "Device_DM4310.hpp"

#include "Algorithm_Math.hpp"
#include "Middleware_CAN.hpp"

extern Class_CAN CPP_CAN1;
extern Class_CAN CPP_CAN2;

/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

static int float_to_uint(float x, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void DM4310::Init(DM_Type _motor_type,uint8_t _id, CAN_HandleTypeDef* _hcan, DM_CtrlMode _ctrl_mode) {
    motor_type = _motor_type;
    ctrl_mode = _ctrl_mode;
    motor_id = _id;
    CAN_Handle = _hcan;

    pid_speed.Init(speed, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                   0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                   0.0f, CPP_PID_DFirst_Enable);
    CPP_PID_Params_t DM4310_pid_speed = {
        0.25f, 0.55f, 10.0f, 0.6f,
        3.0f, 18.0f
    };
    pid_speed.Set_params(&DM4310_pid_speed);
    pid_speed.Set_DFirst(CPP_PID_DFirst_Enable);

    pid_angle.Init(DM4310_angle, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                   0.0f, 0.0f, 0.05f, 0.0f, 0.0f,
                   0.0f, CPP_PID_DFirst_Enable);
    CPP_PID_Params_t DM4310_pid_angle = {
        150.0f, 20.0f, 0.0f, 35.0f,
        60.0f, 60.0f
    };
    pid_angle.Set_params(&DM4310_pid_angle);
    pid_angle.Set_DFirst(CPP_PID_DFirst_Enable);
}

void DM4310::Enable() {
    uint32_t CAN_id = 0xBB;
    uint8_t TxBuffer[8];
    if (ctrl_mode == DM_CtrlMode_SpeedPosition) {
        CAN_id = 0x100 + motor_id;
    }
    else if (ctrl_mode == DM_CtrlMode_Speed) {
        CAN_id = 0x200 + motor_id;
    }
    else if (ctrl_mode == DM_CtrlMode_MIT) {
        CAN_id = 0x000 + motor_id;
    }

    TxBuffer[0] = 0xFF;
    TxBuffer[1] = 0xFF;
    TxBuffer[2] = 0xFF;
    TxBuffer[3] = 0xFF;
    TxBuffer[4] = 0xFF;
    TxBuffer[5] = 0xFF;
    TxBuffer[6] = 0xFF;
    TxBuffer[7] = 0xFC;

    while (motor_info.Err != 1) {
        if (CAN_Handle->Instance == CAN1) {
            CPP_CAN1.CPP_CAN_Tx_Data(TxBuffer, CAN_id);
        }
        else if (CAN_Handle->Instance == CAN2) {
            CPP_CAN2.CPP_CAN_Tx_Data(TxBuffer, CAN_id);
        }
    }
}

void DM4310::Disable() {
    uint32_t CAN_id = 0xBB;
    uint8_t TxBuffer[8];
    if (ctrl_mode == DM_CtrlMode_SpeedPosition) {
        CAN_id = 0x100 + motor_id;
    }
    else if (ctrl_mode == DM_CtrlMode_Speed) {
        CAN_id = 0x200 + motor_id;
    }
    else if (ctrl_mode == DM_CtrlMode_MIT) {
        CAN_id = 0x000 + motor_id;
    }
    TxBuffer[0] = 0xFF;
    TxBuffer[1] = 0xFF;
    TxBuffer[2] = 0xFF;
    TxBuffer[3] = 0xFF;
    TxBuffer[4] = 0xFF;
    TxBuffer[5] = 0xFF;
    TxBuffer[6] = 0xFF;
    TxBuffer[7] = 0xFD;

    while (motor_info.Err != 0) {
        if (CAN_Handle->Instance == CAN1) {
            CPP_CAN1.CPP_CAN_Tx_Data(TxBuffer, CAN_id);
        }
        else if (CAN_Handle->Instance == CAN2) {
            CPP_CAN2.CPP_CAN_Tx_Data(TxBuffer, CAN_id);
        }
    }
}

void DM4310::Update() {
    if (CAN_Handle->Instance == CAN1) {
        if ((CPP_CAN1.CPP_CAN_RxBuffer[0] & 0x0F) != motor_id) return;
        const int p_int = (CPP_CAN1.CPP_CAN_RxBuffer[1] << 8) | CPP_CAN1.CPP_CAN_RxBuffer[2];
        const int v_int = (CPP_CAN1.CPP_CAN_RxBuffer[3] << 4) | (CPP_CAN1.CPP_CAN_RxBuffer[4] >> 4);
        const int t_int = ((CPP_CAN1.CPP_CAN_RxBuffer[4] & 0xF) << 8) | CPP_CAN1.CPP_CAN_RxBuffer[5];
        motor_info.position_raw = uint_to_float(p_int, -12.5f, 12.5f, 16); // (-12.5,12.5)

        switch (motor_type) {
        case DM_4310: {
            motor_info.velocity = uint_to_float(v_int, -45, 45, 12); // (-45.0,45.0)
            motor_info.torque = uint_to_float(t_int, -18, 18, 12); // (-18.0,18.0)
        }
        case DM_4340: {
            motor_info.velocity = uint_to_float(v_int, -10, 10, 12); // (-10.0,10.0)
            motor_info.torque = uint_to_float(t_int, -28, 28, 12); // (-28.0,28.0)
        }
        }

        if(motor_info.position_raw > 6.25f) motor_info.position = motor_info.position_raw-6.25f;
        else if (motor_info.position_raw < 0.0f) motor_info.position = motor_info.position_raw+6.25f;
        else motor_info.position = motor_info.position_raw;
        if(motor_info.position> 6.25f) motor_info.position = motor_info.position-6.25f;
        else if (motor_info.position < 0.0f) motor_info.position = motor_info.position+6.25f;

        motor_info.Err = (CPP_CAN1.CPP_CAN_RxBuffer[0] & 0xF0) >> 4;
        if (motor_info.Err == 1) enabled = true;
        else if (motor_info.Err == 0) enabled = false;
    }
    else if (CAN_Handle->Instance == CAN2) {
        if ((CPP_CAN2.CPP_CAN_RxBuffer[0] & 0x0F) != motor_id) return;
        const int p_int = (CPP_CAN2.CPP_CAN_RxBuffer[1] << 8) | CPP_CAN2.CPP_CAN_RxBuffer[2];
        const int v_int = (CPP_CAN2.CPP_CAN_RxBuffer[3] << 4) | (CPP_CAN2.CPP_CAN_RxBuffer[4] >> 4);
        const int t_int = ((CPP_CAN2.CPP_CAN_RxBuffer[4] & 0xF) << 8) | CPP_CAN2.CPP_CAN_RxBuffer[5];
        motor_info.position = uint_to_float(p_int, -12.5f, 12.5f, 16); // (-12.5,12.5)

        motor_info.velocity = uint_to_float(v_int, -45, 45, 12); // (-45.0,45.0)
        motor_info.torque = uint_to_float(t_int, -18, 18, 12); // (-18.0,18.0)
        motor_info.Err = (CPP_CAN2.CPP_CAN_RxBuffer[0] & 0xF0) >> 4;
        if (motor_info.Err == 1) enabled = true;
        else if (motor_info.Err == 0) enabled = false;
    }
}

void DM4310::Ctrl_MIT_Mode(float Kp, float Kd, float pos, float vel, float tor) {
    ctrl_target.position = pos;
    ctrl_target.velocity = vel;
    ctrl_target.torque = tor;
    ctrl_target.Kd = Kd;
    ctrl_target.Kp = Kp;

    uint8_t TxBuffer[8];
    uint32_t CAN_id;
    if (ctrl_mode == DM_CtrlMode_MIT) {
        CAN_id = 0x000 + motor_id;
    }
    else return;

    int tmp_pos = float_to_uint(ctrl_target.position, -12.5f, 12.5f, 16);
    int tmp_vel = float_to_uint(ctrl_target.velocity, -45, 45, 12);
    int tmp_tor = float_to_uint(ctrl_target.torque, -18, 18, 12);
    int tmp_kp = float_to_uint(ctrl_target.Kp, 0, 500.0f, 12);
    int tmp_kd = float_to_uint(ctrl_target.Kd, 0, 5.0f, 12);

    TxBuffer[0] = (tmp_pos >> 8);
    TxBuffer[1] = tmp_pos;
    TxBuffer[2] = (tmp_vel >> 4);
    TxBuffer[3] = ((tmp_vel & 0xF) << 4) | (tmp_kp >> 8);
    TxBuffer[4] = tmp_kp;
    TxBuffer[5] = (tmp_kd >> 4);
    TxBuffer[6] = ((tmp_kd & 0xF) << 4) | (tmp_tor >> 8);
    TxBuffer[7] = tmp_tor;

    if (CAN_Handle->Instance == CAN1) {
        CPP_CAN1.CPP_CAN_Tx_Data(TxBuffer, CAN_id);
    }
    else if (CAN_Handle->Instance == CAN2) {
        CPP_CAN2.CPP_CAN_Tx_Data(TxBuffer, CAN_id);
    }
}

void DM4310::Ctrl_SpeedPosition_Mode(float pos, float vel) {
    ctrl_target.position = pos;
    ctrl_target.velocity = vel;

    uint8_t TxBuffer[8];
    uint32_t CAN_id;
    if (ctrl_mode == DM_CtrlMode_SpeedPosition) {
        CAN_id = 0x100 + motor_id;
    }
    else return;

    *(float*)TxBuffer = ctrl_target.position;
    *(float*)(TxBuffer + 4) = ctrl_target.velocity;

    if (CAN_Handle->Instance == CAN1) {
        CPP_CAN1.CPP_CAN_Tx_Data(TxBuffer, CAN_id);
    }
    else if (CAN_Handle->Instance == CAN2) {
        CPP_CAN2.CPP_CAN_Tx_Data(TxBuffer, CAN_id);
    }
}

void DM4310::Ctrl_Speed(float Speed) {
    float Max = 60;
    float Min = -60;
    Math_Constrain(Speed, Min, Max);

    pid_speed.Set_Now(motor_info.velocity);
    pid_speed.Set_Target(Speed);
    ctrl_target.velocity = Speed;
    pid_speed.CPP_TIM_Adjust_PeriodElapsedCallback();
    Ctrl_MIT_Mode(0.0f, 0.0f, 0.0f, 0.0f, pid_speed.Get_Out());
}

void DM4310::Ctrl_Angle(float Angle) {
    float Max = 2*PI;
    float Min = 0;
    Math_Constrain(Angle, Min, Max);

    pid_angle.Set_Now(motor_info.position);
    pid_angle.Set_Target(Angle);
    ctrl_target.position = Angle;
    pid_angle.CPP_TIM_Adjust_PeriodElapsedCallback();
    Ctrl_Speed(pid_angle.Get_Out());
}

DM4310 DM4310_1;
DM4310 DM4310_2;
DM4310 DM4310_3;
DM4310 DM4310_4;
DM4310 DM4340_2;
