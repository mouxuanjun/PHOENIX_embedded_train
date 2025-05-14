//
// Created by 29812 on 25-5-13.
//
#include "motor_dm.h"

#include <bits/this_thread_sleep.h>

DM4310::DM4310(const Motor_DM_Mode mode, CAN_HandleTypeDef* hcan, const uint8_t can_id, const uint8_t master_id , const Pid_Type pid_type)
{
    this->mode = mode;
    this->hcan = hcan;
    this->can_id = can_id;
    this->master_id = master_id;
    this->pid_type = pid_type;
}

void DM4310::Set_Angle(const float angle, float speed)
{
    if (this->mode == MIT_MODE)
    {
        this->MIT_Ctl_Msg_Send(angle, speed, 2, 1, 0);
    }
}

void DM4310::Set_Speed(const float speed)
{
    if (this->mode == MIT_MODE)
    {
        this->MIT_Ctl_Msg_Send(0, speed, 0, 1, 0);
    }
}

void DM4310::MIT_Ctl_Msg_Send(const float pos , const float vel , const float kp , const float kd , const float torque) const
{

    CAN_TxHeaderTypeDef tx_header;
    switch(this->mode)
    {
    case(MIT_MODE):
        tx_header.StdId = this->can_id;
        break;
    case(POSITION_AND_SPEED_MODE):
        tx_header.StdId = this->can_id+0x100;
        break;
    case(SPEED_MODE):
        tx_header.StdId = this->can_id+0x200;
        break;
    case(NONE_MODE):
        DM_Error_Handler();
        break;
    }
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    //计算发送数据
    uint16_t pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);
    uint16_t vel_tmp = float_to_uint(vel, V_MIN, V_MAX, 12);
    uint16_t kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t tor_tmp = float_to_uint(torque, T_MIN, T_MAX, 12);
    tx_data[0] = (pos_tmp >> 8);
    tx_data[1] = pos_tmp;
    tx_data[2] = (vel_tmp >> 4);
    tx_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    tx_data[4] = kp_tmp;
    tx_data[5] = (kd_tmp >> 4);
    tx_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    tx_data[7] = tor_tmp;
    HAL_CAN_AddTxMessage(this->hcan,&tx_header,tx_data,&tx_mailbox);
}


void DM4310::Deserialize_Status(const uint8_t* status_buffer)
{

    int p_int=(status_buffer[1]<<8)|status_buffer[2];
    int v_int=(status_buffer[3]<<4)|(status_buffer[4]>>4);
    int t_int=((status_buffer[4]&0xF)<<8)|status_buffer[5];
    this->status.Position = uint_to_float(p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
    this->status.Speed = uint_to_float(v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
    this->status.Torque= uint_to_float(t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
    this->status.error_code = status_buffer[0]>>4;

}

void DM4310::Enable() const
{
    CAN_TxHeaderTypeDef tx_header;
    switch(this->mode)
    {
        case(MIT_MODE):
            tx_header.StdId = this->can_id;
            break;
        case(POSITION_AND_SPEED_MODE):
            tx_header.StdId = this->can_id+0x100;
            break;
        case(SPEED_MODE):
            tx_header.StdId = this->can_id+0x200;
            break;
        case(NONE_MODE):
            DM_Error_Handler();
            break;
    }
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    tx_data[0] = 0xFF;
    tx_data[1] = 0xFF;
    tx_data[2] = 0xFF;
    tx_data[3] = 0xFF;
    tx_data[4] = 0xFF;
    tx_data[5] = 0xFF;
    tx_data[6] = 0xFF;
    tx_data[7] = 0xFC;
    HAL_CAN_AddTxMessage(this->hcan,&tx_header,tx_data,&tx_mailbox);

}

void DM4310::Disable() const
{
    CAN_TxHeaderTypeDef tx_header;
    switch(this->mode)
    {
    case(MIT_MODE):
        tx_header.StdId = this->can_id;
        break;
    case(POSITION_AND_SPEED_MODE):
        tx_header.StdId = this->can_id+0x100;
        break;
    case(SPEED_MODE):
        tx_header.StdId = this->can_id+0x200;
        break;
    case(NONE_MODE):
        DM_Error_Handler();
        break;
    }
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    tx_data[0] = 0xFF;
    tx_data[1] = 0xFF;
    tx_data[2] = 0xFF;
    tx_data[3] = 0xFF;
    tx_data[4] = 0xFF;
    tx_data[5] = 0xFF;
    tx_data[6] = 0xFF;
    tx_data[7] = 0xFD;
    HAL_CAN_AddTxMessage(this->hcan,&tx_header,tx_data,&tx_mailbox);
}

void DM4310::Reset_Error() const
{
    CAN_TxHeaderTypeDef tx_header;
    switch(this->mode)
    {
    case(MIT_MODE):
        tx_header.StdId = this->can_id;
        break;
    case(POSITION_AND_SPEED_MODE):
        tx_header.StdId = this->can_id+0x100;
        break;
    case(SPEED_MODE):
        tx_header.StdId = this->can_id+0x200;
        break;
    case(NONE_MODE):
        DM_Error_Handler();
        break;
    }
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    tx_data[0] = 0xFF;
    tx_data[1] = 0xFF;
    tx_data[2] = 0xFF;
    tx_data[3] = 0xFF;
    tx_data[4] = 0xFF;
    tx_data[5] = 0xFF;
    tx_data[6] = 0xFF;
    tx_data[7] = 0xFB;
    HAL_CAN_AddTxMessage(this->hcan,&tx_header,tx_data,&tx_mailbox);
}


void DM4310::Pid_Update()
{

}

void DM4310::DM_Error_Handler()
{

}

void DM4310::Set_Can_ID(uint8_t can_id)
{
    this->can_id = can_id;
}
void DM4310::Set_Master_ID(uint8_t master_id)
{
    this->master_id = master_id;
}

void DM4310::Set_CTL_Mode(Motor_DM_Mode mode)
{
    this->mode = mode;
}

void DM4310::Bind_CAN(CAN_HandleTypeDef* hcan)
{
    this->hcan = hcan;
}


