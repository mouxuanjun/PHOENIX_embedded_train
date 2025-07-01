#include "GM6020.h"
extern Moto_GM6020_t motor_pitch;  //0x205
extern Moto_GM6020_t motor_yaw;    //0x207
HAL_StatusTypeDef can_1;
HAL_StatusTypeDef can_2;
void Get_GM6020_Motor_Message(uint32_t StdId,uint8_t rx_data[8])
{
    switch(StdId)
    {
        case 0x205:
        {
            motor_pitch.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
            motor_pitch.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
            motor_pitch.torque_current = ((rx_data[4] << 8) | rx_data[5]);
            motor_pitch.temp           =  rx_data[6];
            break;
        }
				case 0x207:
				{
					  motor_yaw.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
            motor_yaw.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
            motor_yaw.torque_current = ((rx_data[4] << 8) | rx_data[5]);
            motor_yaw.temp           =  rx_data[6];
            break;
				}
    }
}
void Set_GM6020_Gimbal_pitch_Voltage(CAN_HandleTypeDef* hcan,Moto_GM6020_t motor_pitch)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             CAN2_tx_data[8] = {0};
    
    tx_header.StdId = 0x1FF;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;
    CAN2_tx_data[0] = ((int16_t)motor_pitch.Speed_pid.output>>8)&0xff;
    CAN2_tx_data[1] = ((int16_t)motor_pitch.Speed_pid.output)&0xff;
    HAL_CAN_AddTxMessage(&hcan2, &tx_header, CAN2_tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

void Set_GM6020_Gimbal_yaw_Voltage(CAN_HandleTypeDef* hcan,Moto_GM6020_t motor_yaw)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             CAN1_tx_data[8] = {0};
    
    tx_header.StdId = 0x1FF;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;
    CAN1_tx_data[4] = ((int16_t)motor_yaw.Speed_pid.output>>8)&0xff;
    CAN1_tx_data[5] = ((int16_t)motor_yaw.Speed_pid.output)&0xff;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, CAN1_tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

