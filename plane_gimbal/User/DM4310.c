#include "DM4310.h"

/**
 * @brief uint转float
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief float转uint
 */
int float_to_uint(float x, float x_min, float x_max, int bits){
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief 对DM4310电机失能和使能的控制
 */
void if_enable(void){
    uint8_t en_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint8_t dis_data[8] = {0XFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}; 
    int cnt = 0;
    if (rc_ctrl.s1 == 1) {
        while (!rc_4310.error_code){
            CAN_TxHeaderTypeDef tx_header;

            tx_header.StdId = 0x01;
            tx_header.IDE = CAN_ID_STD;
            tx_header.RTR = CAN_RTR_DATA;
            tx_header.DLC = 8; 

	        HAL_CAN_AddTxMessage(&hcan1, &tx_header, en_data, (uint32_t *)CAN_TX_MAILBOX0);  
            cnt++;
            if (cnt >= 10){
                break;
            }
        }
	}
    if (rc_ctrl.s1 == 2) {
        while (rc_4310.error_code){
            CAN_TxHeaderTypeDef tx_header;

            tx_header.StdId = 0x01;
            tx_header.IDE = CAN_ID_STD;
            tx_header.RTR = CAN_RTR_DATA;
            tx_header.DLC = 8; 

	        HAL_CAN_AddTxMessage(&hcan1, &tx_header, dis_data, (uint32_t *)CAN_TX_MAILBOX0);  
            cnt++;
            if (cnt >= 10){
                break;
            }
        }
    }
}

/**
 * @brief DM4310电机数据的读取
 * @param DM4310的电机ID
 * @note 电机数据的解算
 */
void DM4310_RxData(uint32_t StdId, uint8_t rx_data[8]){
    if (StdId == 0x11) {
        rc_4310.error_code = rx_data[0] >> 4;
        rc_4310.current_angle = (rx_data[1] << 8) | rx_data[2];
        rc_4310.current_velocity = (rx_data[3] << 4) | (rx_data[4] >> 4);
        rc_4310.current_torque = ((rx_data[4] & 0xF) << 8) | rx_data[5];
        rc_4310.current_angle = uint_to_float(rc_4310.current_angle, P_MIN, P_MAX, 16);
        rc_4310.current_velocity = uint_to_float(rc_4310.current_velocity, V_MIN, V_MAX, 12);
        rc_4310.current_torque = uint_to_float(rc_4310.current_torque, T_MIN, T_MAX, 12);
    }
}

/**
 * @brief DM4310电机pid的调用
 * @param DM4310电机的当前和目标的速度和角度
 */
void DM4310_task(void const * argument){
    while (1) {
        if_enable();
//		control(&rc_ctrl, &rc_4310.DM4310_angle.target_angle);
        Limit(&rc_4310.DM4310_angle, 0.5, -0.4);
        pid_angle_control(&rc_4310.DM4310_angle, rc_4310.current_angle, rc_4310.DM4310_angle.target_angle);
        rc_4310.DM4310_velocity.target_velocity = rc_4310.DM4310_angle.PID_angle_out;
        pid_velocity_control(&rc_4310.DM4310_velocity, rc_4310.current_velocity, rc_4310.DM4310_velocity.target_velocity);
        DM4310_Control(0x01, 0, 0, 0, 0, rc_4310.DM4310_velocity.PID_velocity_out);
        osDelay(1);
    }
}
