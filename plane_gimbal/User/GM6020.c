#include "GM6020.h"

/**
 * @brief GM6020电机数据的读取
 * @param GM6020的电机ID
 * @note 电机数据的解算
 */
void GM6020_RxData(uint32_t StdId, uint8_t rx_data[8]){
	if (StdId == 0x208){
		rc_6020.current_angle = (((uint16_t)(rx_data[0] << 8)) | ((uint16_t)rx_data[1]));
		rc_6020.current_velocity = (((int16_t)(rx_data[2] << 8)) | ((int16_t)rx_data[3]));
		rc_6020.current_torque = (((int16_t)(rx_data[4] << 8)) | ((int16_t)rx_data[5]));
	}
}

/**
 * @brief GM6020电机pid的调用
 * @param GM6020的电机的当前和目标的速度和角度
 */
void GM6020_task(void const * argument){
    pid_velocity_init(&rc_6020.GM6020_velocity, 160, 2, 0, 20000);                                       //GM6020速度环初始化
	pid_angle_init(&rc_6020.GM6020_angle, 1.2, 0.001, 0, 20000, 320, 8192);                              //GM6020角度环初始化
    while (1) {
		control(&rc_ctrl, &rc_6020.GM6020_angle.target_angle, 0x208);
        Limit(&rc_6020.GM6020_angle, 7300, 4300);
        pid_angle_control(&rc_6020.GM6020_angle, rc_6020.current_angle, rc_6020.GM6020_angle.target_angle);
        rc_6020.GM6020_velocity.target_velocity = rc_6020.GM6020_angle.PID_angle_out;
        pid_velocity_control(&rc_6020.GM6020_velocity, rc_6020.current_velocity, rc_6020.GM6020_velocity.target_velocity);
        GM6020_Control(rc_6020.GM6020_velocity.PID_velocity_out);        
        osDelay(1);
    }
}
