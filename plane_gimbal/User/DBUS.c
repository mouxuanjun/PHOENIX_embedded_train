#include "DBUS.h"

/**
 * @brief 遥控器数据的读取
 * @param 要读取的信息的结构体
 */
void RemoteDataProcess(rc_info_t *rc_ctrl, uint8_t rx_data_Dbus[18]) {
    rc_ctrl->ch[0] = ((int16_t)rx_data_Dbus[0] | ((int16_t)rx_data_Dbus[1] << 8)) & 0x07FF;
    rc_ctrl->ch[0] -= 1024;
    rc_ctrl->ch[1] = ((int16_t)(rx_data_Dbus[2] >> 6) | 
                    (int16_t)(rx_data_Dbus[3] << 2) | 
                    (int16_t)(rx_data_Dbus[4] << 10)) & 0x07FF;
    rc_ctrl->ch[1] -= 1024;
    rc_ctrl -> s1 = ((int16_t)rx_data_Dbus[5] >> 4 & 0x0003);

    if ((abs(rc_ctrl->ch[0]) > 660) || (abs(rc_ctrl->ch[1]) > 660)) {
        memset(rc_ctrl, 0, sizeof(rc_info_t));
        return;
    }
}

/**
 * @brief 遥控器的控制
 * @param 要读取的信息的结构体和目标角度
 */
void control(rc_info_t *rc_ctrl, float * target_angle_form){
    *target_angle_form += ((float)rc_ctrl -> ch[1] / 10);
//	if (*target_angle_form > 8191) {
//		*target_angle_form = 0;
//	}else if (*target_angle_form < 0) {
//		*target_angle_form = 8191;
//	}
}

