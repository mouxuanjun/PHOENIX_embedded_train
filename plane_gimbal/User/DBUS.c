#include "DBUS.h"

/**
 * @brief 遥控器数据的读取
 * @param 要读取的信息的结构体
 */
void RemoteDataProcess(rc_info_t *rc_ctrl, uint8_t rx_data_Dbus[18]) {
    rc_ctrl -> ch[0] = ((int16_t)rx_data_Dbus[0]) | ((int16_t)rx_data_Dbus[1] << 8) & 0x07ff;
	rc_ctrl -> ch[0] -= 1024;
    rc_ctrl -> ch[1] = ((int16_t)rx_data_Dbus[1] >> 3) | ((int16_t)rx_data_Dbus[2] << 5) & 0x07ff;
	rc_ctrl -> ch[1] -=1024;
    rc_ctrl -> ch[2] = ((int16_t)rx_data_Dbus[2] >> 6) | ((int16_t)rx_data_Dbus[3] << 2) | ((int16_t)rx_data_Dbus[4] << 10) & 0x07ff;
	rc_ctrl -> ch[2] -= 1024;
    rc_ctrl -> ch[3] = ((int16_t)rx_data_Dbus[4] >> 1) | ((int16_t)rx_data_Dbus[5] << 7) & 0x07ff;
	rc_ctrl -> ch[3] -= 1024;
	rc_ctrl -> s1 = ((int16_t)rx_data_Dbus[5] >> 4 & 0x0003);
    rc_ctrl -> s2 = ((int16_t)rx_data_Dbus[5] >> 4 & 0x000C) >> 2;
	rc_ctrl -> wheel = rx_data_Dbus[16] | (rx_data_Dbus[17] << 8) - 1024;

    if ((abs(rc_ctrl->ch[0]) > 660) || (abs(rc_ctrl->ch[1]) > 660) || (abs(rc_ctrl->ch[2]) > 660) || (abs(rc_ctrl->ch[3]) > 660)) {
        memset(rc_ctrl, 0, sizeof(rc_info_t));
        return;
    }
}

/**
 * @brief 遥控器的控制
 * @param 要读取的信息的结构体和目标角度
 */
void control(rc_info_t *rc_ctrl, float * target_angle_form, uint32_t Id){
    switch (Id){
		case 0x208:
			*target_angle_form += ((float)rc_ctrl -> ch[0] / 500);
//			if (*target_angle_form > 8191) {
//				*target_angle_form = 0;
//			}else if (*target_angle_form < 0) {
//				*target_angle_form = 8191;
//			}
			break;
		case 0x01:
			*target_angle_form += (((float)rc_ctrl -> ch[1] / 8191 * 3.15f) / -500);
			break;
	}
}

