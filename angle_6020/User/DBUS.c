#include "DBUS.h"

void RemoteDataProcess(rc_info_t *rc_ctrl, uint8_t rx_data[18]) {
    rc_ctrl->ch[0] = ((int16_t)rx_data[0] | ((int16_t)rx_data[1] << 8)) & 0x07FF;
    rc_ctrl->ch[0] -= 1024;
    rc_ctrl->ch[1] = ((int16_t)(rx_data[2] >> 6) | 
                      (int16_t)(rx_data[3] << 2) | 
                      (int16_t)(rx_data[4] << 10)) & 0x07FF;
    rc_ctrl->ch[1] -= 1024;

    if ((abs(rc_ctrl->ch[0]) > 660) || (abs(rc_ctrl->ch[1]) > 660)) {
        memset(rc_ctrl, 0, sizeof(rc_info_t));
        return;
    }
}

void control(rc_info_t *rc_ctrl, float * target_angle_form){
    *target_angle_form += ((float)rc_ctrl -> ch[1] / 10);
	if (*target_angle_form > 8191) {
		*target_angle_form = 0;
	}else if (*target_angle_form < 0) {
		*target_angle_form = 8191;
	}
}
