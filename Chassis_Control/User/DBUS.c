#include "DBUS.h"

void RemoteDataProcess(rc_info_t *rc_ctrl, uint8_t rx_data[18]){
    rc_ctrl -> ch[0] = ((int16_t)rx_data[0]) | ((int16_t)rx_data[1] << 8) & 0x07ff;
	rc_ctrl -> ch[0] -= 1024;
    rc_ctrl -> ch[1] = ((int16_t)rx_data[1] >> 3) | ((int16_t)rx_data[2] << 5) & 0x07ff;
	rc_ctrl -> ch[1] -=1024;
    rc_ctrl -> ch[2] = ((int16_t)rx_data[2] >> 6) | ((int16_t)rx_data[3] << 2) | ((int16_t)rx_data[4] << 10) & 0x07ff;
	rc_ctrl -> ch[2] -= 1024;
    rc_ctrl -> ch[3] = ((int16_t)rx_data[4] >> 1) | ((int16_t)rx_data[5] << 7) & 0x07ff;
	rc_ctrl -> ch[3] -= 1024;
	rc_ctrl -> s1 = ((int16_t)rx_data[5] >> 4 & 0x0003);
    rc_ctrl -> s2 = ((int16_t)rx_data[5] >> 4 & 0x000C) >> 2;
	rc_ctrl -> wheel = rx_data[16] | (rx_data[17] << 8) - 1024;
	
	if ((abs(rc_ctrl -> ch[0]) > 660) || (abs(rc_ctrl -> ch[1]) > 660) || (abs(rc_ctrl -> ch[2]) > 660) || (abs(rc_ctrl -> ch[3]) > 660)){
		memset(rc_ctrl, 0, 18);
		return ;
	} 
}

void Control(void){
    switch(Chassis_way){
		case Chassis_follow_gimbal:
			chassis.vx =(float)rc_ctrl.ch[3] / 300;
			chassis.vy =(float)rc_ctrl.ch[4] / 300;
			chassis.vw=-Chassis_Follow_PID.f_cal_pid(&Chassis_Follow_PID,Find_Y_AnglePNY(),0);
			break;
		case Chassis_normal:
			chassis.vx=(float)rc_ctrl.ch[3] / 300;
			chassis.vy=(float)rc_ctrl.ch[4] / 300;
			chassis.vw=0;
			break;
		case Chassis_gyrescope:
			chassis.vx=(float)rc_ctrl.ch[3] / 300;
			chassis.vy=(float)rc_ctrl.ch[4] / 300;
			chassis.vw=-2;
			break;
		case Chassis_stop:
			chassis.vx=0;
			chassis.vy=0;
			chassis.vw=0;
			break;
		default:
			break;
	}
}
