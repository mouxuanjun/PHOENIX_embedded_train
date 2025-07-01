#include "dr16.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"

void RemoteDataProcess(rc_info_t *rc_ctrl, uint8_t rx_data[18]){
    rc_ctrl -> rc.ch0 = (((int16_t)rx_data[0] | ((int16_t)rx_data[1] << 8)) & 0x07ff) - 1024;
    rc_ctrl -> rc.ch1  = (((int16_t)(rx_data[1] >> 3) | ((int16_t)rx_data[2] << 5)) & 0x07ff) - 1024;
    rc_ctrl -> rc.ch2 = (((int16_t)(rx_data[2] >> 6) | ((int16_t)rx_data[3] << 2) | ((int16_t)rx_data[4] << 10)) &0x07ff) - 1024;
    rc_ctrl -> rc.ch3 = (((int16_t)(rx_data[4] >> 1) | ((int16_t)rx_data[5] << 7)) & 0x07ff) - 1024;
    rc_ctrl -> rc.s1 = ((int16_t)(rx_data[5] >> 4) & 0x0003);
    rc_ctrl -> rc.s2 = ((int16_t)(rx_data[5] >> 4) & 0x000C) >> 2;
	  rc_ctrl -> rc.wheel = (rx_data[16] | (rx_data[17] << 8));
	
	  if ((abs(rc_ctrl -> rc.ch1) > 660) || (abs(rc_ctrl -> rc.ch2) > 660) || (abs(rc_ctrl -> rc.ch3) > 660)){
				memset(rc_ctrl, 0, 18);
				return ;
		}
		
		rc_ctrl -> mouse.mouse_x = (int16_t)rx_data[6] | ((int16_t)rx_data[7] << 8);
	  rc_ctrl -> mouse.mouse_y= (int16_t)rx_data[8] | ((int16_t)rx_data[9] << 8);
	  rc_ctrl -> mouse.mouse_press_l = (int16_t)rx_data[10];
	  rc_ctrl -> mouse.mouse_press_r = (int16_t)rx_data[11];

		*(uint16_t *)&rc_ctrl -> key = (uint16_t)(rx_data[12] | (rx_data[13] << 8));

	  rc_ctrl -> key.Null = (rx_data[14] | (rx_data[15] << 8));
}
