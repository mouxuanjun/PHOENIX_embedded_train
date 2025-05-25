#include "GM3508.h"

/**
 * @brief GM3508������ݵĶ�ȡ
 * @param GM3508�ĵ��ID
 * @note ������ݵĽ���
 */
void GM3508_RxData(uint32_t StdId, uint8_t rx_data[8]){
	switch (StdId){
		case 0x201:{
			rc_3508[0].current_angle = (((uint16_t)(rx_data[0] << 8)) | ((uint16_t)rx_data[1]));
			rc_3508[0].real_angle = (((uint16_t)(rx_data[0] << 8)) | ((uint16_t)rx_data[1])) / 8191.0f * 360.0f;
			rc_3508[0].current_velocity = (((int16_t)(rx_data[2] << 8)) | ((int16_t)rx_data[3]));
			rc_3508[0].current_torque = (((int16_t)(rx_data[4] << 8)) | ((int16_t)rx_data[5]));
			break;
		}
		
		case 0x202:{
			rc_3508[1].current_angle = (((uint16_t)(rx_data[0] << 8)) | ((uint16_t)rx_data[1]));
			rc_3508[1].real_angle = (((uint16_t)(rx_data[0] << 8)) | ((uint16_t)rx_data[1])) / 8191.0f * 360.0f;
			rc_3508[1].current_velocity = (((int16_t)(rx_data[2] << 8)) | ((int16_t)rx_data[3]));
			rc_3508[1].current_torque = (((int16_t)(rx_data[4] << 8)) | ((int16_t)rx_data[5]));
		break;
		}
		case 0x203:{
			rc_3508[2].current_angle = (((uint16_t)(rx_data[0] << 8)) | ((uint16_t)rx_data[1]));
			rc_3508[2].real_angle = (((uint16_t)(rx_data[0] << 8)) | ((uint16_t)rx_data[1])) / 8191.0f * 360.0f;
			rc_3508[2].current_velocity = (((int16_t)(rx_data[2] << 8)) | ((int16_t)rx_data[3]));
			rc_3508[2].current_torque = (((int16_t)(rx_data[4] << 8)) | ((int16_t)rx_data[5]));
			break;
		}
		case 0x204:{
			rc_3508[3].current_angle = (((uint16_t)(rx_data[0] << 8)) | ((uint16_t)rx_data[1]));
			rc_3508[3].real_angle = (((uint16_t)(rx_data[0] << 8)) | ((uint16_t)rx_data[1])) / 8191.0f * 360.0f;
			rc_3508[3].current_velocity = (((int16_t)(rx_data[2] << 8)) | ((int16_t)rx_data[3]));
			rc_3508[3].current_torque = (((int16_t)(rx_data[4] << 8)) | ((int16_t)rx_data[5]));
			break;
		}
    }
}

void Chassis_task(void const * argument){
	float MAX, limit_form, init;
	pid_velocity_init(&rc_3508[0].GM3508_velocity, 0, 0, 0, MAX);
	pid_velocity_init(&rc_3508[0].GM3508_velocity, 0, 0, 0, MAX);
	pid_velocity_init(&rc_3508[0].GM3508_velocity, 0, 0, 0, MAX);
	pid_velocity_init(&rc_3508[0].GM3508_velocity, 0, 0, 0, MAX);
	while(1){
        pid_velocity_control(&rc_3508[0].GM3508_velocity, rc_3508[0].current_velocity, rc_3508[0].GM3508_velocity.target_velocity);
		pid_velocity_control(&rc_3508[1].GM3508_velocity, rc_3508[1].current_velocity, rc_3508[1].GM3508_velocity.target_velocity);
		pid_velocity_control(&rc_3508[2].GM3508_velocity, rc_3508[2].current_velocity, rc_3508[2].GM3508_velocity.target_velocity);
		pid_velocity_control(&rc_3508[3].GM3508_velocity, rc_3508[3].current_velocity, rc_3508[3].GM3508_velocity.target_velocity);
        GM3508_Control(rc_3508[0].GM3508_velocity.PID_velocity_out,
						rc_3508[1].GM3508_velocity.PID_velocity_out,
						rc_3508[2].GM3508_velocity.PID_velocity_out,
						rc_3508[3].GM3508_velocity.PID_velocity_out);   	
        osDelay(1);
	}
}
