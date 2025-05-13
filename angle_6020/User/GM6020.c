#include "GM6020.h"

/**
 * @brief GM6020电机数据的读取
 * @param GM6020的电机ID
 * @note 电机数据的解算
 */
void GM6020_RxData(uint32_t StdId, uint8_t rx_data[8]){
	if (StdId == 0x208){
		rc_6020.current_angle = (((uint16_t)(rx_data[0] << 8)) | ((uint16_t)rx_data[1]));
		rc_6020.current_speed = (((int16_t)(rx_data[2] << 8)) | ((int16_t)rx_data[3]));
		rc_6020.current_Torque = (((int16_t)(rx_data[4] << 8)) | ((int16_t)rx_data[5]));
	}
}
