#include "GM6020.h"

void GM6020_RxData(uint32_t StdId, uint8_t rx_data[8]){
	if (StdId == 0x205){
		rc_6020.current_speed = ((rx_data[2] << 8) | rx_data[3]) * 100;
		rc_6020.current_Torque = ((rx_data[4] << 8) | rx_data[5]);
	}
}
