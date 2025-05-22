#include "FreeRTOS.h"
#include "Motor_control.h"
#include "BMI088driver.h"
IMU_Data_t imu;
void Motor_ControlTask(void const * argument)
{

    while(1)
    {
		BMI088_Read(&imu);
		VOFA_Tx();
		osDelay(1);
    }
}
