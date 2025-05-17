#include "BMI088.h"

void BMI088_task(void const * argument){
	int cnt = 0;
    while (1){
	if (cnt == 50){
		BMI088_read(gyro, accel, &temp);
		cnt = 0;
	}else{
		cnt++;
	}
	}
}
