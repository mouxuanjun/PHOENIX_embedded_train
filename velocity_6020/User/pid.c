#include "pid.h"
#include "GM6020.h"

void pid_init(){
		pid.error = 0;
		pid.error_difference = 0;
		pid.error_last = 0;
		pid.error_out = 0;
		pid.PID_out = 0;
		pid.sum_error = 0;
		pid.target_speed = 0;
		pid.Kp = 1.3152;
		pid.Ki = 0.21112;
		pid.Kd = 0.301;
}

int16_t pid_control(pid_t * pid, rc_t * rc_6020){
    pid -> error = pid -> target_speed - rc_6020 -> current_speed;
	  
    //一阶低通滤波
    pid -> error_out = 0.3 * pid -> error + 0.7 * pid -> error_last;
    pid -> error_last = pid -> error_out;
	
    pid -> sum_error += pid -> error_out;
	
    pid -> error_difference = pid -> error - pid -> error_last;

	  //输出限幅
		if (pid -> sum_error > 5000){
        pid -> sum_error = 5000;
    }
    if (pid -> sum_error < -5000){
        pid -> sum_error = -5000;
    } 
		
    pid -> PID_out = pid -> Kp * pid -> error_out
                  + pid -> Ki * pid -> sum_error
                  + pid -> Kd * pid -> error_difference;

    return pid -> PID_out;
}
