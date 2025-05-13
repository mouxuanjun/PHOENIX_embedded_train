#include "pid.h"

/**
 * @brief GM6020电机速度环的初始化
 * @param GM6020的速度环结构体
 */
void pid_velocity_init(pid_v * pid_velocity, float Kp, float Ki, float Kd, int16_t MAX){
	pid_velocity -> error = 0;
	pid_velocity -> error_last = 0;
    pid_velocity -> Kp_v = Kp;
    pid_velocity -> Ki_v = Ki;
    pid_velocity -> Kd_v = Kd;
    pid_velocity -> MAX_I = MAX;
}

/**
 * @brief GM6020电机角度环的初始化
 * @param GM6020的角度环的结构体
 */
void pid_angle_init(pid_a * pid_angle, float Kp, float Ki, float Kd, int16_t MAX){
    pid_angle -> error = 0;
    pid_angle -> error_last = 0;
    pid_angle -> Kp_a = Kp;
    pid_angle -> Ki_a = Ki;
    pid_angle -> Kd_a = Kd;
    pid_angle -> MAX_I = MAX;
}

/**
 * @brief 速度环PID
 * @param 电机结构体、当前速度、上一次速度，积分限幅的最大值
 * @return 速度环PID
 */
float pid_velocity_control(pid_v * pid_velocity, float current_speed_form, float target_speed_form){
    pid_velocity -> current_speed = current_speed_form;
    pid_velocity -> target_speed = target_speed_form;
    pid_velocity -> error_last = pid_velocity -> error;
    pid_velocity -> error = pid_velocity -> target_speed - pid_velocity -> current_speed;

    pid_velocity -> pid_Kp = pid_velocity -> Kp_v * pid_velocity -> error;
	
	pid_velocity -> pid_Ki += pid_velocity -> Ki_v * pid_velocity -> error;
    //积分限幅
	if (pid_velocity -> pid_Ki > pid_velocity -> MAX_I){
        pid_velocity -> pid_Ki = pid_velocity -> MAX_I;
    }
    if (pid_velocity -> pid_Ki < -pid_velocity -> MAX_I){
        pid_velocity -> pid_Ki = -pid_velocity -> MAX_I;
    }

    pid_velocity -> pid_Kd = pid_velocity -> Kd_v * (pid_velocity -> error - pid_velocity -> error_last);

    pid_velocity -> PID_velocity_out = pid_velocity -> pid_Kp
										+ pid_velocity -> pid_Ki
                                        + pid_velocity -> pid_Kd;

    return pid_velocity -> PID_velocity_out;
}

/**
 * @brief 角度环PID
 * @param 电机结构体、当前电机角度、最大值
 * @return 角度环PID
 */
float pid_angle_control(pid_a * pid_angle, float current_angle_form, float target_angle_form){
    pid_angle -> current_angle = current_angle_form;
    pid_angle -> target_angle = target_angle_form;	
    pid_angle -> error_last = pid_angle -> error;	
    pid_angle -> error = pid_angle -> target_angle - pid_angle -> current_angle;	
	//过零保护
	if (target_angle_form - current_angle_form >= 4096) {
		pid_angle -> error -= 8192;
	}
	else if(target_angle_form - current_angle_form <= -4096)
	{
		pid_angle -> error += 8192;
	}

    pid_angle -> pid_Kp = pid_angle -> Kp_a * pid_angle -> error;

    pid_angle -> pid_Ki += pid_angle -> Ki_a * pid_angle -> error;
    //积分限幅
    if (pid_angle -> pid_Ki > pid_angle -> MAX_I){
        pid_angle -> pid_Ki = pid_angle -> MAX_I;
    }
    if (pid_angle -> pid_Ki < -pid_angle -> MAX_I){
        pid_angle -> pid_Ki = -pid_angle -> MAX_I;
    }
	
    pid_angle -> pid_Kd = pid_angle -> Kd_a * (pid_angle -> error - pid_angle -> error_last);

    pid_angle -> PID_angle_out = pid_angle -> pid_Kp
                                + pid_angle -> pid_Ki
                                + pid_angle -> pid_Kd;
    //输出限幅
    if (pid_angle -> PID_angle_out > 320){
        pid_angle -> PID_angle_out = 320;
    }
    if (pid_angle -> PID_angle_out < -320){
        pid_angle -> PID_angle_out = -320;
    }

    return pid_angle -> PID_angle_out;
}
