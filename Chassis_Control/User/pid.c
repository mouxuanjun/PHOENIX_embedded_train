#include "pid.h"

/**
 * @brief 速度环的初始化
 * @param 速度环结构体
 */
void pid_velocity_init(pid_v * pid_velocity, float Kp, float Ki, float Kd, float MAX){
	pid_velocity -> error = 0;
	pid_velocity -> error_last = 0;
    pid_velocity -> Kp_v = Kp;
    pid_velocity -> Ki_v = Ki;
    pid_velocity -> Kd_v = Kd;
    pid_velocity -> MAX_I = MAX;
}

/**
 * @brief 速度环PID
 * @param 电机结构体、当前速度、目标速度
 * @return 速度环PID
 */
float pid_velocity_control(pid_v * pid_velocity, float current_speed_form, float target_speed_form){
    pid_velocity -> current_velocity = current_speed_form;
    pid_velocity -> target_velocity = target_speed_form;
    pid_velocity -> error_last = pid_velocity -> error;
    pid_velocity -> error = pid_velocity -> target_velocity - pid_velocity -> current_velocity;

    pid_velocity -> Kp_v_out = pid_velocity -> Kp_v * pid_velocity -> error;

	pid_velocity -> Ki_v_out += pid_velocity -> Ki_v * pid_velocity -> error;
    //积分限幅
	if (pid_velocity -> Ki_v_out > pid_velocity -> MAX_I){
        pid_velocity -> Ki_v_out = pid_velocity -> MAX_I;
    }
    if (pid_velocity -> Ki_v_out < -pid_velocity -> MAX_I){
        pid_velocity -> Ki_v_out = -pid_velocity -> MAX_I;
    }

    pid_velocity -> Kd_v_out = pid_velocity -> Kd_v * (pid_velocity -> error - pid_velocity -> error_last);

    pid_velocity -> PID_velocity_out = pid_velocity -> Kp_v_out
										+ pid_velocity -> Ki_v_out
                                        + pid_velocity -> Kd_v_out;
    return pid_velocity -> PID_velocity_out;
}
