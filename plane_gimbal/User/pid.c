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
 * @brief 角度环的初始化
 * @param 角度环的结构体
 */
void pid_angle_init(pid_a * pid_angle, float Kp, float Ki, float Kd, float MAX, float limit_form, float init){
    pid_angle -> error = 0;
    pid_angle -> error_last = 0;
    pid_angle -> target_angle = pid_angle -> current_angle;
    pid_angle -> Kp_a = Kp;
    pid_angle -> Ki_a = Ki;
    pid_angle -> Kd_a = Kd;
    pid_angle -> limit = limit_form;
    pid_angle -> MAX_I = MAX;
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

/**
 * @brief 角度环PID
 * @param 电机结构体、当前电机角度、目标电机角度
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

    pid_angle -> Kp_a_out = pid_angle -> Kp_a * pid_angle -> error;

    pid_angle -> Ki_a_out += pid_angle -> Ki_a * pid_angle -> error;
    //积分限幅
    if (pid_angle -> Ki_a_out > pid_angle -> MAX_I){
        pid_angle -> Ki_a_out = pid_angle -> MAX_I;
    }
    if (pid_angle -> Ki_a_out < -pid_angle -> MAX_I){
        pid_angle -> Ki_a_out = -pid_angle -> MAX_I;
    }

    pid_angle -> Kd_a_out = pid_angle -> Kd_a * (pid_angle -> error - pid_angle -> error_last);
    pid_angle -> PID_angle_out = pid_angle -> Kp_a_out
                                + pid_angle -> Ki_a_out
                                + pid_angle -> Kd_a_out;
    //输出限幅
    if (pid_angle -> PID_angle_out > pid_angle -> limit){
        pid_angle -> PID_angle_out = pid_angle -> limit;
    }
    if (pid_angle -> PID_angle_out < -pid_angle -> limit){
        pid_angle -> PID_angle_out = -pid_angle -> limit;
    }

    return pid_angle -> PID_angle_out;
}

void Limit(pid_a * pid_angle, float H, float L){
	if (pid_angle -> target_angle > H){
		pid_angle -> target_angle = H;
	}
	if (pid_angle -> target_angle < L){
		pid_angle -> target_angle = L;
	}
}
