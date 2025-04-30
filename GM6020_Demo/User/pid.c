#include "freertos.h"
#include "pid.h"
#include "dvc_dji_gm6020.h"
#include "timers.h"
#include "queue.h"

#define ENCODER_MAX   8191  // 2^13 - 1
#define ENCODER_RANGE 8192  // 2^13
#define HALF_RANGE    4096  // 过零判断阈值

//电压给定值范围：-25000~0~25000。不要在debug的时候突然退出debug！！！！！！！！
PID PosePID={100,0,0};

PID VelPID={1,0,0};


/**
 * @file pid.c
 * @brief 
 * @author CGH
 * @date 2025/4/28
 */
extern QueueHandle_t Usb_queueHandle;
float error;
float dt1=0.05,dt2=0.01;//dt1表示角度环的运行周期，dt2表示速度环的运行周期


float wrap_error(float target, float measured) {
    float error = target - measured;
    
    // 过零修正
    if (error > HALF_RANGE) {
        error -= ENCODER_RANGE;
    } else if (error < -HALF_RANGE) {
        error += ENCODER_RANGE;
    }
    
    return error;
}

float position_PID(float target, float current) {
	  uint32_t past;
		uint32_t now = xTaskGetTickCount();
		dt1=(now-past)*(1.0 / configTICK_RATE_HZ);
	  past=now;
    static float integral = 0;
    static float last_error = 0;
    // 角度解算
    float current_angle = current * (360.0f / ENCODER_MAX); // 转换为角度值
    
    // 过零处理
    float error = wrap_error(target, current_angle);
   // error = target - current;
    integral += error * dt1;
    float derivative = (error - last_error) / dt1;
    
    last_error = error;
	//死区，看情况启用if(error<=0.01){
		//error=0;
		//}
	  float result = PosePID.P*error + PosePID.I*integral + PosePID.D*derivative;
		
    return result;
}


float velocity_PID(float target, float current) {
	  uint32_t Vpast;
	  uint32_t now = xTaskGetTickCount();
		dt2=(now-Vpast)*(1.0 / configTICK_RATE_HZ);
	  Vpast=now;
    static float vintegral = 0;
    static float vlast_error = 0;
    
    error = target - current;
    vintegral += error * dt2;
    float vderivative = (error - vlast_error) / dt2;
    
    vlast_error = error;
    float vresult = VelPID.P*error + VelPID.I*vintegral + VelPID.D*vderivative;
    return vresult;
}

/*
float PID_Control(float target,float pos_current,float vel_current){
	float temp_result1=position_PID(target,pos_current);
	float temp_result2=velocity_PID(temp_result1,vel_current);
	xQueueSendFromISR(Usb_quene, &temp_result1, NULL);
	
}*/

