#include "math.h"
#include "freertos.h"
#include "pid.h"
#include "dvc_dji_gm6020.h"
#include "timers.h"
#include "queue.h"

/**
 * @file pid.c
 * @brief 
 * @author CGH
 * @date 2025/4/28
 */
 




// --- 正弦波参数  --
#define M_PI 3.14159265358979323846f
#define SINE_AMPLITUDE     1000.0f // 幅度 (编码器单位)
#define SINE_FREQUENCY_HZ  0.5f    // 频率 (Hz)
#define SINE_CENTER_POINT  4096.0f // 中心点 (编码器单位, 8192量程)
#define TWO_PI             (2.0f * (float)M_PI) // 圆周率的两倍
//GM6020参数
#define ENCODER_MAX   8191  // 2^13 - 1
#define ENCODER_RANGE 8192  // 2^13
#define HALF_RANGE    4096  // 过零判断阈值
#define INTERGEL_MAX  15000
#define INTERGEL_MIN  -15000
//电压给定值范围：-25000~0~25000。不要在debug的时候突然退出debug！！！！！！！！

//P,I,D,F
//PID PosePID={0.054,0,0.0428,0};
PID PosePID_yaw={.P=0.0758,.I=0,.D=0.079,.F=0,.integral=0,.last_error=0,.derivative=0};

PID PosePID_pitch={.P=0.0758,.I=0,.D=0.079,.F=0,.integral=0,.last_error=0,.derivative=0};
//PID VelPID={74,565,0};
PID VelPID_yaw={.P=63,.I=521,.D=0,.F=0,.integral=0,.last_error=0,.derivative=0};
PID VelPID_pitch={.P=63,.I=521,.D=0,.F=0,.integral=0,.last_error=0,.derivative=0};



// 正弦波发生器状态变量
static float sine_gen_current_y = 0.0f; // 当前目标值 (相对于中心点)
static float sine_gen_current_v = 0.0f; // 当前目标值的变化率 (速度/斜率)
static uint32_t sine_gen_last_tick = 0; // 上次调用发生器时的 Tick Count
// 预计算常量
static float sine_gen_omega = 0.0f;
static float sine_gen_omega_squared = 0.0f;
uint8_t begin=1;

//前馈
float feedforward=0;
void init_sine_generator(float initial_phase_rad); // 正弦生成器初始化函数原型
float generate_sine_target(void);                  // 正弦波发生器函数原型
/* USER CODE END FunctionPrototypes */






extern QueueHandle_t Usb_queueHandle;

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

float position_PID(float target, float current, PID *PosePID) {  // 使用指针传递
    static uint32_t past;
    uint32_t now = xTaskGetTickCount();
    float dt = (now - past) * (1.0 / configTICK_RATE_HZ);  // 取消注释，计算时间差
    past = now;
    
    // 计算前馈项
    float feedforward = (target - PosePID->last_target);  // 除以时间得到变化率
    PosePID->last_target = target;  // 更新目标值，但不要重置为0
    
    // 过零处理
    float error = wrap_error(target, current);
    if(begin==1){
			PosePID->last_error=error;
			begin=0;
		}
    // 更新积分项，但不要重置为0
    PosePID->integral += error ;  // 乘以时间差
    
    // 积分限幅
    if(PosePID->integral < INTERGEL_MIN) PosePID->integral = INTERGEL_MIN;
    if(PosePID->integral > INTERGEL_MAX) PosePID->integral = INTERGEL_MAX;
    
    // 计算微分项，不要重置last_error为0
    PosePID->derivative = (error - PosePID->last_error) ;  // 除以时间差
    PosePID->last_error = error;  // 更新上次误差
    
    // 死区处理
    // if(fabs(error) <= 0.01) error = 0;
    
    // 计算最终输出（包含前馈项）
    float result = PosePID->P * error + 
                   PosePID->I * PosePID->integral + 
                   PosePID->D * PosePID->derivative + 
                   PosePID->F * feedforward;  // 添加前馈项
    
    return result;
}

float velocity_PID(float target, float current, PID *VelPID) {  // 使用指针传递
    static uint32_t Vpast;
    uint32_t now = xTaskGetTickCount();
    float dt2 = (now - Vpast) * (1.0 / configTICK_RATE_HZ);
    Vpast = now;
    
    // 不要重置积分和上次误差
    // VelPID.integral = 0;  // 删除这行
    // VelPID.last_error = 0;  // 删除这行
    
    float error = target - current;
    if(begin==1){
			VelPID->last_error=error;
			begin=0;
		}
    // 死区处理
    if(fabs(error) <= 0.01) {
        error = 0;
    }
    
    // 积分项计算
    VelPID->integral += error * dt2;
    
    // 积分限幅
    if(VelPID->integral < INTERGEL_MIN) VelPID->integral = INTERGEL_MIN;
    if(VelPID->integral > INTERGEL_MAX) VelPID->integral = INTERGEL_MAX;
    
    // 微分项计算
    VelPID->derivative = (error - VelPID->last_error) / dt2;
    
    // 更新上次误差
    VelPID->last_error = error;
    
    // 计算前馈项（需要在外部定义或传入feedforward）
    float feedforward = 0.1*(error-VelPID->last_error); // 这里应该定义前馈项或从外部传入
    
    // 计算最终输出
    float vresult = VelPID->P * error + 
                    VelPID->I * VelPID->integral + 
                    VelPID->D * VelPID->derivative;
    
    return vresult + (feedforward * VelPID->F);
}


float velocity_PID_incre(float target, float current,PID VelPID) {
	static uint32_t Vpast;
	  uint32_t now = xTaskGetTickCount();
		dt2=(now-Vpast)*(1.0 / configTICK_RATE_HZ);
	  Vpast=now;
    static float vintegral = 0;
    static float vlast_error = 0;
	  //float error=0;
    //float past_error=error;
    float error = target - current;
	  if(fabs(error)<=0.01){
		  error=0;
		}
    vintegral += error * dt2;
		//积分限幅
	  vintegral=(vintegral<INTERGEL_MIN)?INTERGEL_MIN:vintegral;
	  vintegral=(vintegral>INTERGEL_MAX)?INTERGEL_MAX:vintegral;
    float vderivative = (error - vlast_error) / dt2;
    
    vlast_error = error;
    float vresult = VelPID.P*error + VelPID.I*vintegral + VelPID.D*vderivative;
    return vresult;
}



/**
 * @brief 初始化正弦波发生器的状态变量和常量
 * @param initial_phase_rad 起始相位 (弧度)
 */
void init_sine_generator(float initial_phase_rad) {
    sine_gen_omega = TWO_PI * SINE_FREQUENCY_HZ;
    sine_gen_omega_squared = sine_gen_omega * sine_gen_omega;

    // 根据初始相位设置初始状态
    // y(0) = A * sin(phi)
    // v(0) = A * omega * cos(phi)
    sine_gen_current_y = SINE_AMPLITUDE * sinf(initial_phase_rad);
    sine_gen_current_v = SINE_AMPLITUDE * sine_gen_omega * cosf(initial_phase_rad);

    // 记录初始的 Tick Count，用于第一次计算 dt
    sine_gen_last_tick = xTaskGetTickCount();
}

/**
 * @brief 使用迭代法生成下一个正弦波目标位置，并动态计算dt
 * @retval float 包含中心点偏移的最终目标位置
 */
float generate_sine_target(void) {
    float dt = 0.0f;
    uint32_t now = xTaskGetTickCount();

   
    // 防止第一次计算 dt 时 sine_gen_last_tick 为 0 导致 dt 过大
    if (sine_gen_last_tick != 0) {
        // 计算距离上次调用的时间差 (单位：秒)
        dt = (float)(now - sine_gen_last_tick) * (1.0f / configTICK_RATE_HZ);
    } else {
        // 第一次调用，给 dt 一个默认的小值
        dt = 0.001f;
    }
    // 更新 last_tick 
    sine_gen_last_tick = now;

    //防止任务挂起导致的dt差异
    if (dt > 0.1f) dt = 0.001f; // 限制最大 dt，例如 100ms
    if (dt <= 0.0f) dt = 0.001f; // 防止 dt 为 0 或负数

    
    // 预计算以提高效率
    float omega_squared_dt = sine_gen_omega_squared * dt;
    float v_dt = sine_gen_current_v * dt;

  
    sine_gen_current_v = sine_gen_current_v - omega_squared_dt * sine_gen_current_y;
    sine_gen_current_y = sine_gen_current_y + v_dt;


    
    float target_position = SINE_CENTER_POINT + sine_gen_current_y;

    return target_position;
}





float limit(float value,float MAX,float MIN){
	value=(value<MIN)?MIN:value;
	value=(value>MAX)?MAX:value;
}
