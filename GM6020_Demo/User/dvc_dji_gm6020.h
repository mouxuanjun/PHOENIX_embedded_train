#ifndef __GM6020_H__
#define __GM6020_H__

#include "main.h"
#include "can.h"
#include "stm32f4xx.h"
#include <stdint.h>
#include <math.h>
#include <string.h>

// 定义π常数（如果系统没有定义的话）
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// GM6020编码器参数定义
#define ENCODER_MAX_VALUE 8191      // GM6020编码器最大值 (14位)
#define ENCODER_RANGE 8192          // 编码器范围 (2^14)
#define ENCODER_HALF_RANGE 4096     // 编码器范围的一半，用于判断回绕

// 电机ID定义
#define Motor_1_ID 0x207            // Yaw轴电机ID
#define Motor_2_ID 0x205            // Pitch轴电机ID

// CAN相关定义
#define CAN_CHASSIS_ALL_ID 0x1FF    // 标识符的ID
#define CHASSIS_CAN hcan1           // 主CAN总线
#define CHASSIS_CAN2 hcan2          // 备用CAN总线

/**
 * @brief GM6020电机数据结构体
 */
typedef struct
{
    uint16_t can_id;        // 电机ID
    int16_t  set_voltage;   // 设定的电压值
    float    Set_Speed;     // 设定速度
    float    Set_Angle;     // 设定角度

    uint16_t rotor_angle;   // 电机角度（编码器值：0-8191）
    int16_t  rotor_speed;   // 电机速度（rpm）
    int16_t  torque_current;// 电机转矩电流
    uint8_t  temp;          // 温度
    uint16_t test;          // 测试字段

} Moto_GM6020_t;

/**
 * @brief 编码器滤波器状态结构体
 * @note 用于保存滤波器的历史状态，支持编码器回绕处理
 */
typedef struct {
    float prev_angle;       // 上次有效角度
    float prev_speed;       // 上次有效速度  
    float prev_delta;       // 上次角度变化量
    uint32_t last_update;   // 最后更新时间戳
    uint8_t initialized;    // 初始化标志
} EncoderFilterState;

/**
 * @brief 滤波器参数配置结构体
 */
typedef struct {
    float max_delta_per_ms; // 每毫秒允许的最大变化量
    float max_accel;        // 最大加速度限制
    float max_speed_change; // 最大速度变化量
} FilterConfig;

// ========================= 函数声明 =========================

/**
 * @brief 计算编码器角度差值（处理回绕问题）
 * @param new_angle 新角度值
 * @param old_angle 旧角度值
 * @return 实际角度变化量（考虑回绕）
 */
float calculate_encoder_angle_diff(float new_angle, float old_angle);

/**
 * @brief 改进的编码器角度滤波函数
 * @param new_val 新的角度值
 * @param state 滤波器状态指针
 * @param max_delta_per_ms 每毫秒允许的最大变化量
 * @param max_accel 最大加速度限制
 * @return 滤波后的角度值
 * @note 专门处理GM6020编码器的0-8191回绕问题
 */
float encoder_angle_filter(float new_val, EncoderFilterState* state, 
                          float max_delta_per_ms, float max_accel);

/**
 * @brief 简单的速度滤波函数
 * @param new_val 新的速度值
 * @param state 滤波器状态指针
 * @param max_change 允许的最大变化量
 * @return 滤波后的速度值
 */
float speed_filter(float new_val, EncoderFilterState* state, float max_change);

/**
 * @brief GM6020电机消息接收处理函数
 * @param StdId 标准帧ID
 * @param rx_data 接收到的8字节数据
 * @note 支持编码器回绕处理和智能滤波
 */
void Get_GM6020_Motor_Message(uint32_t StdId, uint8_t rx_data[8]);

/**
 * @brief 发送GM6020电机控制消息
 * @param motor1 电机1控制值（通常对应0x201）
 * @param motor2 电机2控制值（通常对应0x202）
 * @param motor3 电机3控制值（通常对应0x203）
 * @param motor4 电机4控制值（通常对应0x204）
 * @note 发送到0x1FF标识符，控制电机ID 0x201-0x204
 */
void Send_GM6020_Motor_Message(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
 * @brief 设置GM6020云台电压（兼容原有接口）
 * @param hcan CAN句柄指针
 * @param GM6020_Yaw Yaw轴电机结构体
 * @param GM6020_Pitch Pitch轴电机结构体
 * @deprecated 建议使用Send_GM6020_Motor_Message函数
 */
void Set_GM6020_Gimbal_Voltage(CAN_HandleTypeDef* hcan, Moto_GM6020_t GM6020_Yaw, Moto_GM6020_t GM6020_Pitch);

/**
 * @brief 重置滤波器状态
 * @param motor_id 电机ID（0x207或0x205，0为重置所有）
 * @note 在电机重新启动或检测到异常时调用
 */
void Reset_Filter_State(uint32_t motor_id);

/**
 * @brief 角度单位转换：编码器值转换为角度（度）
 * @param encoder_value 编码器值（0-8191）
 * @return 角度值（度，0-360）
 */
static inline float Encoder_To_Degree(uint16_t encoder_value) {
    return (float)encoder_value * 360.0f / ENCODER_RANGE;
}

/**
 * @brief 角度单位转换：角度（度）转换为编码器值
 * @param degree 角度值（度）
 * @return 编码器值（0-8191）
 */
static inline uint16_t Degree_To_Encoder(float degree) {
    // 确保角度在0-360范围内
    while (degree < 0) degree += 360.0f;
    while (degree >= 360.0f) degree -= 360.0f;
    return (uint16_t)(degree * ENCODER_RANGE / 360.0f);
}

/**
 * @brief 角度单位转换：编码器值转换为弧度
 * @param encoder_value 编码器值（0-8191）
 * @return 角度值（弧度，0-2π）
 */
static inline float Encoder_To_Radian(uint16_t encoder_value) {
    return (float)encoder_value * 2.0f * M_PI / ENCODER_RANGE;
}

/**
 * @brief 检查编码器值是否有效
 * @param encoder_value 编码器值
 * @return 1: 有效, 0: 无效
 */
static inline int Is_Encoder_Valid(uint16_t encoder_value) {
    return (encoder_value <= ENCODER_MAX_VALUE) ? 1 : 0;
}

// ========================= 全局变量声明 =========================
extern volatile uint32_t temp3;           // 临时变量，用于调试
extern uint8_t chassis_can_send_data[8];   // CAN发送数据缓冲区

// ========================= 默认配置参数 =========================
// Yaw轴电机默认滤波参数
#define YAW_DEFAULT_MAX_DELTA_PER_MS    80.0f    // 80编码器单位/ms
#define YAW_DEFAULT_MAX_ACCEL           5000.0f  // 5000单位/s²
#define YAW_DEFAULT_MAX_SPEED_CHANGE    2000.0f  // 2000rpm

// Pitch轴电机默认滤波参数  
#define PITCH_DEFAULT_MAX_DELTA_PER_MS  60.0f    // 60编码器单位/ms
#define PITCH_DEFAULT_MAX_ACCEL         3000.0f  // 3000单位/s²
#define PITCH_DEFAULT_MAX_SPEED_CHANGE  1500.0f  // 1500rpm

#endif /* __GM6020_H__ */
