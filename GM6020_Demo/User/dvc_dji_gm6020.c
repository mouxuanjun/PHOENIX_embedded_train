#include "dvc_dji_gm6020.h"
#include "dri_can.h"
#include <math.h>
#include <string.h>

HAL_StatusTypeDef can_1;
extern Moto_GM6020_t GM6020;
extern Moto_GM6020_t GM6020_pitch;

uint8_t chassis_can_send_data[8];
volatile uint32_t temp3;

// GM6020编码器参数
#define ENCODER_MAX_VALUE 8191
#define ENCODER_RANGE 8192
#define ENCODER_HALF_RANGE 4096

// 滤波器状态结构体（简化版）
typedef struct {
    float last_valid_angle;    // 上次有效角度值
    float last_valid_speed;    // 上次有效速度值
    uint8_t initialized;       // 初始化标志
} SimpleFilterState;

// 全局滤波器状态
static SimpleFilterState motor_207_filter = {0};
static SimpleFilterState motor_205_filter = {0};

/**
 * @brief 计算考虑回绕的真实角度变化
 * @param new_angle 新角度值
 * @param old_angle 旧角度值
 * @return 真实的角度变化量
 */
float get_real_angle_change(float new_angle, float old_angle) {
    float diff = new_angle - old_angle;
    
    // 处理回绕情况，获取真实的小变化量
    if (diff > ENCODER_HALF_RANGE) {
        return diff - ENCODER_RANGE;  // 正向回绕：8191→0 实际是+1
    } else if (diff < -ENCODER_HALF_RANGE) {
        return diff + ENCODER_RANGE;  // 反向回绕：0→8191 实际是-1  
    }
    
    return diff;  // 正常变化，无回绕
}

/**
 * @brief 简单直接的角度滤波器 - 丢弃异常斜率
 * @param new_angle 新角度值
 * @param filter_state 滤波器状态
 * @param max_change_threshold 最大允许变化阈值
 * @return 有效的角度值（异常直接用上次值）
 */
float simple_angle_filter(float new_angle, SimpleFilterState* filter_state, float max_change_threshold) {
    // 首次初始化
    if (!filter_state->initialized) {
        filter_state->last_valid_angle = new_angle;
        filter_state->initialized = 1;
        return new_angle;
    }
    
    // 计算真实变化量（考虑回绕）
    float real_change = get_real_angle_change(new_angle, filter_state->last_valid_angle);
    
    // 判断变化是否过大
    if (fabsf(real_change) > max_change_threshold) {
        // **直接丢弃异常数据，返回上次有效值**
        return filter_state->last_valid_angle;
    }
    
    // 数据正常，更新并返回
    filter_state->last_valid_angle = new_angle;
    return new_angle;
}

/**
 * @brief 简单直接的速度滤波器 - 丢弃异常变化
 * @param new_speed 新速度值
 * @param filter_state 滤波器状态
 * @param max_change_threshold 最大允许变化阈值
 * @return 有效的速度值
 */
float simple_speed_filter(float new_speed, SimpleFilterState* filter_state, float max_change_threshold) {
    // 首次初始化
    if (!filter_state->initialized) {
        filter_state->last_valid_speed = new_speed;
        return new_speed;
    }
    
    // 计算速度变化
    float speed_change = fabsf(new_speed - filter_state->last_valid_speed);
    
    // 判断变化是否过大
    if (speed_change > max_change_threshold) {
        // **直接丢弃异常数据，返回上次有效值**
        return filter_state->last_valid_speed;
    }
    
    // 数据正常，更新并返回
    filter_state->last_valid_speed = new_speed;
    return new_speed;
}

/**
 * @brief GM6020电机消息接收处理函数
 */
void Get_GM6020_Motor_Message(uint32_t StdId, uint8_t rx_data[8])
{
    temp3 = StdId;
    
    switch(StdId)
    {
        case 0x207: // Yaw轴电机
        {
            // 解析原始数据
            float raw_angle = (float)((rx_data[0] << 8) | rx_data[1]);
            float raw_speed = (float)((int16_t)((rx_data[2] << 8) | rx_data[3]));
            
            // 简单直接的滤波：异常数据直接丢弃
            GM6020.rotor_angle = (uint16_t)simple_angle_filter(
                raw_angle, 
                &motor_207_filter, 
                2000.0f    // Yaw轴：允许最大100个编码器单位的变化
            );
            
            GM6020.rotor_speed = (int16_t)simple_speed_filter(
                raw_speed, 
                &motor_207_filter, 
                160.0f   // Yaw轴：允许最大1000rpm的速度变化
            );
            
            // 其他数据直接赋值
            GM6020.torque_current = (int16_t)((rx_data[4] << 8) | rx_data[5]);
            GM6020.temp = rx_data[6];
            break;
        }
        
        case 0x205: // Pitch轴电机
        {
            // 解析原始数据
            float raw_angle = (float)((rx_data[0] << 8) | rx_data[1]);
            float raw_speed = (float)((int16_t)((rx_data[2] << 8) | rx_data[3]));
            
            // Pitch轴使用更严格的阈值
            GM6020_pitch.rotor_angle = (uint16_t)simple_angle_filter(
                raw_angle, 
                &motor_205_filter, 
                3000.0f     // Pitch轴：允许最大50个编码器单位的变化
            );
            
            GM6020_pitch.rotor_speed = (int16_t)simple_speed_filter(
                raw_speed, 
                &motor_205_filter, 
                500.0f    // Pitch轴：允许最大500rpm的速度变化
            );
            
            GM6020_pitch.torque_current = (int16_t)((rx_data[4] << 8) | rx_data[5]);
            GM6020_pitch.temp = rx_data[6];
            break;
        }
        
        default:
            break;
    }
}

/**
 * @brief 重置滤波器状态
 * @param motor_id 电机ID
 */
void Reset_Filter_State(uint32_t motor_id)
{
    switch(motor_id)
    {
        case 0x207:
            memset(&motor_207_filter, 0, sizeof(SimpleFilterState));
            break;
            
        case 0x205:
            memset(&motor_205_filter, 0, sizeof(SimpleFilterState));
            break;
            
        default:
            // 重置所有滤波器
            memset(&motor_207_filter, 0, sizeof(SimpleFilterState));
            memset(&motor_205_filter, 0, sizeof(SimpleFilterState));
            break;
    }
}

// 保持发送函数不变
CAN_TxHeaderTypeDef Ctx;

void Send_GM6020_Motor_Message(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box; 
    
    Ctx.StdId = CAN_CHASSIS_ALL_ID;
    Ctx.IDE = CAN_ID_STD;
    Ctx.RTR = CAN_RTR_DATA;
    Ctx.DLC = 0x08;
    
    chassis_can_send_data[0] = (uint8_t)(motor1 >> 8); 
    chassis_can_send_data[1] = (uint8_t)(motor1); 
    chassis_can_send_data[2] = (uint8_t)(motor2 >> 8); 
    chassis_can_send_data[3] = (uint8_t)(motor2); 
    chassis_can_send_data[4] = (uint8_t)(motor3 >> 8); 
    chassis_can_send_data[5] = (uint8_t)(motor3); 
    chassis_can_send_data[6] = (uint8_t)(motor4 >> 8); 
    chassis_can_send_data[7] = (uint8_t)(motor4); 
 
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &Ctx, chassis_can_send_data, &send_mail_box); 
    HAL_CAN_AddTxMessage(&CHASSIS_CAN2, &Ctx, chassis_can_send_data, &send_mail_box); 
}