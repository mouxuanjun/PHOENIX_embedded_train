#ifndef LK9025_H
#define LK9025_H

#include "stdint.h"
#include "bsp_can.h"
// #include "controller.h"
// #include "motor_def.h"
// #include "daemon.h"

#define LK_MOTOR_MX_CNT 4 // 最多允许4个LK电机使用多电机指令,挂载在一条总线上

#define I_MIN -2000
#define I_MAX 2000
#define CURRENT_SMOOTH_COEF 0.9f
#define SPEED_SMOOTH_COEF 0.85f
#define REDUCTION_RATIO_DRIVEN 1
#define ECD_ANGLE_COEF_LK (360.0f / 65536.0f)
#define CURRENT_TORQUE_COEF_LK 0.00512f  // 电流设定值转换成扭矩的系数，这里对应的是16T
#pragma pack(1)



#ifndef PI
#define PI 3.1415926535f
#endif
#define PI2 (PI * 2.0f) // 2 pi

#define RAD_2_DEGREE 57.2957795f    // 180/pi
#define DEGREE_2_RAD 0.01745329252f // pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f       // ×360°/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f // ×2pi/60sec

typedef struct _
{
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // CAN消息填入的邮箱号
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id
    uint8_t rx_len;                // 接收长度,可能为0-8
    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct _ *); // callback needs an instance to tell among registered ones
    void *id;                                // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} CANInstance;

typedef struct // 9025
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 当前编码器值
    float angle_single_round; // 单圈角度
    float speed_rads;         // speed rad/s
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度,C°

    float total_angle;   // 总角度
    int32_t total_round; // 总圈数

    float feed_dt;
    uint32_t feed_dwt_cnt;
} LKMotor_Measure_t;

// typedef struct
// {
//     Closeloop_Type_e outer_loop_type;              // 最外层的闭环,未设置时默认为最高级的闭环
//     Closeloop_Type_e close_loop_type;              // 使用几个闭环(串级)
//     Motor_Reverse_Flag_e motor_reverse_flag;       // 是否反转
//     Feedback_Reverse_Flag_e feedback_reverse_flag; // 反馈是否反向
//     Feedback_Source_e angle_feedback_source;       // 角度反馈类型
//     Feedback_Source_e speed_feedback_source;       // 速度反馈类型
//     Feedfoward_Type_e feedforward_flag;            // 前馈标志

// } Motor_Control_Setting_s;


// typedef struct
// {
//     LKMotor_Measure_t measure;

//     Motor_Control_Setting_s motor_settings;

//     float *other_angle_feedback_ptr; // 其他反馈来源的反馈数据指针
//     float *other_speed_feedback_ptr;
//     float *speed_feedforward_ptr;   // 速度前馈数据指针,可以通过此指针设置速度前馈值,或LQR等时作为速度状态变量的输入
//     float *current_feedforward_ptr; // 电流前馈指针
//     // PIDInstance current_PID;
//     // PIDInstance speed_PID;
//     // PIDInstance angle_PID;
//     float pid_ref;

//     // Motor_Working_Type_e stop_flag; // 启停标志

//     CANInstance *motor_can_ins;

//     DaemonInstance *daemon;

// } LKMotorInstance;

// /**
//  * @brief 初始化LK电机
//  *
//  * @param config 电机配置
//  * @return LKMotorInstance* 返回实例指针
//  */
// LKMotorInstance *LKMotorInit(Motor_Init_Config_s *config);

// /**
//  * @brief 设置参考值
//  * @attention 注意此函数设定的ref是最外层闭环的输入,若要设定内层闭环的值请通过前馈数据指针设置
//  *
//  * @param motor 要设置的电机
//  * @param ref 设定值
//  */
// void LKMotorSetRef(LKMotorInstance *motor, float ref);

/**
 * @brief 为所有LK电机计算pid/反转/模式控制,并通过bspcan发送电流值(发送CAN报文)
 *
 */
//void LKMotorControl();

// /**
//  * @brief 停止LK电机,之后电机不会响应任何指令
//  *
//  * @param motor
//  */
// void LKMotorStop(LKMotorInstance *motor);

// /**
//  * @brief 启动LK电机
//  *
//  * @param motor
//  */
// void LKMotorEnable(LKMotorInstance *motor);

// uint8_t LKMotorIsOnline(LKMotorInstance *motor);
void Motor_LK9025_disable(CAN_HandleTypeDef* hcan, uint16_t id) ;
void Motor_LK9025_Enable(CAN_HandleTypeDef* hcan, uint16_t id) ;
void LKMotorDecode(LKMotor_Measure_t *measure, uint8_t *rx_buff) ;
void Motor_LK9025_control_speed(CAN_HandleTypeDef* hcan, uint16_t id, int32_t speed) ;
void Motor_LK9025_control_torque(CAN_HandleTypeDef* hcan, uint16_t id, int16_t iqControl) ;
void Motor_LK9025_Control_Torque_Multi(CAN_HandleTypeDef* hcan, int16_t iq_controls[4]);

#endif // LK9025_H
