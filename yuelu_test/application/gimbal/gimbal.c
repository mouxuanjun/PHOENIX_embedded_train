#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "buzzer.h"
#include "bmi088.h"

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;
static BuzzzerInstance *buzzer;//蜂鸣器
static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

void GimbalInit()
{
    BuzzerInit();
    Buzzer_config_s buzzer_config = {
        .alarm_level = ALARM_LEVEL_HIGH,
        .loudness = 0.1,
        .octave = 1,
    };
    buzzer = BuzzerRegister(&buzzer_config);
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id =1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp =0.95,//0.4_0.25(2_28)i
                .Ki = 0,//0.15
                .Kd = 0.002,//0.006
                .CoefA = 0.5,//0.5
                .CoefB = 0.6,//0.6
                .DeadBand = 0.02,//0.005
                .Output_LPF_RC = 0.01,//0.01
                .Improve = PID_Trapezoid_Intergral |PID_ChangingIntegrationRate| PID_Integral_Limit| PID_OutputFilter|PID_DerivativeFilter,
                .IntegralLimit =0.5,//1
                .MaxOut = 400,//600
            },
            .speed_PID = {
                .Kp=2900,//14000_10000(2_28)
                .Ki =0,//0
                .Kd =210,//0.0005
                // .CoefA =1500,//1500
                // .CoefB =2000,//2000
                .Output_LPF_RC = 0.02,//0.005
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit  | PID_OutputFilter,
                .IntegralLimit =5000,//3000
                .MaxOut = 20000,//20000
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.75, //0.55  
                .Ki = 0.05,//0.05
                .Kd = 0.02,//0.02
                .CoefA =0.5,//0.5
                .CoefB = 0.6,//0.6
                .Output_LPF_RC = 0.01,
                .DeadBand = 0.005,//0.02
                .Derivative_LPF_RC=0.01,//0.008
                .Improve = PID_Trapezoid_Intergral |PID_ChangingIntegrationRate| PID_Integral_Limit |PID_Derivative_On_Measurement |  PID_OutputFilter |PID_DerivativeFilter,
                .IntegralLimit = 1,
                .MaxOut = 600,
            },
            .speed_PID = {
                .Kp = 10000,//22000
                .Ki = 0,//
                .Kd =0.0005,
                // .CoefA = 0.8,
                // .CoefB = 0.1,
                .Output_LPF_RC = 0.002,//0.002
                .Improve = PID_Trapezoid_Intergral |PID_Integral_Limit |PID_Derivative_On_Measurement |  PID_OutputFilter,
                .IntegralLimit = 5000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP|ANGLE_LOOP ,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,//FEEDBACK_DIRECTION_REVERSE
        },
        .motor_type = GM6020,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    // last_total_angle=yaw_motor->measure.last_angle;
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);//改成电机反馈？
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);//
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        break;
    default:
        break;
    }
    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_angle = yaw_motor->measure.angle_single_round;
    gimbal_feedback_data.pitch_angle = pitch_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}