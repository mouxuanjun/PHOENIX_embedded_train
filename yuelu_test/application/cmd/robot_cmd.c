// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "buzzer.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
static Work_Mode_e vision_work_mode;
static float chassis_speed_buff;
static uint8_t EmergencyHandlerflag = 0;
static gimbal_control_e Gimbal_control;
static float Gimbal[2];
static int keyC_flag = 0;
static int keyC_last_flag =0;
static int cap_flag = 0;//用于超电回复死区

static uint16_t chassis_power_robot_level;
// PITCH轴限位，带测定

#define PITCH_MAX 21
#define PITCH_MIN -19   

void RobotCMDInit()
{
    Gimbal[0]=0;
    Gimbal[1]=0;//初始化gimbal调参数组，以便Ozone寻地址
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    shoot_cmd_send.attack_mode = NORMAL;
    gimbal_cmd_send.pitch = 0;
    gimbal_cmd_send.yaw = 0;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send. = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    chassis_cmd_send.rotate_control = 1.0;
    // 云台软件限位
    if (gimbal_cmd_send.pitch > PITCH_MAX)
        gimbal_cmd_send.pitch = PITCH_MAX;
    else if (gimbal_cmd_send.pitch < PITCH_MIN)
        gimbal_cmd_send.pitch = PITCH_MIN;

    // 左侧开关状态为[中]且右侧开关为[下],视觉模式
    if (switch_is_mid(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right))
    {

        //gimbal_cmd_send.yaw = (vision_recv_data->yaw == 0 ? gimbal_cmd_send.yaw : -(vision_recv_data->yaw)); // 由于视觉调试时角度给反所以添加负号-H
        //gimbal_cmd_send.pitch = (vision_recv_data->pitch == 0 ? gimbal_cmd_send.pitch : vision_recv_data->pitch);
        // 待添加,视觉会发来和目标的误差,同样将其转化为total angle的增量进行控制
        chassis_cmd_send.chassis_mode = CHASSIS_RE_ROTATE;                                                     // 由于24赛季检录需要滑环检测故调整，调试视觉时请取消注释
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        // chassis_cmd_send.fly_flag = 1;
        // ...
    }
    if(switch_is_mid(rc_data[TEMP].rc.switch_left) && switch_is_mid(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    if (switch_is_down(rc_data[TEMP].rc.switch_left) || vision_recv_data->target_state == NO_TARGET)
    {                                                                                                // 按照摇杆的输出大小进行角度增量,增益系数需调整
        gimbal_cmd_send.yaw += 0.0025f * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
        // gimbal_cmd_send.yaw = Gimbal[0];
        // gimbal_cmd_send.pitch = Gimbal[1];  
        // Gimbal[]用于gimbal调参（阶跃响应），需要调参时取消注释
    }
    if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘小陀螺
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_REMOTE; // CHASSIS_ROTATE
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }


    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    chassis_cmd_send.vx = 80.0f * (float)rc_data[TEMP].rc.rocker_r_; // _水平方向
    chassis_cmd_send.vy = 80.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1竖直方向

    // 发射参数

    // 摩擦轮控制,拨轮向上打为负,向下为正
    if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开摩擦轮
        shoot_cmd_send.friction_mode = FRICTION_ON;
    else
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (rc_data[TEMP].rc.dial < -500)
    {
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE; // LOAD_BURSTFIRELOAD_1_BULLET
    }
    else
        shoot_cmd_send.load_mode = LOAD_STOP;
    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    if (rc_data[TEMP].rc.switch_left == 3 && switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
        if (vision_recv_data->target_state == READY_TO_FIRE)
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        else
            shoot_cmd_send.load_mode = LOAD_STOP;
    }
    shoot_cmd_send.shoot_rate = 13;
}

// /**
//  * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
//  *         停止的阈值'300'待修改成合适的值,或改为开关控制.
//  *
//  * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
//  *
//  */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {

        EmergencyHandlerflag = 1;
    }
    if (switch_is_mid(rc_data[TEMP].rc.switch_right) && EmergencyHandlerflag == 1)
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;

        LOGINFO("[CMD] reinstate, robot ready");
        EmergencyHandlerflag = 0;
        gimbal_cmd_send.yaw = -gimbal_fetch_data.gimbal_imu_data.YawTotalAngle; // 急停时设定值保持与实际值同步，避免恢复时疯转
        gimbal_cmd_send.pitch = 0;
    }
    // 此处为cyx设置，将遥控器急停恢复设置为右上然后右中组合，以防键鼠控制无法急停等
    // 故不建议在右上添加底盘跟随或小陀螺（surprise）
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
    {
        switch (rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL][Key_C] % 2) // ctrl+c 进入急停
        {
        case 0:
            robot_state = ROBOT_READY;
            shoot_cmd_send.shoot_mode = SHOOT_ON;
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            break;

        default:
            robot_state = ROBOT_STOP;
            gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
            shoot_cmd_send.shoot_mode = SHOOT_OFF;
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.load_mode = LOAD_STOP;

            gimbal_cmd_send.yaw = -gimbal_fetch_data.gimbal_imu_data.YawTotalAngle; // 急停时设定值保持与实际值同步，避免恢复时疯转
            gimbal_cmd_send.pitch = 0;
            break;
        }
    }
}

static void MouseKeySet()
{
    shoot_cmd_send.shoot_rate = 8;
    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    chassis_cmd_send.heat_control = HOLD;
    chassis_cmd_send.rotate_control = 1.0;
    chassis_cmd_send.fly_flag = 0;
    chassis_cmd_send.vy = 0.55 * (rc_data[TEMP].key[KEY_PRESS].w * chassis_speed_buff - rc_data[TEMP].key[KEY_PRESS].s * chassis_speed_buff); // 系数待测，平移运动功率限制！
    chassis_cmd_send.vx = 0.55 * (rc_data[TEMP].key[KEY_PRESS].a * chassis_speed_buff - rc_data[TEMP].key[KEY_PRESS].d * chassis_speed_buff);
    switch (rc_data[TEMP].key[KEY_PRESS].x) // X键刷新UI
    {
    case 1:
        chassis_cmd_send.ui_mode = UI_REFRESH;
        break;
    default:
        chassis_cmd_send.ui_mode = UI_KEEP;
        break;
    }

    switch (rc_data[TEMP].mouse.press_r) // 鼠标右键开启自瞄
    {
    case 0:
        gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 8; // 系数待测
        gimbal_cmd_send.pitch -= (float)rc_data[TEMP].mouse.y / 660 * 8;
        // pitch限位
        if (gimbal_cmd_send.pitch > PITCH_MAX)
            gimbal_cmd_send.pitch = PITCH_MAX;
        else if (gimbal_cmd_send.pitch < PITCH_MIN)
            gimbal_cmd_send.pitch = PITCH_MIN;
        break;

    default:
        shoot_cmd_send.shoot_rate = 13;
        if (vision_recv_data->target_state == NO_TARGET)
        {
            gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 8; // 系数待测
            gimbal_cmd_send.pitch -= (float)rc_data[TEMP].mouse.y / 660 * 8;
        }
        else
        {
            gimbal_cmd_send.yaw = -(vision_recv_data->yaw == 0 ? gimbal_cmd_send.yaw : vision_recv_data->yaw);
            gimbal_cmd_send.pitch = (vision_recv_data->pitch == 0 ? gimbal_cmd_send.pitch : vision_recv_data->pitch);
        }
        // 视觉状态
        if (vision_recv_data->target_state == NO_TARGET)
            chassis_cmd_send.vision_mode = UNLOCK;
        else if (vision_recv_data->target_state == TARGET_CONVERGING)
            chassis_cmd_send.vision_mode = CONVERGE;
        else if (vision_recv_data->target_state == READY_TO_FIRE)
            chassis_cmd_send.vision_mode = LOCK;
        else
            chassis_cmd_send.vision_mode = UNLOCK;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2) // E键设置切换发射模式：单发/连发
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        chassis_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    case 1:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        chassis_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
        //提前为chassis赋值，以在发弹前更新UI
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_V] % 2) // V键设置切换射频为狂暴模式
    {
    case 0:
        chassis_cmd_send.attack_mode = NORMAL;
        break;
    case 1:
        chassis_cmd_send.attack_mode = VIOLENT;
        chassis_cmd_send.heat_control = FIGHT;
        shoot_cmd_send.shoot_rate = 25;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 2) // Z键设置是否接入热量闭环
    {
    case 0:
        break;
    case 1:
        chassis_cmd_send.heat_control = FIGHT;
        break;
    }
    switch (rc_data[TEMP].mouse.press_l) // 鼠标左键射击
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    default:
        if (shoot_cmd_send.friction_mode != FRICTION_ON)
            shoot_cmd_send.load_mode = LOAD_STOP; // 摩擦轮不开启则拨盘不转, 防止卡弹
        if (rc_data[TEMP].mouse.press_r && (vision_recv_data->target_state != READY_TO_FIRE))
        {
            shoot_cmd_send.load_mode = LOAD_STOP;
            break;
        }
        if(chassis_fetch_data.over_heat_flag==1 && chassis_cmd_send.heat_control == HOLD)
        {
            shoot_cmd_send.load_mode = LOAD_STOP;
            break;
        }
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 2) // G键狗洞模式
    {
    case 0:
        chassis_cmd_send.tunnel_mode = TUNNEL_OFF;
        break;
    default:
        gimbal_cmd_send.pitch = 0;
        chassis_cmd_send.tunnel_mode = TUNNEL_ON;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Q] % 2) // Q键设置底盘运动模式
    {
    case 0:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    default:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键设置45度转向
    {
    case 0:
        chassis_cmd_send.fly_flag = 0;
        break;
    default:
        chassis_cmd_send.offset_angle += 45;
        chassis_cmd_send.fly_flag = 1;
        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 按shift使用超级电容
    {
    case 1:
        chassis_speed_buff = 40000;
        break;
    default:
        chassis_speed_buff = 13000;
        break;
    }
    //建议增加超级电容电压控制，如小于12V时降低增速等，充至16V恢复等
    //此处为24赛季双板通信出现问题故未修改（云台板收不到底盘板反馈信息：未解决）

    //TODO:B键设置功率
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_B] % 11)
        {
        case 0:
            chassis_power_robot_level = 0;
            break;
        case 1:
            chassis_power_robot_level = 1;
            break;
        case 2:
            chassis_power_robot_level = 2;
            break;
        case 3:
            chassis_power_robot_level = 3;
            break;
        case 4:
            chassis_power_robot_level = 4;
            break;
        case 5:
            chassis_power_robot_level = 5;
            break;
        case 6:
            chassis_power_robot_level = 6;
            break;
        case 7:
            chassis_power_robot_level = 7;
            break;
        case 8:
            chassis_power_robot_level = 8;
            break;
        case 9:
            chassis_power_robot_level = 9;
            break;
        case 10:
            chassis_power_robot_level = 10;
            break;
        default:
            chassis_power_robot_level = 0;
            break;
        }
        // 24hl：有点蠢，修改等级需要急停
    }




/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    chassis_cmd_send.ui_mode = UI_KEEP;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        MouseKeySet(); // 调试专用
    }
    else
    {
        RemoteControlSet();
    } // 遥控器左侧开关状态为[上],键盘控制
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    VisionSetFlag(chassis_fetch_data.self_color, vision_work_mode, 30);//30为弹速
    //顺序为pitch，yaw（需发送总角度，以防出现角度跟随bug），roll
    VisionSetAltitude(gimbal_fetch_data.gimbal_imu_data.Pitch, gimbal_fetch_data.gimbal_imu_data.YawTotalAngle, gimbal_fetch_data.gimbal_imu_data.Roll);

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
    shoot_cmd_send.bullet_speed = chassis_fetch_data.bullet_speed;
    chassis_cmd_send.friction_mode = shoot_cmd_send.friction_mode;
    chassis_cmd_send.yaw_angle = gimbal_fetch_data.gimbal_imu_data.Yaw;
    chassis_cmd_send.pitch_angle = gimbal_fetch_data.pitch_angle;
    chassis_cmd_send.init_totalangle = shoot_fetch_data.init_totalangle;
    chassis_cmd_send.totalangle = shoot_fetch_data.totalangle;
    chassis_cmd_send.chassis_power_robot_level = chassis_power_robot_level;

    
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
    //chassis_cmd_send.yaw_motor_total_round_angle = gimbal_fetch_data.yaw_motor_total_round_angle;
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
