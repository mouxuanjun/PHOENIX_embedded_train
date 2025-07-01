/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "buzzer.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "math.h"
/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static referee_info_t *referee_data;                                // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data;                          // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI
static BuzzzerInstance *buzzer;                                     // 蜂鸣器
static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static float angle = 0;
static float last_total_angle=0;
static float GIMBAL_angle = 0;
static float YAW_START_ANGLE=0;
/* 用于自旋变速策略的时间变量 */
static referee_info_t *referee_data; // 裁判系统相关数据
static SuperCapInstance *cap;        // 超级电容
static DJIMotorInstance *motor_lf;   // left right forward back
static DJIMotorInstance *motor_rf;
static DJIMotorInstance *motor_lb;
static DJIMotorInstance *motor_rb;
static PIDInstance gimbal_follow;
static uint16_t DataSend2Cap[4] = {0, 0, 0, 0};
static float chassis_SPEED_PID[3] = {4, 0, 0.05};
static float chassis_CURRENT_PID[3] = {0.5, 0, 0};
static float32_t cnt;
static uint16_t power_data[2];
static float The_Angle;
/* 用于自旋变速策略的时间变量,后续考虑查表加速 */
// static float t;
static float GIMBAL_angle;
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
static float rotate_speed_buff = 0;
static float fly_go = 1;

/*弹量及热量计算*/
//拨盘电机数据
static float init_loader_totalangle;//初始化时角度
static float last_loader_totalangle;//用于储存上一次计算时角度
static float last_loader_totalangle1;//用于储存上一次计算时角度
static float loader_totalangle;//当前角度

static uint8_t first_flag=1;//用于读取初始化电机角度
static uint8_t overheatflag=0;//用于判断是否超热量

//剩余弹量
static int rest_bullet=TOTAL_BULLET;

//枪口热量数据
static uint16_t shoot_heat_limit;//热量上限
static uint16_t shoot_cooling_val;//每秒冷却值
static uint16_t shoot_heat=0;//枪口当前热量(计算值)
static uint16_t heat_k=1.2;//控制热量系数
//读取裁判系统时间间隔
static float last_time;
void ChassisInit()
{
    // BuzzerInit();
    Buzzer_config_s buzzer_config = {
        .alarm_level = ALARM_LEVEL_HIGH,
        .loudness = 0.1,
        .octave = 1,
    };
    buzzer = BuzzerRegister(&buzzer_config);
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = chassis_SPEED_PID[0], // 4.5
                .Ki = chassis_SPEED_PID[1], // 0
                .Kd = chassis_SPEED_PID[2], // 0
                .IntegralLimit = 3000,
                .Output_LPF_RC = 0.02,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .MaxOut = 16000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x302, // 超级电容默认接收id
            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // ww超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .daemon_count = 5, //50ms未接受数据则丢失
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化

    PID_Init_Config_s follow_gimbal_config = {
        .Kp = 100,
        .Ki = 2,
        .Kd = 5,
        .Derivative_LPF_RC = 0,
        .Output_LPF_RC = 0,
        .IntegralLimit = 200,
        .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .MaxOut = 13000,
    };
    PIDInit(&gimbal_follow,&follow_gimbal_config);
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf = chassis_vx + chassis_vy + chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = -chassis_vx + chassis_vy + chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    rotate_speed_buff = 4;
    static float  chassis_power_buff = 1.0;
    // 功率限制待添加
    power_data[0] = referee_data->PowerHeatData.buffer_energy;        // 缓冲能量
    //power_data[1] = referee_data->GameRobotState.chassis_power_limit; // 功率限制

    switch (chassis_cmd_recv.chassis_power_robot_level) // shift+ctrl 进入手动设置底盘功率
    {
    case 1:
        power_data[1]=45;  
        break;
    case 2:
        power_data[1]=50;  
        break;
    case 3:
        power_data[1]=55;  
        break;
    case 4:
        power_data[1]=60;  
        break;
    case 5:
        power_data[1]=65;  
        break;
    case 6:
        power_data[1]=70;  
        break;
    case 7:
        power_data[1]=75;  
        break;
    case 8:
        power_data[1]=80;  
        break;
    case 9:
        power_data[1]=90;  
        break;
    case 10:
        power_data[1]=100;  
        break;

    default:
        power_data[1]=referee_data->GameRobotState.chassis_power_limit; 
        break; 
    }
    // 发送功率数据
    SuperCapSend(cap, (uint8_t *)&power_data);
    chassis_cmd_recv.super_cap.chassis_power_mx = cap->cap_msg.vol * 0.001;
    switch (power_data[1]) // referee_data->GameRobotState.chassis_power_limit
    {
    case 45:
        rotate_speed_buff = 3.6;
        chassis_power_buff = 1.0;
        break;
    case 50:
        rotate_speed_buff = 4.1;
        chassis_power_buff = 1.0;
        break;
    case 55:
        rotate_speed_buff = 4.3;
        chassis_power_buff = 1.0;
        break;
    case 60:
        rotate_speed_buff = 4.4;
        chassis_power_buff = 1.0;//1.4
        break;
        //-----------------------
    case 65:
        rotate_speed_buff = 5;
        chassis_power_buff = 1.08;
        break;
    case 70:
        rotate_speed_buff = 5.2;//5.6
        chassis_power_buff = 1.1;//1.6
        break;
    case 75:
        rotate_speed_buff = 5.4;
        chassis_power_buff = 1.15;
        break;
    case 80: // 调试ing
        rotate_speed_buff = 5.6;
        chassis_power_buff = 1.17;
        break;
    case 85: // 调试ing
        rotate_speed_buff = 5.6;
        chassis_power_buff = 1.17;
        break;
    case 90:
        rotate_speed_buff = 5.8;
        chassis_power_buff = 1.2;
        break;
    case 95:
        rotate_speed_buff = 5.8;
        chassis_power_buff = 1.2;
        break;
    case 100:
        rotate_speed_buff = 6.0;
        chassis_power_buff = 1.25;
        break;
    case 120:
        rotate_speed_buff = 6.2;
        chassis_power_buff = 1.4;
    default:
        rotate_speed_buff = 3.7;
        chassis_power_buff = 1.0;
        break;
    }
    rotate_speed_buff += 0.15*(cap->cap_msg.vol*0.001-19);
    chassis_power_buff += 0.01*(cap->cap_msg.vol*0.001-19);
    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf*chassis_power_buff);
    DJIMotorSetRef(motor_rf, vt_rf*chassis_power_buff);
    DJIMotorSetRef(motor_lb, vt_lb*chassis_power_buff);
    DJIMotorSetRef(motor_rb, vt_rb*chassis_power_buff);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    chassis_feedback_data.real_wz = ( motor_lf->measure.speed_aps + motor_lb->measure.speed_aps  + motor_rb->measure.speed_aps + motor_rf->measure.speed_aps)
    /19/4*76.5/2/332/1.414;// /减速比/取平均*轮子半径/车小陀螺半径（332/2）/根号2
}
/*计算枪口热量和剩余弹量*/
static void Cal_bullet_heat()
{  
    //读取热量上限和冷却值
   shoot_heat_limit=referee_data->GameRobotState.shooter_barrel_heat_limit;
   shoot_cooling_val=referee_data->GameRobotState.shooter_barrel_cooling_value;
   
   //记录初始情况下的电机角度，时间
    if(first_flag==1)
    {
        last_time=DWT_GetTimeline_ms();
        last_loader_totalangle1=init_loader_totalangle;
        shoot_heat=0;
        first_flag=0;
        rest_bullet=TOTAL_BULLET;
    }

//每0.1s根据裁判系统更新一次数据，是否有必要？
//hl：暂时未加，由于Ozone观测该热量预测与裁判系统吻合，故取消验证，以免赛场裁判系统读取出现问题
//     if(DWT_GetTimeline_ms()-last_time>=100)
//     {
//     shoot_heat=referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
//     last_time=DWT_GetTimeline_ms();
//    }

    //计算实际发弹数以及剩余弹量
    if((loader_totalangle-last_loader_totalangle)>=ONE_BULLET_DELTA_ANGLE*REDUCTION_RATIO_LOADER && rest_bullet >0)
    {
       rest_bullet=TOTAL_BULLET-(loader_totalangle-init_loader_totalangle)/(ONE_BULLET_DELTA_ANGLE*REDUCTION_RATIO_LOADER);
       last_loader_totalangle=loader_totalangle;
    }

    // //比赛中补弹计算
    // if(referee_data->SupplyProjectileAction.supply_robot_id == referee_data->GameRobotState.robot_id && referee_data->SupplyProjectileAction.supply_projectile_step == 2)
    // rest_bullet+=referee_data->SupplyProjectileAction.supply_projectile_num;
    //全向轮无需补实体弹

    //以10Hz每秒减少、增加热量
    if(DWT_GetTimeline_ms()-last_time>=100 && shoot_heat>=shoot_cooling_val/10)
    {
        shoot_heat-=shoot_cooling_val/10;
        last_time=DWT_GetTimeline_ms();
            //每发射1发小弹丸加10点热量
            if((loader_totalangle-last_loader_totalangle1)>=3600)//40*90
            {
                shoot_heat+=((loader_totalangle-last_loader_totalangle1)/(3600*1.0))*10;
                last_loader_totalangle1=loader_totalangle;
            }
    }else  if(DWT_GetTimeline_ms()-last_time>=100 && shoot_heat<shoot_cooling_val/10)
    {
        shoot_heat=0;
        last_time=DWT_GetTimeline_ms();
            //每发射1发小弹丸加10点热量
            if((loader_totalangle-last_loader_totalangle1)>=3600)//40*90
            {
                shoot_heat+=((loader_totalangle-last_loader_totalangle1)/(3600*1.0))*10;
                last_loader_totalangle1=loader_totalangle;
            }
    }

    //限幅+冷却范围整定
    if(shoot_heat>(2*shoot_heat_limit))
    {
        shoot_heat=2*shoot_heat_limit;
    }
    else if (shoot_heat<=0)
    {
        shoot_heat=0;
    }

    //发送超热量标志射频*（任务频率or冷却频率？）*每发热量*系数
    //允许超热量1-2发
    if(shoot_heat>shoot_heat_limit-chassis_cmd_recv.shoot_rate*0.1*10*heat_k)
        overheatflag=1;
    else if(shoot_heat <= shoot_heat_limit-chassis_cmd_recv.shoot_rate*0.1*10*heat_k)
        overheatflag=0;
}


/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    The_Angle = 0;
    //The_Angle用于抵消转45度之后偏航角计算偏差

    // 后续增加没收到消息的处理(双板的情况)
    // 24hl：已增加，表现为底盘板超过50ms未收到云台板消息时蜂鸣器工作，详情见can_comm.c文件

    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
    //  StayAngle();
#endif // CHASSIS_BOARD

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }
    if(chassis_cmd_recv.fly_flag == 1)
    {
       The_Angle = (float)45;
    }//抵消45度转向偏航角偏差
        
    // 根据控制模式设定旋转速度
    cnt = (float32_t)DWT_GetTimeline_s();//用于变速小陀螺
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
        chassis_cmd_recv.wz = -3*chassis_cmd_recv.offset_angle*abs(chassis_cmd_recv.offset_angle);
        //Init模块已初始化底盘跟随PID，但未使用，待调参
        // PIDCalculate(gimbal_follow,chassis_cmd_recv.offset_angle,0);//反馈值为robot_cmd中计算的偏航角，目标值为0
        break;
    case CHASSIS_ROTATE:             // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = (1200+100*(float32_t)sin(cnt))*rotate_speed_buff*chassis_cmd_recv.rotate_control;//此处系数繁琐，待修改
        break;
    case CHASSIS_RE_ROTATE:
        chassis_cmd_recv.wz = -2400;      
        break;
    case CHASSIS_ROTATE_REMOTE: // 遥控器降速版小陀螺，用于24赛季滑环检测
        chassis_cmd_recv.wz = 2400;
        break;
    default:
        break;
    }
    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为y，右侧为x（y指向正北时x指向正东）
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32((chassis_cmd_recv.offset_angle-The_Angle) * DEGREE_2_RAD);
    sin_theta = arm_sin_f32((chassis_cmd_recv.offset_angle-The_Angle) * DEGREE_2_RAD);
    chassis_vx = (chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta);
    chassis_vy = (chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta);
    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    //更新发射机构数据
    init_loader_totalangle=chassis_cmd_recv.init_totalangle;
    loader_totalangle=chassis_cmd_recv.totalangle;
    //用于计算发弹量：通过拨盘转过角度计算
    Cal_bullet_heat();
    // 根据电机的反馈速度和IMU(如果有)计算真实速度（24hl：未使用IMU）
    EstimateSpeed();

    // 底盘反馈云台版信息：
    // 视觉数据
    // 我方颜色id小于10是红色,大于10是蓝色, 1:red , 2:blue
    chassis_feedback_data.self_color = referee_data->GameRobotState.robot_id > 10 ? COLOR_BLUE : COLOR_RED;
    // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    chassis_feedback_data.robot_level = referee_data->GameRobotState.robot_level;//用于手动设置功率，通过设置等级设置
    chassis_feedback_data.bullet_speed = referee_data->ShootData.bullet_speed;//用于shoot模块控制弹速c
    chassis_feedback_data.cap_vol = cap->cap_msg.vol * 0.001;//用于控制并观测超级电容电压
#ifdef CHASSIS_BOARD
    chassis_feedback_data.wz = Chassis_IMU_data->Gyro[2];
#endif 
    //发送枪口超热量标志
    chassis_feedback_data.over_heat_flag=overheatflag;

    //绘制UI所需数据
    ui_data.Chassis_Power_Data.chassis_power_mx = chassis_cmd_recv.super_cap.chassis_power_mx;
    ui_data.chassis_mode = chassis_cmd_recv.chassis_mode;
    ui_data.friction_mode = chassis_cmd_recv.friction_mode;
    ui_data.loader_mode = chassis_cmd_recv.load_mode;
    ui_data.ui_mode = chassis_cmd_recv.ui_mode;
    ui_data.vision_mode = chassis_cmd_recv.vision_mode;
    ui_data.attack_mode = chassis_cmd_recv.attack_mode;
    ui_data.heat_control = chassis_cmd_recv.heat_control;
    ui_data.tunnel_mode = chassis_cmd_recv.tunnel_mode;
    ui_data.rest_bullets=rest_bullet;
    ui_data.shoot_heat=shoot_heat;
    ui_data.shoot_heat_limit=referee_data->GameRobotState.shooter_barrel_heat_limit;
    ui_data.chassis_power_robot_level=chassis_cmd_recv.chassis_power_robot_level;
    ui_data.cross_state=chassis_cmd_recv.fly_flag;

    // 比赛时间获取（24赛季未使用）
    switch (referee_data->GameState.stage_remain_time)
    {
    // case 365:
    // case 275:
    // case 185:
    // case 110:
    case 35:
        ui_data.time_mode =2;
        break;
    // case 375:
    // case 285:
    // case 195:
    // case 120:
    case 45:
        ui_data.time_mode =1;
        break;
    default:
        ui_data.time_mode =0;
        break;
    }
    // 推送反馈消息

// 一定注意：不同赛季裁判系统串口通信不一定相同，注意及时修改！，可参考hero修改记录

#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}