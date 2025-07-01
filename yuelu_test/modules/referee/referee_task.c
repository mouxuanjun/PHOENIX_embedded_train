/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"
#include "math.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
// @todo 不应该使用全局变量

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
// static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void UITask()
{
    // RobotModeTest(Interactive_data); // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
    MyUIRefresh(referee_recv_info, Interactive_data);
}

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_Energy[4];      // 电容能量条
static Graph_Data_t UI_Energy[4];      // 电容能量条
static Graph_Data_t UI_Vision[3];      // 视觉标识框
static Graph_Data_t UI_heat[3];        // 热量标识
static String_Data_t UI_State_sta[7];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[10]; // 机器人状态,动态先add才能change
static Graph_Data_t UI_heat[3];        // 热量闭环能量条
static Graph_Data_t UI_restbullet[3];  // 弹舱弹量能量条
void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务

    if (Interactive_data->ui_mode == UI_KEEP)
        return;

    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制发射基准线
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, 540, 1210, 540); // 横向基准线
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, 960, 340, 960, 740);  // 纵向基准线
    UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Purplish_red, 2, 760, 510, 1160, 510);
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Orange, 3, 960, 140, 928, 510);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[0], UI_shoot_line[1]);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[5], UI_shoot_line[4]);
    // // 联盟赛旧步兵发射管偏移标注斜线：
    // UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Orange, 3, 935, 140, 932, 540);
    // UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[5]);
    // 绘制车辆状态标志指示
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 150, 700, "frict:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 150, 650, "vision:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 150, 600, "load:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    UICharDraw(&UI_State_sta[6], "sc1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 150, 800, "CLEVEL");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[6]);

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Orange, 0, 2, 680, 680, "follow");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 700, "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 650, "unlock");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    UICharDraw(&UI_State_dyn[3], "sd4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 1130, 680, "nor-13");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    UICharDraw(&UI_State_dyn[4], "sd7", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 600, "load_1");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
    // UICharDraw(&UI_State_dyn[5], "sd9", UI_Graph_ADD, 8, UI_Color_Cyan, 0, 2, 920, 680, "15");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);
    UICharDraw(&UI_State_dyn[6], "sv1", UI_Graph_ADD, 8, UI_Color_Purplish_red, 15, 2, 1130, 400, "hold");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[6]);
    // UILineDraw(&UI_shoot_line[5],"sd8",UI_Graph_ADD, 8, UI_Color_Purplish_red, 2, 1600, 680, 1700, 680);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_shoot_line[5]);
    UICharDraw(&UI_State_dyn[9], "sf1", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 800, "RF");
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_State_dyn[9]);
    // 绘制视觉标识框
    UIRectangleDraw(&UI_Vision[0], "sd3", UI_Graph_ADD, 9, UI_Color_Green, 2, 650, 700, 1270, 380);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Vision[0]);
    // pitch旋转初始圆形及限位
    //  UICircleDraw(&UI_Vision[1],"ss4",UI_Graph_ADD, 7 ,UI_Color_Orange,2,1600,680,40);
    //  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Vision[1]);
    //  UILineDraw(&UI_shoot_line[6],"sl6",UI_Graph_ADD,7,UI_Color_Yellow,2,1600,680,1693,716);
    //  UILineDraw(&UI_shoot_line[8],"sl8",UI_Graph_ADD,7,UI_Color_Yellow,2,1600,680,1693,644);
    //  UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[6],UI_shoot_line[8]);
    // 45度转向UI显示
    UILineDraw(&UI_shoot_line[6], "sl6", UI_Graph_ADD, 7, UI_Color_Yellow, 0, 1550, 630, 1650, 730);
    UILineDraw(&UI_shoot_line[8], "sl8", UI_Graph_ADD, 7, UI_Color_Yellow, 0, 1650, 730, 1550, 630);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[6], UI_shoot_line[8]);

    // 弹量显示弧度初始化
    UIArcDraw(&UI_restbullet[0], "sr0", UI_Graph_ADD, 7, UI_Color_Green, 0, 360, 4, 1600, 680, 40, 40);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_restbullet[0]);

    // 能量条框
    UIRectangleDraw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 670, 50, 1428, 90);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);
    // 底盘功率显示,动态
    UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 630, 120, 24000);
    // 能量条初始状态
    UILineDraw(&UI_Energy[2], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 30, 670, 70, 920, 70);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
    // 狗洞划线
    UILineDraw(&UI_Energy[2], "sg1", UI_Graph_ADD, 8, UI_Color_Pink, 0, 700, 700, 600, 400);
    UILineDraw(&UI_Energy[3], "sg2", UI_Graph_ADD, 8, UI_Color_Pink, 0, 1300, 700, 1400, 400);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[2], UI_Energy[3]);
    // //热量闭环框
    // UIRectangleDraw(&UI_heat[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 710, 70, 1418, 110);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_heat[0]);
    // // 热量闭环能量条初始状态
    // UILineDraw(&UI_heat[1], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 30, 710, 100, 910, 100);
    // UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_heat[1]);
    // 枪口热量显示，静态
    // UICharDraw(&UI_State_sta[6], "ss6", UI_Graph_ADD, 9, UI_Color_Green, 15, 2, 620, 70, "Heat:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[6]);
    // //热量条框
    // UIRectangleDraw(&UI_heat[0], "sh0", UI_Graph_ADD, 9, UI_Color_Green, 2, 720, 50, 1128, 90);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_heat[0]);
    // //
    // //UIFloatDraw(&UI_heat[2], "sh6", UI_Graph_ADD, 9, UI_Color_Green, 18, 2, 2, 1150, 210,0);
    // // 热量条初始状态
    // UILineDraw(&UI_heat[1], "sh1", UI_Graph_ADD, 9, UI_Color_Cyan, 0, 720, 70, 720, 70);
    // UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_heat[1],UI_heat[2]);
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        switch (_Interactive_data->chassis_mode)
        {
        case CHASSIS_ZERO_FORCE:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 20, 3, 680, 680, "zeroforce");
            break;
        case CHASSIS_ROTATE:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 20, 3, 680, 680, "rotate   ");
            // 此处注意字数对齐问题，字数相同才能覆盖掉
            break;
        default:
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 20, 3, 680, 680, "         ");
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 20, 3, 680, 680, "         ");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }
    // friction
    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    {
        UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 700, _Interactive_data->friction_mode == FRICTION_ON ? "on " : "off");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }
    // vision
    if (_Interactive_data->Referee_Interactive_Flag.vision_flag == 1)
    {
        switch (_Interactive_data->vision_mode)
        {
        case UNLOCK:
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, "unlock  ");
            UIRectangleDraw(&UI_Vision[0], "sd3", UI_Graph_Change, 9, UI_Color_Green, 2, 650, 700, 1270, 380);
            break;
        case CONVERGE:
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, "converge");
            UIRectangleDraw(&UI_Vision[0], "sd3", UI_Graph_Change, 9, UI_Color_Purplish_red, 2, 650, 700, 1270, 380);
            break;
        case LOCK:
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 650, "lock    ");
            UIRectangleDraw(&UI_Vision[0], "sd3", UI_Graph_Change, 9, UI_Color_Purplish_red, 2, 650, 700, 1270, 380);
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Vision[0]);
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Vision[0]);
        _Interactive_data->Referee_Interactive_Flag.vision_flag = 0;
    }
    // attack
    if (_Interactive_data->Referee_Interactive_Flag.attack_flag == 1)
    {
        switch (_Interactive_data->attack_mode)
        {
        case NORMAL:
            UICharDraw(&UI_State_dyn[3], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 20, 2, 1130, 680, "nor-08");
            break;
        case VIOLENT:
            UICharDraw(&UI_State_dyn[3], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 20, 2, 1130, 680, "vio-25");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.attack_flag = 0;
    }
    // power
    if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)
    {
        UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 630, 120, _Interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
        UILineDraw(&UI_Energy[2], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 30, 670, 70, (uint32_t)690 + _Interactive_data->Chassis_Power_Data.chassis_power_mx * 30, 70);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    }
    // load
    if (_Interactive_data->Referee_Interactive_Flag.loader_flag == 1)
    {
        switch (_Interactive_data->loader_mode)
        {
        case LOAD_1_BULLET:
            UICharDraw(&UI_State_dyn[4], "sd7", UI_Graph_Change, 8, UI_Color_Cyan, 20, 2, 270, 600, "load_1    ");
            break;
        case LOAD_BURSTFIRE:
            UICharDraw(&UI_State_dyn[4], "sd7", UI_Graph_Change, 8, UI_Color_Orange, 20, 2, 270, 600, "load_burst");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
        _Interactive_data->Referee_Interactive_Flag.loader_flag = 0;
    }

    // //Pitch
    // if (_Interactive_data->Referee_Interactive_Flag.pitch_flag == 1)
    // {
    //     static float32_t angle = 0;
    //     static uint32_t situation_x;
    //     static uint32_t situation_y;
    //     angle = (float32_t)293.59-_Interactive_data->pitch_angle;
    //     situation_x = 1600+100*arm_cos_f32(angle*0.01745329252);
    //     situation_y = 680+100*arm_sin_f32(angle*0.01745329252);
    //     UILineDraw(&UI_shoot_line[5], "sd8", UI_Graph_Change, 8, UI_Color_Purplish_red, 2, 1600, 680, situation_x, situation_y);//_Interactive_data->pitch_angle*1000
    //     UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_shoot_line[5]);
    //     _Interactive_data->Referee_Interactive_Flag.pitch_flag = 0;
    // }

    // Time
    // if (_Interactive_data->Referee_Interactive_Flag.time_flag == 1)
    // {
    //     if (_Interactive_data->time_mode == 1)
    //         UICharDraw(&UI_State_dyn[5], "sd9", UI_Graph_Change, 8, UI_Color_Cyan, 30, 2, 920, 680, "15");
    //     else if (_Interactive_data->time_mode == 2)
    //         UICharDraw(&UI_State_dyn[5], "sd9", UI_Graph_Change, 8, UI_Color_Cyan, 30, 2, 920, 680, "5 ");
    //     UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);
    //     _Interactive_data->Referee_Interactive_Flag.time_flag == 0;
    // }
    // Heat_control
    if (_Interactive_data->Referee_Interactive_Flag.heat_flag == 1)
    {
        switch (_Interactive_data->heat_control)
        {
        case HOLD:
            UICharDraw(&UI_State_dyn[6], "sv1", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 1130, 400, "hold ");
            break;
        case FIGHT:
            UICharDraw(&UI_State_dyn[6], "sv1", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 1130, 400, "fight");
            break;
        default:
            // UICharDraw(&UI_State_dyn[6], "sv1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 1130,400, "     ");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[6]);
        _Interactive_data->Referee_Interactive_Flag.heat_flag = 0;
    }
    // 狗洞
    if (_Interactive_data->Referee_Interactive_Flag.tunnel_flag == 1)
    {
        if (_Interactive_data->tunnel_mode == TUNNEL_ON)
        {
            UILineDraw(&UI_Energy[2], "sg1", UI_Graph_Change, 8, UI_Color_Pink, 10, 700, 700, 600, 400);
            UILineDraw(&UI_Energy[3], "sg2", UI_Graph_Change, 8, UI_Color_Pink, 10, 1300, 700, 1400, 400);
        }
        else
            UILineDraw(&UI_Energy[2], "sg1", UI_Graph_Change, 8, UI_Color_Pink, 0, 700, 700, 600, 400);
        UILineDraw(&UI_Energy[3], "sg2", UI_Graph_Change, 8, UI_Color_Pink, 0, 1300, 700, 1400, 400);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[2], UI_Energy[3]);
        _Interactive_data->Referee_Interactive_Flag.tunnel_flag == 0;
    }
    // rest_bullet
    if (_Interactive_data->Referee_Interactive_Flag.bullet_flag == 1)
    {
        // UIIntDraw(&UI_restbullet[2], "sh6", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 1150, 210,1228+(int)(Interactive_data->rest_bullets*1.0/240*200));
        if (Interactive_data->rest_bullets > 300)
        {
            UIArcDraw(&UI_restbullet[0], "sr0", UI_Graph_Change, 7, UI_Color_Green, 0, 0.6 * (int)(Interactive_data->rest_bullets), 4, 1600, 680, 40, 40);
            UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_restbullet[0]);
            _Interactive_data->Referee_Interactive_Flag.bullet_flag = 0;
        }
        else if (Interactive_data->rest_bullets > 100 && Interactive_data->rest_bullets <= 300)
        {
            UIArcDraw(&UI_restbullet[0], "sr0", UI_Graph_Change, 7, UI_Color_Orange, 0, 0.6 * (int)(Interactive_data->rest_bullets), 4, 1600, 680, 40, 40);
            UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_restbullet[0]);
            _Interactive_data->Referee_Interactive_Flag.bullet_flag = 0;
        }
        else if (Interactive_data->rest_bullets <= 100)
        {
            UIArcDraw(&UI_restbullet[0], "sr0", UI_Graph_Change, 7, UI_Color_Purplish_red, 0, 0.6 * (int)(Interactive_data->rest_bullets), 4, 1600, 680, 40, 40);
            UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_restbullet[0]);
            _Interactive_data->Referee_Interactive_Flag.bullet_flag = 0;
        }
    }
    if (_Interactive_data->Referee_Interactive_Flag.chassis_level_flag == 1)
    {
        switch (_Interactive_data->chassis_power_robot_level)
        {
        case 0:
            UICharDraw(&UI_State_dyn[9], "sf1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, "RF");
            break;
        case 1:
            UICharDraw(&UI_State_dyn[9], "sf1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, "1 ");
            break;
        case 2:
            UICharDraw(&UI_State_dyn[9], "sf1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, "2 ");
            break;
        case 3:
            UICharDraw(&UI_State_dyn[9], "sf1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, "3 ");
            break;
        case 4:
            UICharDraw(&UI_State_dyn[9], "sf1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, "4 ");
            break;
        case 5:
            UICharDraw(&UI_State_dyn[9], "sf1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, "5 ");
            break;
        case 6:
            UICharDraw(&UI_State_dyn[9], "sf1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, "6 ");
            break;
        case 7:
            UICharDraw(&UI_State_dyn[9], "sf1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, "7 ");
            break;
        case 8:
            UICharDraw(&UI_State_dyn[9], "sf1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, "8 ");
            break;
        case 9:
            UICharDraw(&UI_State_dyn[9], "sf1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, "9 ");
            break;
        case 10:
            UICharDraw(&UI_State_dyn[9], "sf1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, "10");
            break;
        default:
            UICharDraw(&UI_State_dyn[9], "sf1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, "RF");
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[9]);
        _Interactive_data->Referee_Interactive_Flag.chassis_level_flag = 0;
    }
    // 45度转向
    if (_Interactive_data->Referee_Interactive_Flag.cross_state_flag == 1)
    {
        switch (_Interactive_data->cross_state)
        {
        case 0:
            UILineDraw(&UI_shoot_line[6], "sl6", UI_Graph_Change, 7, UI_Color_Yellow, 10, 1550, 630, 1650, 730);
            UILineDraw(&UI_shoot_line[8], "sl8", UI_Graph_Change, 7, UI_Color_Yellow, 10, 1550, 730, 1650, 630);
            break;
        case 1:
            UILineDraw(&UI_shoot_line[6], "sl6", UI_Graph_Change, 7, UI_Color_Yellow, 10, 1600, 630, 1600, 730);
            UILineDraw(&UI_shoot_line[8], "sl8", UI_Graph_Change, 7, UI_Color_Yellow, 10, 1650, 680, 1550, 680);
            break;
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[6], UI_shoot_line[8]);
        _Interactive_data->Referee_Interactive_Flag.cross_state_flag = 0;
    }
}
// heat
// if (_Interactive_data->Referee_Interactive_Flag.heat_flag == 1)
// {
//     //UIIntDraw(&UI_heat[2], "sh5", UI_Graph_Change, 9, UI_Color_Green, 18, 2, 2, 950, 210,(float)((uint16_t)720 + _Interactive_data->shoot_heat_limit/408*Interactive_data->shoot_heat));
//     UILineDraw(&UI_heat[1], "sh1", UI_Graph_Change, 9, UI_Color_Pink, 30, 720, 70, 720 + (int)(408*1.0/_Interactive_data->shoot_heat_limit*Interactive_data->shoot_heat), 70);
//     UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_heat[1],UI_heat[2]);
//     _Interactive_data->Referee_Interactive_Flag.heat_flag = 0;
// }
// if (_Interactive_data->Referee_Interactive_Flag.heat_flag == 1)
// {
//     //UIIntDraw(&UI_heat[2], "sh5", UI_Graph_Change, 9, UI_Color_Green, 18, 2, 2, 950, 210,(float)((uint16_t)720 + _Interactive_data->shoot_heat_limit/408*Interactive_data->shoot_heat));
//     UILineDraw(&UI_heat[1], "sh1", UI_Graph_Change, 9, UI_Color_Pink, 30, 720, 70, 720 + (int)(408*1.0/_Interactive_data->shoot_heat_limit*Interactive_data->shoot_heat), 70);
//     UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_heat[1],UI_heat[2]);
//     _Interactive_data->Referee_Interactive_Flag.heat_flag = 0;
// }

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    // if (_Interactive_data->pitch_angle != _Interactive_data->pitch_last_angle)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.pitch_flag = 1;
    //     _Interactive_data->pitch_last_angle = _Interactive_data->pitch_angle;
    // }

    if (_Interactive_data->heat_control != _Interactive_data->heat_last_control)
    {
        _Interactive_data->Referee_Interactive_Flag.heat_flag = 1;
        _Interactive_data->heat_last_control = _Interactive_data->heat_control;
    }

    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }

    if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    {
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
        _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    }

    if (_Interactive_data->vision_mode != _Interactive_data->vision_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.vision_flag = 1;
        _Interactive_data->vision_last_mode = _Interactive_data->vision_mode;
    }

    if (_Interactive_data->attack_mode != _Interactive_data->attack_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.attack_flag = 1;
        _Interactive_data->attack_last_mode = _Interactive_data->attack_mode;
    }
    if (_Interactive_data->loader_mode != _Interactive_data->loader_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.loader_flag = 1;
        _Interactive_data->loader_last_mode = _Interactive_data->loader_mode;
    }
    if (_Interactive_data->time_mode != _Interactive_data->time_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.time_flag = 1;
        _Interactive_data->time_last_mode = _Interactive_data->time_mode;
    }
    if (_Interactive_data->tunnel_mode != _Interactive_data->tunnel_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.tunnel_flag = 1;
        _Interactive_data->tunnel_last_mode = _Interactive_data->tunnel_mode;
    }

    if (_Interactive_data->rest_bullets != _Interactive_data->last_rest_bullets)
    {
        _Interactive_data->Referee_Interactive_Flag.bullet_flag = 1;
        _Interactive_data->last_rest_bullets = _Interactive_data->rest_bullets;
    }
    if (_Interactive_data->shoot_heat != _Interactive_data->last_shoot_heat)
    {
        _Interactive_data->last_shoot_heat = _Interactive_data->shoot_heat;
        _Interactive_data->Referee_Interactive_Flag.heat_flag = 1;
    }
    if (_Interactive_data->chassis_power_robot_level != _Interactive_data->chassis_power_last_robot_level)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_level_flag = 1;
        _Interactive_data->chassis_power_last_robot_level = _Interactive_data->chassis_power_robot_level;
    }
    if (_Interactive_data->cross_state != _Interactive_data->cross_last_state)
    {
        _Interactive_data->Referee_Interactive_Flag.cross_state_flag = 1;
        _Interactive_data->cross_last_state = _Interactive_data->cross_state;
    }
}
