/**
 * @file LK9025.c
 * @brief LK9025 motor driver implementation
 * 
 * This file contains the implementation of LK9025 motor control functions
 * including motor initialization, control, and communication via CAN bus.
 * 
 * @author [Author Name]
 * @date [Date]
 * @version 1.0
 */

#include "LK9025.h"          // LK9025电机驱动头文件
#include <stdio.h>
#include <string.h>
#include "stdlib.h"         // 标准库，用于malloc等函数
// #include "general_def.h" // 通用定义
// #include "daemon.h"      // 看门狗模块
#include "bsp_dwt.h"     // DWT计时器
// #include "bsp_log.h"     // 日志输出

//测试
//ID是1，用的时候是0X140+ID，也就是0X141

/** @brief 电机实例计数器，记录已注册的电机数量 */
static uint8_t idx;

/** @brief LK电机实例数组，存储所有注册的电机实例指针 */
// static LKMotorInstance *lkmotor_instance[LK_MOTOR_MX_CNT] = {NULL};

/** @brief 多电机发送时使用的CAN实例(当前保存的是注册的第一个电机的CAN实例) */
static CANInstance *sender_instance;

/** @brief 全局电机测量数据结构体，用于存储电机的测量数据 */
LKMotor_Measure_t measure;

/**
 * @brief 电机反馈报文解析函数
 * 
 * 解析从CAN总线接收到的电机反馈数据，包括编码器值、速度、电流和温度。
 * 同时计算多圈角度和数据更新时间间隔。
 *
 * @param[out] measure 电机测量数据结构体指针，用于存储解析后的数据
 * @param[in] rx_buff CAN接收缓冲区指针，包含原始的8字节反馈数据
 * 
 * @note 数据帧格式：
 *       - rx_buff[0]: 保留字节
 *       - rx_buff[1]: 温度数据
 *       - rx_buff[2-3]: 电流数据 (低字节在前)
 *       - rx_buff[4-5]: 速度数据 (低字节在前)
 *       - rx_buff[6-7]: 编码器数据 (低字节在前)
 */
void LKMotorDecode(LKMotor_Measure_t *measure, uint8_t *rx_buff) {
    // // 通过CAN实例保存的father id获取对应的电机实例
    // LKMotorInstance *motor = (LKMotorInstance *)_instance->id; 
    // // 获取电机测量数据结构体指针
    // LKMotor_Measure_t *measure = &motor->measure;
    // 获取CAN接收缓冲区指针
    //uint8_t *rx_buff = _instance->rx_buff;
     
    // DaemonReload(motor->daemon); // 喂狗(重载看门狗计数器)
    
    // 计算数据更新时间间隔
    measure->feed_dt = DWT_GetDeltaT(&measure->feed_dwt_cnt);

    // 保存上一次的编码器值，用于计算转动圈数
    measure->last_ecd = measure->ecd;
    
    // 解析编码器原始值：将第6、7字节组合成16位编码器值
    measure->ecd = (uint16_t)((rx_buff[7] << 8) | rx_buff[6]);

    // 计算单圈角度：编码器值乘以角度转换系数
    measure->angle_single_round = ECD_ANGLE_COEF_LK * measure->ecd;

    // 解析并滤波速度数据：从第4、5字节提取速度值，转换为弧度/秒并进行低通滤波
    measure->speed_rads = (1 - SPEED_SMOOTH_COEF) * measure->speed_rads +
                          DEGREE_2_RAD * SPEED_SMOOTH_COEF * (float)((int16_t)(rx_buff[5] << 8 | rx_buff[4]));

    // 解析并滤波电流数据：从第2、3字节提取电流值并进行低通滤波
    measure->real_current = (1 - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rx_buff[3] << 8 | rx_buff[2]));

    // 直接读取温度数据（第1字节）
    measure->temperature = rx_buff[1];

    // 多圈角度计算：检测编码器是否跨越零点
    if (measure->ecd - measure->last_ecd > 8192) {       // 编码器从大值跳到小值，反向转过零点
        measure->total_round--;                           // 总圈数减1
    } else if (measure->ecd - measure->last_ecd < -8192) { // 编码器从小值跳到大值，正向转过零点
        measure->total_round++;                           // 总圈数加1
    }
    
    // 计算总角度：总圈数×360° + 当前单圈角度
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

/**
 * @brief 电机离线回调函数
 * 
 * 当电机失去通信连接时调用此函数，用于记录警告信息。
 * 
 * @param[in] motor_ptr 电机实例指针（void*类型，需要强制转换）
 */
// static void LKMotorLostCallback(void *motor_ptr) {
//     // 将通用指针转换为电机实例指针
//     LKMotorInstance *motor = (LKMotorInstance *)motor_ptr;
//     // 输出警告日志，显示离线电机的ID
//     LOGWARNING("[LKMotor] motor lost, id: %d", motor->motor_can_ins->tx_id);
// }

/**
 * @brief 启用LK9025电机
 * 
 * 通过CAN总线发送启用命令，使电机进入工作状态。
 * 
 * @param[in] hcan CAN句柄指针
 * @param[in] id 电机CAN ID
 * 
 * @note 发送数据格式：[0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 */
void Motor_LK9025_Enable(CAN_HandleTypeDef* hcan, uint16_t id) {
    CAN_TxHeaderTypeDef _TxHeader;
    uint8_t Txtemp[8];
    
    // 配置CAN发送帧头
    _TxHeader.StdId = id;             // 设置标准ID
    _TxHeader.IDE = CAN_ID_STD;       // 使用标准ID格式
    _TxHeader.RTR = CAN_RTR_DATA;     // 数据帧
    _TxHeader.DLC = 0x08;             // 数据长度8字节
    
    // 填充启用命令数据
    Txtemp[0] = 0x88;  // 启用命令字
    Txtemp[1] = 0x00;
    Txtemp[2] = 0x00;
    Txtemp[3] = 0x00;
    Txtemp[4] = 0x00;
    Txtemp[5] = 0x00;
    Txtemp[6] = 0x00;
    Txtemp[7] = 0x00;

    // 尝试通过多个邮箱发送，确保发送成功
    HAL_CAN_AddTxMessage(hcan, &_TxHeader, Txtemp, (uint32_t*)CAN_TX_MAILBOX0);
    HAL_CAN_AddTxMessage(hcan, &_TxHeader, Txtemp, (uint32_t*)CAN_TX_MAILBOX1);
    HAL_CAN_AddTxMessage(hcan, &_TxHeader, Txtemp, (uint32_t*)CAN_TX_MAILBOX2);
}

/**
 * @brief 禁用LK9025电机
 * 
 * 通过CAN总线发送禁用命令，使电机停止工作。
 * 
 * @param[in] hcan CAN句柄指针
 * @param[in] id 电机CAN ID
 * 
 * @note 发送数据格式：[0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
 */
void Motor_LK9025_disable(CAN_HandleTypeDef* hcan, uint16_t id) {
    CAN_TxHeaderTypeDef _TxHeader;
    uint8_t Txtemp[8];
    
    // 配置CAN发送帧头
    _TxHeader.StdId = id;             // 设置标准ID
    _TxHeader.IDE = CAN_ID_STD;       // 使用标准ID格式
    _TxHeader.RTR = CAN_RTR_DATA;     // 数据帧
    _TxHeader.DLC = 0x08;             // 数据长度8字节
    
    // 填充禁用命令数据
    Txtemp[0] = 0x81;  // 禁用命令字
    Txtemp[1] = 0x00;
    Txtemp[2] = 0x00;
    Txtemp[3] = 0x00;
    Txtemp[4] = 0x00;
    Txtemp[5] = 0x00;
    Txtemp[6] = 0x00;
    Txtemp[7] = 0x00;
    
    HAL_CAN_AddTxMessage(hcan, &_TxHeader, Txtemp, (uint32_t*)CAN_TX_MAILBOX0);
}

/**
 * @brief 控制LK9025电机速度
 * 
 * 通过CAN总线发送速度控制命令。
 * 
 * @param[in] hcan CAN句柄指针
 * @param[in] id 电机CAN ID
 * @param[in] speed 目标速度值（32位有符号整数）
 * 
 * @note 发送数据格式：[0xA2, 0x00, 0x00, 0x00, speed_low, speed_mid_low, speed_mid_high, speed_high]
 *       速度数据按小端格式存储在第4-7字节中
 */
void Motor_LK9025_control_speed(CAN_HandleTypeDef* hcan, uint16_t id, int32_t speed) {
    CAN_TxHeaderTypeDef _TxHeader;
    uint8_t Txtemp[8];
    
    // 配置CAN发送帧头
    _TxHeader.StdId = id;             // 设置标准ID
    _TxHeader.IDE = CAN_ID_STD;       // 使用标准ID格式
    _TxHeader.RTR = CAN_RTR_DATA;     // 数据帧
    _TxHeader.DLC = 0x08;             // 数据长度8字节
    
    // 填充速度控制命令数据
    Txtemp[0] = 0xA2;  // 速度控制命令字
    Txtemp[1] = 0x00;
    Txtemp[2] = 0x00;
    Txtemp[3] = 0x00;
    
    // 将32位速度值按小端格式分解到4个字节中
    Txtemp[4] = speed;            // 最低字节
    Txtemp[5] = speed >> 8;       // 次低字节
    Txtemp[6] = speed >> 16;      // 次高字节
    Txtemp[7] = speed >> 24;      // 最高字节
    
    HAL_CAN_AddTxMessage(hcan, &_TxHeader, Txtemp, (uint32_t*)CAN_TX_MAILBOX0);
}





/**
* @brief 多电机转矩闭环控制（标准帧0x280）
* 
* @param[in] hcan CAN句柄指针
* @param[in] iq_controls 转矩电流数组（int16_t[4]类型，范围-2000~2000）
* 
* @note 数据格式：
*       数据域[0-1] 电机1转矩电流（小端）
*       数据域[2-3] 电机2转矩电流（小端）
*       数据域[4-5] 电机3转矩电流（小端）
*       数据域[6-7] 电机4转矩电流（小端）
*/

void Motor_LK9025_Control_Torque_Multi(CAN_HandleTypeDef* hcan, int16_t iq_controls[4]) {
    CAN_TxHeaderTypeDef _TxHeader;
    uint8_t Txtemp[8];
    
    // 配置CAN帧头（固定标识符0x280）
    _TxHeader.StdId = 0x280;        // 固定帧ID
    _TxHeader.IDE = CAN_ID_STD;     // 标准帧
    _TxHeader.RTR = CAN_RTR_DATA;   // 数据帧
    _TxHeader.DLC = 0x08;           // 固定8字节
    
    // 将4个转矩值打包为小端格式
    for(int i = 0; i < 4; i++) {
        Txtemp[2*i] = iq_controls[i] & 0xFF;        // 低字节
        Txtemp[2*i + 1] = (iq_controls[i] >> 8) & 0xFF; // 高字节
    }
    
    HAL_CAN_AddTxMessage(hcan, &_TxHeader, Txtemp, (uint32_t*)CAN_TX_MAILBOX0);
}



/**
 * @brief 控制LK9025电机转矩电流
 * 
 * 通过CAN总线发送转矩闭环控制命令
 * 
 * @param[in] hcan CAN句柄指针
 * @param[in] id 电机CAN ID
 * @param[in] iqControl 目标转矩电流值（16位有符号整数，范围-2000~2000对应-32A~32A）
 * 
 * @note 发送数据格式：[0xA1, 0x00, 0x00, 0x00, iq_low, iq_high, 0x00, 0x00]
 *       转矩数据按小端格式存储在第4-5字节中
 */
void Motor_LK9025_control_torque(CAN_HandleTypeDef* hcan, uint16_t id, int16_t iqControl) {
    CAN_TxHeaderTypeDef _TxHeader;
    uint8_t Txtemp2[8];
    
    // 配置CAN发送帧头
    _TxHeader.StdId = id;             // 设置标准ID
    _TxHeader.IDE = CAN_ID_STD;       // 使用标准ID格式
    _TxHeader.RTR = CAN_RTR_DATA;     // 数据帧
    _TxHeader.DLC = 0x08;             // 数据长度8字节
    
    // 填充转矩控制命令数据
    Txtemp2[0] = 0xA1;  // 转矩控制命令字
    Txtemp2[1] = 0x00;
    Txtemp2[2] = 0x00;
    Txtemp2[3] = 0x00;
    
    // 将16位转矩值按小端格式分解到4-5字节
    Txtemp2[4] = (uint8_t)(iqControl & 0xFF);       // 低字节
    Txtemp2[5] = (uint8_t)((iqControl >> 8) & 0xFF); // 高字节
    Txtemp2[6] = 0x00;
    Txtemp2[7] = 0x00;
    
    HAL_CAN_AddTxMessage(hcan, &_TxHeader, Txtemp2, (uint32_t*)CAN_TX_MAILBOX0);
		HAL_CAN_AddTxMessage(hcan, &_TxHeader, Txtemp2, (uint32_t*)CAN_TX_MAILBOX1);
}


// /**
//  * @brief 停止电机
//  * @param motor 要停止的电机实例
//  */
// void LKMotorStop(LKMotorInstance *motor)
// {
//     motor->stop_flag = MOTOR_STOP;
// }

// /**
//  * @brief 启用电机
//  * @param motor 要启用的电机实例
//  */
// void LKMotorEnable(LKMotorInstance *motor)
// {
//     motor->stop_flag = MOTOR_ENALBED;
// }

// /**
//  * @brief 设置电机参考值
//  * @param motor 电机实例
//  * @param ref 参考值
//  */
// void LKMotorSetRef(LKMotorInstance *motor, float ref)
// {
//     motor->pid_ref = ref;
// }

// /**
//  * @brief 检查电机是否在线
//  * @param motor 电机实例
//  * @return uint8_t 返回1表示在线，0表示离线
//  */
// uint8_t LKMotorIsOnline(LKMotorInstance *motor)
// {
//     return DaemonIsOnline(motor->daemon);
// }
// 	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
// }	

// void Motor_LK9025_control_speed(CAN_HandleTypeDef* hcan,uint16_t id,int32_t speed)
// {
// 	CAN_TxHeaderTypeDef _TxHeader;
// 	uint8_t Txtemp[8];
// 	_TxHeader.StdId = id;
// 	_TxHeader.IDE = CAN_ID_STD;
// 	_TxHeader.RTR = CAN_RTR_DATA;
// 	_TxHeader.DLC = 0x08;
// 	Txtemp[0] = 0xA2;
// 	Txtemp[1] = 0x00;
// 	Txtemp[2] = 0x00;
// 	Txtemp[3] = 0x00;
	
// 	Txtemp[4] = speed;   
// 	Txtemp[5] = speed>>8; 
	
// 	Txtemp[6] = (speed>>8)>>8;
// 	Txtemp[7] = ((speed>>8)>>8)>>8; 
	
// 	HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
// }

// /**
//  * @brief 停止电机
//  * @param motor 要停止的电机实例
//  */
// void LKMotorStop(LKMotorInstance *motor)
// {
//     motor->stop_flag = MOTOR_STOP;
// }

// /**
//  * @brief 启用电机
//  * @param motor 要启用的电机实例
//  */
// void LKMotorEnable(LKMotorInstance *motor)
// {
//     motor->stop_flag = MOTOR_ENALBED;
// }

// /**
//  * @brief 设置电机参考值
//  * @param motor 电机实例
//  * @param ref 参考值
//  */
// void LKMotorSetRef(LKMotorInstance *motor, float ref)
// {
//     motor->pid_ref = ref;
// }

// /**
//  * @brief 检查电机是否在线
//  * @param motor 电机实例
//  * @return uint8_t 返回1表示在线，0表示离线
//  */
// uint8_t LKMotorIsOnline(LKMotorInstance *motor)
// {
//     return DaemonIsOnline(motor->daemon);
// }
