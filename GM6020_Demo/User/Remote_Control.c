#include "Remote_Control.h"    // 用户自定义头文件
#include "main.h"      // 主工程头文件
#include "cmsis_os.h"  // CMSIS-RTOS头文件
#include "freertos.h"
#include "usart.h"
//CMSIS-RTOS 的消息队列（osMessageQ）在底层设计上 统一使用 32 位（4 字节）存储消息值，无论你实际定义的消息数据类型是什么。这意味着：
//即使你定义的消息类型是 uint8_t、uint16_t 甚至结构体，发送时也必须将数据强制转换为 uint32_t。
//接收端在读取消息时，需要将 uint32_t 转换回原始数据类型。
#define MSG_MAGIC 0x00FF1234    //消息头用于分别数据，此处表示
#define MSG_MAGIC2 0x00FF1235   //用于分别数据，表示是yaw轴  




extern osMessageQId RCqueueHandle; //声明这个句柄
extern uint8_t sbus_buf[18];
//uint8_t sbus_buf[18];
RC_Ctl_t ctl;

uint8_t test2=0;
void RC_init() {
    // 确保DMA时钟已使能 (通常在MX_DMA_Init()中完成)
    // 确保USART3和DMA相关的GPIO和NVIC已在HAL_UART_MspInit()中正确配置

    // 启动DMA模式的UART接收
    HAL_StatusTypeDef status;
    status = HAL_UART_Receive_DMA(&huart3, sbus_buf, 18); // 18是您期望接收的字节数
    if (status != HAL_OK) {
        // 启动DMA接收失败
        // 在这里设置断点，查看 status 的具体值 (HAL_BUSY, HAL_ERROR, HAL_TIMEOUT)
        // 以便诊断为什么DMA启动失败
        Error_Handler();
    }
}
/**
* @brief RC串口处理函数
*
* @note 通过消息队列传输,此处使用了无限等待消息队列空闲，实际使用中不要这么干。此外发送时候强制转换为uint32以便于兼容，所以接收时候也要记得转换回uint16
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){//传入的参数用于确认是哪个USART串口接收到的数据
 // Channel 0: 从字节 0-1 提取 11 位数据
    ctl.rc.ch0 = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07FF; 

    // Channel 1: 从字节 1-2 提取 11 位数据
    ctl.rc.ch1 = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07FF;

    // Channel 2: 从字节 2-4 提取 11 位数据
    ctl.rc.ch2 = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07FF;

    // Channel 3: 从字节 4-5 提取 11 位数据
    ctl.rc.ch3 = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07FF;

    // 左开关（bit5的bit4-5）
    ctl.rc.sL = (sbus_buf[5] >>4) & 0x03; 
    // 右开关（bit5的bit6-7）
    ctl.rc.sR = (sbus_buf[5] >>6) & 0x03;

		test2++;
   //yaw轴消息头
    osMessagePut(RCqueueHandle, MSG_MAGIC2, 0);
	//传递Yaw轴数据（右摇杆横向）
		osMessagePut(RCqueueHandle, (uint32_t)ctl.rc.ch0, 0);//也可以直接传递指针但是会失去线程安全的意义
		//消息头，用于分割和识别数据
		osMessagePut(RCqueueHandle, MSG_MAGIC, 0);
	//传递左开关
	  osMessagePut(RCqueueHandle, (uint32_t)ctl.rc.sL, 0);
}