/* 头文件包含 */
#include "Usart1.h"    // 用户自定义头文件
#include "main.h"      // 主工程头文件
#include "cmsis_os.h"  // CMSIS-RTOS头文件
#include "usart.h"     // HAL库UART头文件
//记住RX对应TX，也就是TXD要接接收端的RXD。
/* 全局变量定义 */
uint8_t tx_buf[] = "HelloRM\r\n";


/**
* @brief 串口发送函数
*
* @note 占位符
*/
void usart_Send()  
{
     
		// 阻塞式发送数据
		HAL_UART_Transmit(&huart6, tx_buf, sizeof(tx_buf)-1, 1000);  
		
		
    
}

/**a
* @brief 串口通信任务
*
* @note 暂无
*/
void usart_Task2(void const * argument){//此处参数是一个意义不明的东西，需要学习.此段代码废弃
	while(1)  
    {
			usart_Send();
		}
	
}


