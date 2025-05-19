#include "FreeRTOS.h"
#include "task.h"  
#include "dri_can.h"
#include "cmsis_os.h"



#define Motor_1_ID 0x207
#define Motor_2_ID 0x205
//uint8_t test3=0;
/**
 * @file BSP_Can.c
 * @brief 初始化筛选器（这里掩码和显码都是0）
 * @author HWX
 * @editor CGH
 * @date 2025/5/14
 */
 extern TaskHandle_t CAN_input_taskHandle; // 吃屎的freertos和cmsis混合API

uint8_t CAN_Input;//can接收中断标志位
CAN_RxHeaderTypeDef rx_header;//将其作为公共使用
uint8_t rx_data[8];//接收（RX）缓冲器
void CAN_Filter_Init(void)
{
    CAN_FilterTypeDef can1_filter_st;
	
    can1_filter_st.FilterIdHigh = 0x0000;
    can1_filter_st.FilterIdLow = 0x0000;
    can1_filter_st.FilterMaskIdHigh = 0x0000;
    can1_filter_st.FilterMaskIdLow = 0x0000;
    can1_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can1_filter_st.FilterActivation = ENABLE;
    can1_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can1_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can1_filter_st.FilterBank = 0;
    can1_filter_st.SlaveStartFilterBank = 14;
	
	//使能CAN通道
    if (HAL_CAN_ConfigFilter(&hcan1, &can1_filter_st) != HAL_OK)// 配置CAN1过滤器
    {
        Error_Handler();  // 处理错误·
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)// 启动CAN1
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)// 使能 CAN1 接受 FIFO0 中断
    {
        Error_Handler();
    }
		HAL_Delay(10);
		//CAN2初始化配置
		 CAN_FilterTypeDef can2_filter_st;
	
    can2_filter_st.FilterIdHigh = 0x0000;
    can2_filter_st.FilterIdLow = 0x0000;
    can2_filter_st.FilterMaskIdHigh = 0x0000;
    can2_filter_st.FilterMaskIdLow = 0x0000;
    can2_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can2_filter_st.FilterActivation = ENABLE;
    can2_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can2_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can2_filter_st.FilterBank = 14;
    can2_filter_st.SlaveStartFilterBank = 14;
	
	//使能CAN通道
    if (HAL_CAN_ConfigFilter(&hcan2, &can2_filter_st) != HAL_OK)// 配置CAN2过滤器
    {
        Error_Handler();  // 处理错误·
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK)// 启动CAN2
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)// 使能 CAN2 接受 FIFO0 中断
    {
        Error_Handler();
    }
}

/**
 * @brief CAN接收中断函数
 * @param hcan CAN通道
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    //test3++;
    //uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if(hcan->Instance == CAN1){
			if((rx_header.StdId == Motor_1_ID)||(rx_header.StdId == Motor_2_ID)){//这里改成用定义，修改太麻烦了
				CAN_Input=1;//标志位 置1
				// 发送通知唤醒任务（CMSIS
				//osSignalSet(CAN_input_taskHandle, 0x1); //非中断安全API，后期还得改
		    
		 }
    }
}

// CAN2数据处理（FIFO1中断）
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if(hcan->Instance == CAN2){
        // 处理0x201（M2006）、0x203-0x204（发射M3508）、0x205（云台Pitch）
			
				if((rx_header.StdId == Motor_1_ID)||(rx_header.StdId == Motor_2_ID)){//这里改成用定义，修改太麻烦了
				CAN_Input=1;//标志位 置1
				// 发送通知唤醒任务（CMSIS
				//osSignalSet(CAN_input_taskHandle, 0x1); //非中断安全API，后期还得改
		    }
    }
}


