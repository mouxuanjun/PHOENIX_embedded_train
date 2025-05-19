#include "bsp_can.h"

/**
 * @file BSP_Can.c
 * @brief 初始化筛选器（这里显码和掩码都是0x0000）
 * @author HWX
 * @date 2024/10/20
 */
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
    if (HAL_CAN_ConfigFilter(&hcan1, &can1_filter_st) != HAL_OK)// 配置 CAN1 过滤器
    {
        Error_Handler();  // 处理错误
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)// 启动 CAN1
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)// 使能 CAN1 接收 FIFO0 消息中断
    {
        Error_Handler();
    }
}

/**
 * @file BSP_Can.c
 * @brief CAN接受中断函数
 * @param hcan CAN通道
 * @author HWX
 * @date 2024/10/20
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if(hcan->Instance == CAN1)
    {
        if(rx_header.StdId == 0x202)
        {
            Get_GM6020_Motor_Message(rx_header.StdId,rx_data);
        }
    }
}

