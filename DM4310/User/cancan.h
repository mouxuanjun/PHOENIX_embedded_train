//
// Created by 29812 on 25-5-14.
//

#ifndef CANCAN_H
#define CANCAN_H
#include "can.h"
#include "task_cpp.h"
#endif //CANCAN_H


extern DM4310 motor;
inline void CAN_Filter_Init(void)
{
    CAN_FilterTypeDef filterConfig;
    filterConfig.FilterActivation = ENABLE;
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

    filterConfig.FilterIdHigh = 0x0000;
    filterConfig.FilterIdLow = 0x0000;

    filterConfig.FilterMaskIdHigh = 0x0000;
    filterConfig.FilterMaskIdLow = 0x0000;
    //RX端初始化
    filterConfig.FilterBank = 0;
    filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &filterConfig);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  //使能各种中断


}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1){
        CAN_RxHeaderTypeDef rx_header;
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header,motor.status_buffer);
        motor.Deserialize_Status(motor.status_buffer);
    }
}