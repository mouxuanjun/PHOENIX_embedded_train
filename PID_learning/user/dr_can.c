#include "dr_can.h"
#include "GM6020.h"
#include "3508.h"
extern motor_measure_t motor_chassis[7];
void CAN1_Filter_Init(void)
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

    if(HAL_CAN_ConfigFilter(&hcan1, &can1_filter_st) != HAL_OK)
    {
      Error_Handler();
    }
		
    if(HAL_CAN_Start(&hcan1) != HAL_OK)
    {
      Error_Handler();
    }
		
    if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
      Error_Handler();
    }
}
void CAN2_Filter_Init(void)
{

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

    if(HAL_CAN_ConfigFilter(&hcan2, &can2_filter_st) != HAL_OK)
    {
      Error_Handler();
    }
		
    if(HAL_CAN_Start(&hcan2) != HAL_OK)
    {
      Error_Handler();
    }
		
    if(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
      Error_Handler();
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
        case 0x201:
            get_motor_measure(&motor_chassis[0], rx_data);
            break;
        case 0x202:
            get_motor_measure(&motor_chassis[1], rx_data);
            break;
        case 0x203:
            get_motor_measure(&motor_chassis[2], rx_data);
            break;
        case 0x204:
            get_motor_measure(&motor_chassis[3], rx_data);
            break;
        case 0x207:
            Get_GM6020_Motor_Message(rx_header.StdId, rx_data);
            break;
        case 0x205:
            Get_GM6020_Motor_Message(rx_header.StdId, rx_data);
            break;

    }
    
}

