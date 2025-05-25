#include "bsp_can.h"

/**
 * @brief CAN的滤波器
 */
void Filter_Init(void){
    CAN_FilterTypeDef can1_filter_st;
	CAN_FilterTypeDef can2_filter_st;

    can1_filter_st.FilterIdHigh = 0x0000;
    can1_filter_st.FilterIdLow = 0x0000;
    can1_filter_st.FilterMaskIdHigh = 0x0000;
    can1_filter_st.FilterMaskIdLow = 0x0000;
    can1_filter_st.FilterActivation = ENABLE;
    can1_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can1_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can1_filter_st.FilterBank = 0;
	can1_filter_st.SlaveStartFilterBank = 14;
    can1_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

	can2_filter_st.FilterIdHigh = 0x0000;
    can2_filter_st.FilterIdLow = 0x0000;
    can2_filter_st.FilterMaskIdHigh = 0x0000;
    can2_filter_st.FilterMaskIdLow = 0x0000;
    can2_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can2_filter_st.FilterActivation = ENABLE;
    can2_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can2_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can2_filter_st.FilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &can1_filter_st);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan1);
	HAL_Delay(5);
	HAL_CAN_ConfigFilter(&hcan2, &can2_filter_st);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
}

/**
 * @brief GM3508电机的控制
 * @param GM3508的电机的目标速度
 * @note 电机ID和解算
 */
void GM3508_Control(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4){
	uint8_t Data[8];
	CAN_TxHeaderTypeDef tx_header;

	tx_header.StdId = 0X200;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 8;

	Data[0] = (motor1 >> 8) & 0xFF;
	Data[1] = motor1 & 0xFF;
	Data[2] = (motor2 >> 8) & 0xFF;
	Data[3] = motor2 & 0xFF;
	Data[4] = (motor3 >> 8) & 0xFF;
	Data[5] = motor3 & 0xFF;
	Data[6] = (motor4 >> 8) & 0xFF;
	Data[7] = motor4 & 0xFF;

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, Data, (uint32_t *)CAN_TX_MAILBOX0);
}
