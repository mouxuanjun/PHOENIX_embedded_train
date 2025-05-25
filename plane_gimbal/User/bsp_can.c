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
 * @brief 读取GM6020电机的参数
 * @param can通信的选择
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8]; 
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId){
    case 0x208:
		GM6020_RxData(rx_header.StdId, rx_data);
        break;
    case 0x11:
        DM4310_RxData(rx_header.StdId, rx_data);
        break;
//	case 0x201:
//		GM3508_RxData(rx_header.StdId, rx_data);
//		break;
//	case 0x202:
//		GM3508_RxData(rx_header.StdId, rx_data);
//		break;
	}
}

/**
 * @brief GM6020电机的控制
 * @param GM6020的电机的目标速度
 * @note 电机ID和解算
 */
void GM6020_Control(int16_t motor){
	uint8_t target_Torque[8];
	CAN_TxHeaderTypeDef tx_header;

	tx_header.StdId = 0X1FF;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 8;

	target_Torque[6] = (motor >> 8) & 0xFF;
	target_Torque[7] = motor & 0xFF;

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, target_Torque, (uint32_t *)CAN_TX_MAILBOX0);  
}

/**
 * @brief DM4310电机的控制
 * @param DM4310的电机的目标速度
 * @note 电机ID和解算
 */
void DM4310_Control(uint16_t id, float angle, float velocity, float Kp, float Kd, float torque){ 
	uint8_t tx_data[8] = {0};
    CAN_TxHeaderTypeDef tx_header;
    uint16_t tor_tmp;
    tor_tmp = float_to_uint(torque, T_MIN, T_MAX, 12);

    tx_header.StdId = 0x01;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    tx_data[6] = (tor_tmp >> 8);
    tx_data[7] = tor_tmp;

	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);  
}

/**
 * @brief GM3508电机的控制
 * @param GM3508的电机的目标速度
 * @note 电机ID和解算
 */
void GM3508_Control(int16_t motor1, int16_t motor2){
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

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, Data, (uint32_t *)CAN_TX_MAILBOX0);
}
