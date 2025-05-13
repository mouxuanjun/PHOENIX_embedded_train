#include "rc_can.h"
#include "can.h"
#include "GM6020.h"

void Filter_Init(void){
    CAN_FilterTypeDef can1_filter_st;

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
	  
	  HAL_CAN_ConfigFilter(&hcan1, &can1_filter_st);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan1);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8]; 
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if(hcan -> Instance == CAN1){
		if (rx_header.StdId == 0x205){
		  GM6020_RxData(rx_header.StdId, rx_data);
		}
	}
}

void GM6020_Control(int16_t motor1){
	CAN_TxHeaderTypeDef tx_header;
	
  tx_header.StdId = 0X1FF;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;

	target_Torque[0] = (motor1 >> 8) & 0xFF;
	target_Torque[1] = motor1 & 0xFF;

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, target_Torque, (uint32_t *)CAN_TX_MAILBOX0);  
}
