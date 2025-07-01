#include "dr_can.h"
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

//void DM4310_Control(uint16_t id, float angle, float speed, float Kp, float Kd, float torque){ 
//	uint8_t tx_data[8];
//    CAN_TxHeaderTypeDef tx_header;
//    uint16_t a_tmp, v_tmp, kp_tmp, kd_tmp, tor_tmp;
//    a_tmp = float_to_uint(angle, P_MIN, P_MAX, 16);
//    v_tmp = float_to_uint(speed, V_MIN, V_MAX, 12);
//    kp_tmp = float_to_uint(Kp, KP_MIN, KP_MAX, 12);
//    kd_tmp = float_to_uint(Kd, KD_MIN, KD_MAX, 12);
//    tor_tmp = float_to_uint(torque, T_MIN, T_MAX, 12);

//    tx_header.StdId = 0x02;
//    tx_header.IDE = CAN_ID_STD;
//    tx_header.RTR = CAN_RTR_DATA;
//    tx_header.DLC = 8;

//    tx_data[0] = (a_tmp >> 8);
//    tx_data[1] = a_tmp;
//    tx_data[2] = (v_tmp >> 4);
//    tx_data[3] = ((v_tmp & 0xF) << 4) | (kp_tmp >> 8);
//    tx_data[4] = kp_tmp;
//    tx_data[5] = (kd_tmp >> 4);
//    tx_data[6] = ((kd_tmp & 0xF) << 4)|(tor_tmp >> 8);
//    tx_data[7] = tor_tmp;

//	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);  
//}
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
