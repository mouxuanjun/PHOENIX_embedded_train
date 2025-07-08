#include "BSP_CAN.h"
#include "Motor4310.h"
#include "Motor4310_Driver.h"
#include "LK9025.h"
#define DM4310_ID 0x00

/**
 * @file BSP_Can.c
 * @brief ��ʼ��ɸѡ����������������붼��0x0000��
 * @author HWX
 * @date 2024/10/20
 */
extern LKMotor_Measure_t measure;
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
	//ʹ��CANͨ��
    if (HAL_CAN_ConfigFilter(&hcan1, &can1_filter_st) != HAL_OK)// ���� CAN1 ������
    {
        Error_Handler();  // ��������
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)// ���� CAN1
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)// ʹ�� CAN1 ���� FIFO0 ��Ϣ�ж�
    {
        Error_Handler();
    }
		
		HAL_Delay(5);
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
	//ʹĜCANͨµÀ
    if (HAL_CAN_ConfigFilter(&hcan2, &can2_filter_st) != HAL_OK)// Ťփ CAN1 ¹ýƷ
    {
        Error_Handler();  // ´¦À񛞳
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK)// ƴ¶¯ CAN1
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)// ʹĜ CAN1 ½ӊՠFIFO0 ϻϢ֐¶ύ
    {
        Error_Handler();
    }
}

/**
 * @file BSP_Can.c
 * @brief CAN�����жϺ���
 * @param hcan CANͨ��
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
        if(rx_header.StdId == 0x207)
        {
            Get_GM6020_Motor_Message(rx_header.StdId,rx_data);
        }else if (rx_header.StdId == 0x141){
                    LKMotorDecode(&measure, rx_data);
                }
    }
		if(hcan->Instance == CAN2)
    {
        if(rx_header.StdId == 0x205)
        {
            Get_GM6020_Motor_Message(rx_header.StdId,rx_data);
        }else if (rx_header.StdId  == DM4310_ID){
					switch (rx_data[0]& 0x0F){//取低4位作为id
						case 1:
							dm4310_fbdata(&motor[Motor1], rx_data); 
						break;
						case 2:
							dm4310_fbdata(&motor[Motor1], rx_data);
						break;
						case 3:
							dm4310_fbdata(&motor[Motor1], rx_data);
						break;
						}
				}else if (rx_header.StdId == 0x141||rx_header.StdId == 0x142||rx_header.StdId == 0x181){
                    LKMotorDecode(&measure, rx_data);
                }
                
    }
}

