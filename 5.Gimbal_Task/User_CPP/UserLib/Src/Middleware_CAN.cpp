//
// Created by 22560 on 25-4-20.
//

#include "can.h"
#include "Middleware_CAN.hpp"
#include "Device_DM4310.hpp"
#include "Middleware_CAN.h"
#include "Device_GM6020.hpp"
#include "stm32f4xx_hal_can.h"

extern DM4310 DM4310_1;
extern DM4310 DM4310_2;
extern DM4310 DM4310_3;
extern DM4310 DM4310_4;
extern DM4310 DM4340_2;
extern GM6020_All GM6020_All1;

void Class_CAN::CPP_CAN_Filter_Init() {
    if (CPP_hcan->Instance == CAN1) {
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

        HAL_CAN_ConfigFilter(CPP_hcan, &can1_filter_st);
        HAL_CAN_ActivateNotification(CPP_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        HAL_CAN_Start(CPP_hcan);
    }
    else if (CPP_hcan->Instance == CAN2) {
        CAN_FilterTypeDef can2_filter_st;
        can2_filter_st.FilterIdHigh = 0x0000;
        can2_filter_st.FilterIdLow = 0x0000;
        can2_filter_st.FilterMaskIdHigh = 0x0000;
        can2_filter_st.FilterMaskIdLow = 0x0000;
        can2_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
        can2_filter_st.FilterActivation = ENABLE;
        can2_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
        can2_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
        can2_filter_st.FilterBank = 14;
        can2_filter_st.SlaveStartFilterBank = 14;

        HAL_CAN_ConfigFilter(CPP_hcan, &can2_filter_st);
        HAL_CAN_ActivateNotification(CPP_hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
        HAL_CAN_Start(CPP_hcan);
    }
}

void Class_CAN::CPP_CAN_Init(CAN_HandleTypeDef* hcan) {
    CPP_hcan = hcan;
    CPP_CAN_Filter_Init();
}

void Class_CAN::CPP_CAN_Rx_Data() {
    CAN_RxHeaderTypeDef RxHeader;

    if (CPP_hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(CPP_hcan, CAN_RX_FIFO0, &RxHeader, CPP_CAN_RxBuffer);
        CPP_CAN_Rx_CAN_id = RxHeader.StdId;
    }
    else if (CPP_hcan->Instance == CAN2) {
        HAL_CAN_GetRxMessage(CPP_hcan, CAN_RX_FIFO1, &RxHeader, CPP_CAN_RxBuffer);
        CPP_CAN_Rx_CAN_id = RxHeader.StdId;
    }
}

void Class_CAN::CPP_CAN_Tx_Data(uint8_t TxData[], uint32_t CAN_id) {
    CAN_TxHeaderTypeDef TxHeader;

    TxHeader.IDE = CAN_ID_STD; //使用标准数据帧模式
    TxHeader.RTR = CAN_RTR_DATA; //数据帧类型
    TxHeader.DLC = 0x08; //数据帧长度

    TxHeader.StdId = CAN_id;
    CPP_CAN_TxBuffer[0] = TxData[0];
    CPP_CAN_TxBuffer[1] = TxData[1];
    CPP_CAN_TxBuffer[2] = TxData[2];
    CPP_CAN_TxBuffer[3] = TxData[3];
    CPP_CAN_TxBuffer[4] = TxData[4];
    CPP_CAN_TxBuffer[5] = TxData[5];
    CPP_CAN_TxBuffer[6] = TxData[6];
    CPP_CAN_TxBuffer[7] = TxData[7];

    //逐个信箱发送
    if (HAL_CAN_AddTxMessage(CPP_hcan, &TxHeader, CPP_CAN_TxBuffer, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
        if (HAL_CAN_AddTxMessage(CPP_hcan, &TxHeader, CPP_CAN_TxBuffer, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
            HAL_CAN_AddTxMessage(CPP_hcan, &TxHeader, CPP_CAN_TxBuffer, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
}

Class_CAN CPP_CAN1;
Class_CAN CPP_CAN2;

extern "C" {
void CAN_Init(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN1) {
        CPP_CAN1.CPP_CAN_Init(hcan);
    }
    else if (hcan->Instance == CAN2) {
        CPP_CAN2.CPP_CAN_Init(hcan);
    }
}

void CAN_Rx_Data(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN1) {
        CPP_CAN1.CPP_CAN_Rx_Data();
    }
    else if (hcan->Instance == CAN2) {
        CPP_CAN2.CPP_CAN_Rx_Data();
    }
}

void CAN_Tx_Data(CAN_HandleTypeDef* hcan, uint8_t TxData[], uint32_t CAN_id) {
    if (hcan->Instance == CAN1) {
        CPP_CAN1.CPP_CAN_Tx_Data(TxData, CAN_id);
    }
    else if (hcan->Instance == CAN2) {
        CPP_CAN2.CPP_CAN_Tx_Data(TxData, CAN_id);
    }
}

void CAN_GetData(CAN_HandleTypeDef* hcan, uint8_t RxData[], uint32_t* CAN_id) {
    if (hcan->Instance == CAN1) {
        *CAN_id = CPP_CAN1.CPP_CAN_Rx_CAN_id;
        RxData[0] = CPP_CAN1.CPP_CAN_RxBuffer[0];
        RxData[1] = CPP_CAN1.CPP_CAN_RxBuffer[1];
        RxData[2] = CPP_CAN1.CPP_CAN_RxBuffer[2];
        RxData[3] = CPP_CAN1.CPP_CAN_RxBuffer[3];
        RxData[4] = CPP_CAN1.CPP_CAN_RxBuffer[4];
        RxData[5] = CPP_CAN1.CPP_CAN_RxBuffer[5];
        RxData[6] = CPP_CAN1.CPP_CAN_RxBuffer[6];
        RxData[7] = CPP_CAN1.CPP_CAN_RxBuffer[7];
    }
    else if (hcan->Instance == CAN2) {
        *CAN_id = CPP_CAN2.CPP_CAN_Rx_CAN_id;
        RxData[0] = CPP_CAN2.CPP_CAN_RxBuffer[0];
        RxData[1] = CPP_CAN2.CPP_CAN_RxBuffer[1];
        RxData[2] = CPP_CAN2.CPP_CAN_RxBuffer[2];
        RxData[3] = CPP_CAN2.CPP_CAN_RxBuffer[3];
        RxData[4] = CPP_CAN2.CPP_CAN_RxBuffer[4];
        RxData[5] = CPP_CAN2.CPP_CAN_RxBuffer[5];
        RxData[6] = CPP_CAN2.CPP_CAN_RxBuffer[6];
        RxData[7] = CPP_CAN2.CPP_CAN_RxBuffer[7];
    }
}

//can中断回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN1) {
        CPP_CAN1.CPP_CAN_Rx_Data();
        switch(CPP_CAN1.CPP_CAN_Rx_CAN_id) {
            case 0x205:case 0x206:case 0x207:case 0x208: {
                GM6020_All1.Update();
                break;
            }
            case 0x016:case 0x116:case 0x216:{
                DM4310_1.Update();
                break;
            }
            case 0x012:case 0x112:case 0x212:{
                DM4310_2.Update();
                DM4340_2.Update();
                break;
            }
            case 0x013:case 0x113:case 0x213:{
                DM4310_3.Update();
                break;
            }
            case 0x014:case 0x114:case 0x214:{
                DM4310_4.Update();
                break;
            }
            default: {
                break;
            }
        }
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN2) {
        CPP_CAN2.CPP_CAN_Rx_Data();

        switch(CPP_CAN2.CPP_CAN_Rx_CAN_id) {
            case 0x016:case 0x116:case 0x216:{
                DM4310_1.Update();
                break;
            }
            default: break;
        }
    }
}

}