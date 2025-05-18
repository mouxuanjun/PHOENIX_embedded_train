//
// Created by 22560 on 25-4-20.
//

#ifndef MIDDLEWARE_CAN_HPP
#define MIDDLEWARE_CAN_HPP

#include "main.h"

class Class_CAN {
public:
    CAN_HandleTypeDef *CPP_hcan;
    uint8_t CPP_CAN_RxBuffer[8];
    uint32_t CPP_CAN_Rx_CAN_id;
    uint8_t CPP_CAN_TxBuffer[8];

    void CPP_CAN_Init(CAN_HandleTypeDef *hcan);
    void CPP_CAN_Filter_Init();
    void CPP_CAN_Rx_Data();
    void CPP_CAN_Tx_Data(uint8_t TxData[], uint32_t CAN_id);
};

#endif //MIDDLEWARE_CAN_HPP
