//
// Created by 22560 on 25-4-20.
//

#ifndef MIDDLEWARE_CAN_H
#define MIDDLEWARE_CAN_H
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

    void CAN_Init(CAN_HandleTypeDef *hcan);

    void CAN_Rx_Data(CAN_HandleTypeDef *hcan);

    void CAN_Tx_Data(CAN_HandleTypeDef *hcan,uint8_t TxData[], uint32_t CAN_id);

    void CAN_GetData(CAN_HandleTypeDef *hcan,uint8_t RxData[], uint32_t *CAN_id);


#ifdef __cplusplus
}
#endif


#endif //MIDDLEWARE_CAN_H
