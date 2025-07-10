#include "6020.h"
#include "main.h"

void motor_read(Motor6020rx *motor,uint8_t *data){//读取6020电机数据

    motor->angle=(data[0]<<8)|data[1];
    motor->speed=(data[2]<<8)|data[3];
    motor->current=(data[4]<<8)|data[5];
    motor->temperature=data[6];
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){//中断函数，CAN接收到数据就存入motor6020变量
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

    if(RxHeader.StdId == 0x208){
        motor_read(&motor6020rx,RxData);

    }   
}

CAN_TxHeaderTypeDef TxHeader;
uint8_t can_send[8];

HAL_StatusTypeDef motor_write(uint32_t stdid,int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4){
    TxHeader.StdId = stdid;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x08;
    
    can_send[0] = motor1>>8;
    can_send[1] = motor1;
    can_send[2] = motor2>>8;
    can_send[3] = motor2;
    can_send[4] = motor3>>8;
    can_send[5] = motor3;
    can_send[6] = motor4>>8;
    can_send[7] = motor4;

    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, can_send, (uint32_t*)CAN_TX_MAILBOX0);
}