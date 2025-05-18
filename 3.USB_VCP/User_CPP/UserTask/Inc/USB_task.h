//
// Created by 22560 on 25-4-18.
//

#ifndef USB_TASK_H
#define USB_TASK_H
#include "main.h"

//上位机发报
typedef struct {
    char start ; //0 帧头取 's'
    char type; //1 消息类型：上->下：0xA0
    char find_bool; //2 是否追踪
    float yaw; //3-6 yaw数据
    float pitch; //7-10 pitch数据
    char end; //31 帧尾取'e'
} usb_msg_t;
//下位机发报
typedef struct {
    char start ; //0 帧头取 's'
    char type; //1 消息类型：下->上：0xB0
    float yaw; //2-5 yaw数据
    float pitch; //6-9 pitch数据
    char end; //31 帧尾取'e'
} usb_msgRX_t;

#ifdef __cplusplus
extern "C"{
#endif
    void USB_Task(void const * argument);

    void USBData_Process();

    void USBData_GetData(uint8_t* Buf, uint32_t *Len);

    void USBData_SendData(uint8_t* Buf, uint32_t *Len);
#ifdef __cplusplus
    }
#endif

#endif //USB_TASK_H
