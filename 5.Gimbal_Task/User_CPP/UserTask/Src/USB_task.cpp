//
// Created by 22560 on 25-4-18.
//

#include "USB_task.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"

class USB_Data {
public:
    uint8_t CPP_myUSBRxData[64];
    uint16_t CPP_myUSBRxNum;
    uint8_t CPP_myUSBTxData[64];
    uint16_t CPP_myUSBTxNum;
    usb_msg_t CPP_msg_from_minipc;
    usb_msg_t CPP_msg_to_minipc;

    void CPP_USBData_Init(void);
    void CPP_USBData_Process(void);
    void CPP_USBData_GetData(uint8_t* Buf, uint32_t* Len);
    void CPP_USBData_SendData(uint8_t* Buf, uint32_t* Len);
    void CPP_USBData_SendMsg(char find_bool, float yaw, float pitch);
};

/**
 * @brief 初始化接收来自视觉数据的帧格式配置
 */
void USB_Data::CPP_USBData_Init() {
    CPP_msg_from_minipc.start = 's';
    CPP_msg_from_minipc.end = 'e';
    CPP_msg_to_minipc.start = 's';
    CPP_msg_to_minipc.end = 'e';
    CPP_msg_from_minipc.type = 0xA0;
    CPP_msg_to_minipc.type = 0xB0;

    CPP_msg_to_minipc.find_bool = 0;
    CPP_msg_to_minipc.yaw = 0;
    CPP_msg_to_minipc.pitch = 0;
    CPP_msg_from_minipc.find_bool = 0;
    CPP_msg_from_minipc.yaw = 0;
    CPP_msg_from_minipc.pitch = 0;
}

/**
 * @brief 处理
 */
void USB_Data::CPP_USBData_Process(void) {
    if (CPP_myUSBRxNum) {
        if (CPP_myUSBRxData[1] == 0xA0) {
            CPP_msg_from_minipc.start = CPP_myUSBRxData[0];
            CPP_msg_from_minipc.type = CPP_myUSBRxData[1];
            CPP_msg_from_minipc.find_bool = CPP_myUSBRxData[2];
            CPP_msg_from_minipc.yaw = *((float*)&(CPP_myUSBRxData[3]));
            CPP_msg_from_minipc.pitch = *((float*)&(CPP_myUSBRxData[7]));
            CPP_msg_from_minipc.end = CPP_myUSBRxData[31];
            memset(CPP_myUSBRxData, 0, 64); //数据处理后清空缓存区
            CPP_myUSBRxNum = 0; //有利于判断，置0
        }
        memset(CPP_myUSBRxData, 0, 64); //数据处理后清空缓存区
        CPP_myUSBRxNum = 0; //有利于判断，置0
    }
}

void USB_Data::CPP_USBData_GetData(uint8_t* Buf, uint32_t* Len) {
    memset(CPP_myUSBRxData, 0, 64); //清空缓存区
    memcpy(CPP_myUSBRxData, Buf, *Len); //接收的数据复制到缓存区
    CPP_myUSBRxNum = *Len; //复制字节数
}

void USB_Data::CPP_USBData_SendMsg(char find_bool, float yaw, float pitch) {
    CPP_msg_to_minipc.start = 's';
    CPP_msg_to_minipc.end = 'e';
    CPP_msg_to_minipc.type = 0xB0;
    CPP_msg_to_minipc.yaw = yaw;
    CPP_msg_to_minipc.pitch = pitch;
    CPP_msg_to_minipc.find_bool = find_bool;

    char float_yaw[4], float_pitch[4];
    float_yaw[0] = *(char*)(&CPP_msg_to_minipc.yaw);
    float_yaw[1] = *((char*)(&CPP_msg_to_minipc.yaw) + 1);
    float_yaw[2] = *((char*)(&CPP_msg_to_minipc.yaw) + 2);
    float_yaw[3] = *((char*)(&CPP_msg_to_minipc.yaw) + 3);
    float_pitch[0] = *(char*)(&CPP_msg_to_minipc.pitch);
    float_pitch[1] = *((char*)(&CPP_msg_to_minipc.pitch) + 1);
    float_pitch[2] = *((char*)(&CPP_msg_to_minipc.pitch) + 2);
    float_pitch[3] = *((char*)(&CPP_msg_to_minipc.pitch) + 3);

    CPP_myUSBTxNum = 32;
    CPP_myUSBTxData[0] = CPP_msg_to_minipc.start;
    CPP_myUSBTxData[1] = CPP_msg_to_minipc.type;
    CPP_myUSBTxData[2] = float_yaw[0];
    CPP_myUSBTxData[3] = float_yaw[1];
    CPP_myUSBTxData[4] = float_yaw[2];
    CPP_myUSBTxData[5] = float_yaw[3];
    CPP_myUSBTxData[6] = float_pitch[0];
    CPP_myUSBTxData[7] = float_pitch[1];
    CPP_myUSBTxData[8] = float_pitch[2];
    CPP_myUSBTxData[9] = float_pitch[3];
    for (char i = 10; i < 30; i++) { //中间留空
        CPP_myUSBTxData[i] = 0x00;
    }
    CPP_myUSBTxData[31] = CPP_msg_to_minipc.end;

    CDC_Transmit_FS(CPP_myUSBTxData, CPP_myUSBTxNum);
    memset(CPP_myUSBTxData, 0, 64); //数据处理后清空缓存区
    CPP_myUSBTxNum = 0; //有利于判断，置0
}


void USB_Data::CPP_USBData_SendData(uint8_t* Buf, uint32_t* Len) {
    memset(CPP_myUSBTxData, 0, 64);
    memcpy(CPP_myUSBTxData, Buf, *Len);
    CPP_myUSBTxNum = *Len;
    CDC_Transmit_FS(Buf, *Len);
    memset(CPP_myUSBTxData, 0, 64);
}

USB_Data USB_Data1;

void CPP_USB_Task() {
    uint32_t TxLen = 10;
    USB_Data1.CPP_USBData_Init();

    while (1) {
        osDelay(2);
        USB_Data1.CPP_USBData_Process();
        USB_Data1.CPP_USBData_SendMsg(0,
                                      3.145f,
                                      5.314f);
    }
}

extern "C" {
    void USB_Task(void const* argument) {
        CPP_USB_Task();
    }

    void USBData_Process() {
        USB_Data1.CPP_USBData_Process();
    }

    void USBData_GetData(uint8_t* Buf, uint32_t* Len) {
        USB_Data1.CPP_USBData_GetData(Buf, Len);
    }

    void USBData_SendData(uint8_t* Buf, uint32_t* Len) {
        USB_Data1.CPP_USBData_SendData(Buf, Len);
    }
}
