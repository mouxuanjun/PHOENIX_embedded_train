//
// Created by 22560 on 25-4-13.
//

#include "DR16_Remote.hpp"
extern uint8_t RC_Data[18];

class DR16 {
public:
    void RC_Processing_Data();
    RC_t _RC;
    void RC_Getdata(RC_t* RC_temp);
};

void DR16 :: RC_Processing_Data() {
    _RC.ch0 = (((int16_t)RC_Data[0] | ((int16_t)RC_Data[1] << 8)) & 0x07FF)-1024;
    _RC.ch1 = ((((int16_t)RC_Data[1] >> 3) | ((int16_t)RC_Data[2] << 5)) & 0x07FF)-1024;
    _RC.ch2 = ((((int16_t)RC_Data[2] >> 6) | ((int16_t)RC_Data[3] << 2) |((int16_t)RC_Data[4] << 10)) & 0x07FF)-1024;
    _RC.ch3 = ((((int16_t)RC_Data[4] >> 1) | ((int16_t)RC_Data[5]<<7)) & 0x07FF)-1024;
    _RC.s1 = ((RC_Data[5] >> 4) & 0x000C) >> 2;
    _RC.s2 = ((RC_Data[5] >> 4) & 0x0003);
    _RC.x = ((int16_t)RC_Data[6]) | ((int16_t)RC_Data[7] << 8);
    _RC.y = ((int16_t)RC_Data[8]) | ((int16_t)RC_Data[9] << 8);
    _RC.z = ((int16_t)RC_Data[10]) | ((int16_t)RC_Data[11] << 8);
    _RC.press_l = RC_Data[12];
    _RC.press_r = RC_Data[13];
    _RC.key = ((int16_t)RC_Data[14]) | ((int16_t)RC_Data[15] << 8);
    _RC.wheel = ((int16_t)RC_Data[16] | (int16_t)RC_Data[17] << 8) - 1024;
}

void DR16 :: RC_Getdata(RC_t* RC_temp) {
    DR16::RC_Processing_Data();
    RC_temp->ch0 = _RC.ch0;
    RC_temp->ch1 = _RC.ch1;
    RC_temp->ch2 = _RC.ch2;
    RC_temp->ch3 = _RC.ch3;
    RC_temp->s1 = _RC.s1;
    RC_temp->s2 = _RC.s2;
    RC_temp->x = _RC.x;
    RC_temp->y = _RC.y;
    RC_temp->z = _RC.z;
    RC_temp->press_l = _RC.press_l;
    RC_temp->press_r = _RC.press_r;
    RC_temp->key = _RC.key;
    RC_temp->wheel = _RC.wheel;
}

DR16 DR16_1;

//给C封装函数
extern "C"{
    void DR16_GetData(RC_t* RC_temp) {
        DR16_1.RC_Getdata(RC_temp);
    }

    void DR16_Processing_Data() {
        DR16_1.RC_Processing_Data();
    }
}

