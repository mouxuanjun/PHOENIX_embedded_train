#ifndef __dr16_H__
#define __dr16_H__

#include "main.h"

typedef struct{
	struct{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    char s1;
    char s2;
	  int16_t wheel;
	} rc;
	
	struct{
		int16_t mouse_x;
		int16_t mouse_y;
		char mouse_press_l;
	  char mouse_press_r;
	} mouse;
	
	struct{
		uint16_t W : 1;
		uint16_t S : 1;
		uint16_t D : 1;
		uint16_t A : 1;
		uint16_t shift : 1;
		uint16_t ctrl : 1;
		uint16_t P : 1;
		uint16_t E : 1;
		uint16_t R : 1;
		uint16_t F : 1;
		uint16_t G : 1;
		uint16_t Z : 1;
		uint16_t C : 1;
		uint16_t V : 1;
		uint16_t B : 1;
		int16_t Null;
	} key;
} rc_info_t;

extern rc_info_t rc_ctrl;

void RemoteDataProcess(rc_info_t *rc_ctrl, uint8_t rx_data[18]);

#endif
