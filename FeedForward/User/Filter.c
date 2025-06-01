#include <stdio.h>
#include "Filter.h"

/**
* @brief 滤波器回调函数，用于方便的选取滤波器
*
*
* @note 具体逻辑是这样：以规定的格式实现一系列的滤波器，调用的时候使用标准的滤波器处理函数filter_process，然后往里面填滤波器具体实现的函数名地址，输入数据地址和滤波器参数结构体。
*/
// 初始化滤波器
void filter_init(filter_t* filter, filter_callback_t callback, void* user_data) {
    filter->callback = callback;
    filter->user_data = user_data;
    
    // TODO: 在这里初始化您的滤波器参数
}

// 滤波器处理函数
float filter_process(filter_t* filter, float input) {
    float output;
    
    // TODO: 在这里实现您的滤波算法
    // 例如：
    // output = your_filter_algorithm(input);
    output = input; // 临时直通，等待您的实现
    
    // 调用回调函数
    if (filter->callback != NULL) {
        filter->callback(input, output, filter->user_data);
    }
    
    return output;
}
//////滤波器具体定义//////////
