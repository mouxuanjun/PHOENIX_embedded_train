#include <stdint.h>

// 滤波器回调函数类型
typedef void (*filter_callback_t)(float input, float output, void* user_data);

// 滤波器结构体
typedef struct {
    filter_callback_t callback;  // 回调函数指针
    void* user_data;            // 用户自定义数据的地址
    // 滤波器参数...
} filter_t;