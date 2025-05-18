/**
 *@brief 只用于cpp文件的数学运算
 */
#ifndef ALGORITHM_MATH_HPP
#define ALGORITHM_MATH_HPP

#include "stdint.h"
#include "limits.h"
#include "math.h"
#include "stdint.h"
#include "float.h"

#define PI (3.14159265f)

void Math_Endian_Reverse_16(void* Address);
void Math_Endian_Reverse_16(void* Source, void* Destination);
void Math_Endian_Reverse_32(void* Address);
void Math_Endian_Reverse_32(void* Source, void* Destination);

uint8_t Math_Sum_8(uint8_t* Address, uint32_t Length);
uint16_t Math_Sum_16(uint16_t* Address, uint32_t Length);
uint32_t Math_Sum_32(uint32_t* Address, uint32_t Length);

float Math_Sinc(float x);

/**
 * @brief 限幅函数
 *
 * @tparam Type
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 */
template <typename Type>
void Math_Constrain(Type* x, Type Min, Type Max) {
    if (*x < Min) {
        *x = Min;
    }
    else if (*x > Max) {
        *x = Max;
    }
}

/**
 * @brief 求绝对值
 *
 * @tparam Type
 * @param x 传入数据
 * @return Type x的绝对值
 */
template <typename Type>
Type Math_Abs(Type x) {
    return ((x > 0) ? x : -x);
}

/**
 * @brief 限幅函数
 *
 * @tparam Type
 * @param x 传入数据
 * @return Type x的绝对值
 */
template <typename T>
void Math_Constrain(T& value, T min, T max) {
    if (value < min) {
        value = min;
    } else if (value > max) {
        value = max;
    }
}

#endif //ALGORITHM_MATH_HPP
