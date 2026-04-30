#pragma once

/**
 * @brief 数学工具库
 * @note 包含限幅、大小端转换、映射及常用常数
 */

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <math.h>
#include "arm_math.h"

// Eigen 库警告处理
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-result"
#include "Eigen/Dense"
#pragma clang diagnostic pop

/* Exported macros -----------------------------------------------------------*/

#ifndef PI
#define PI 3.14159265358979f
#endif

// 常用转换常数
#define RPM_TO_RADPS      (2.0f * PI / 60.0f)
#define DEG_TO_RAD        (PI / 180.0f)
#define RAD_TO_DEG        (180.0f / PI)
#define CELSIUS_TO_KELVIN (273.15f)

/* Exported function declarations --------------------------------------------*/

// 基础逻辑
void Math_Boolean_Logical_Not(bool* Value);

// 大小端转换重载
void Math_Endian_Reverse_16(void* Address);
uint16_t Math_Endian_Reverse_16(void* Source, void* Destination);
void Math_Endian_Reverse_32(void* Address);
uint32_t Math_Endian_Reverse_32(void* Source, void* Destination);

// 校验和计算 (使用模板支持不同类型)
template <typename T>
T Math_Sum(T* Address, uint32_t Length) {
    T sum = 0;
    for (uint32_t i = 0; i < Length; i++) {
        sum += Address[i];
    }
    return sum;
}

// 映射函数
float Math_Sinc(float x);
int32_t Math_Float_To_Int(float x, float Float_1, float Float_2, int32_t Int_1, int32_t Int_2);
float Math_Int_To_Float(int32_t x, int32_t Int_1, int32_t Int_2, float Float_1, float Float_2);

/* Template Functions --------------------------------------------------------*/

/**
 * @brief 限幅函数
 */
template <typename T>
T Math_Constrain(T* x, T Min, T Max) {
    if (*x < Min) *x = Min;
    else if (*x > Max) *x = Max;
    return *x;
}

/**
 * @brief 绝对值
 */
template <typename T>
T Math_Abs(T x) {
    return (x >= 0) ? x : -x;
}

/**
 * @brief 角度/数值归一化 (取模归化到 ±modulus/2)
 */
template <typename T>
T Math_Modulus_Normalization(T x, T modulus) {
    T half_modulus = modulus / (T)2;
    T tmp = fmod(x + half_modulus, modulus);
    if (tmp < 0) tmp += modulus;
    return tmp - half_modulus;
}