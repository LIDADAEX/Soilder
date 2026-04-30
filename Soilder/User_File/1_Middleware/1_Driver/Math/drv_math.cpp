/**
 * @brief 数学工具库实现
 */

#include "drv_math.h"

/**
 * @brief 布尔值取反
 */
void Math_Boolean_Logical_Not(bool* Value) {
    if (Value == nullptr) return;
    *Value = !(*Value);
}

/**
 * @brief 16位大小端原地转换
 */
void Math_Endian_Reverse_16(void* Address) {
    uint8_t* p = (uint8_t*)Address;
    uint8_t tmp = p[0];
    p[0] = p[1];
    p[1] = tmp;
}

/**
 * @brief 16位大小端转换并拷贝
 */
uint16_t Math_Endian_Reverse_16(void* Source, void* Destination) {
    uint8_t* src = (uint8_t*)Source;
    uint16_t result = (uint16_t)(src[0] << 8 | src[1]);

    if (Destination != nullptr) {
        uint8_t* dst = (uint8_t*)Destination;
        dst[0] = src[1];
        dst[1] = src[0];
    }
    return result;
}

/**
 * @brief 32位大小端原地转换
 */
void Math_Endian_Reverse_32(void* Address) {
    uint8_t* p = (uint8_t*)Address;
    uint8_t tmp;
    // 交换 0,3
    tmp = p[0]; p[0] = p[3]; p[3] = tmp;
    // 交换 1,2
    tmp = p[1]; p[1] = p[2]; p[2] = tmp;
}

/**
 * @brief 32位大小端转换并拷贝
 */
uint32_t Math_Endian_Reverse_32(void* Source, void* Destination) {
    uint8_t* src = (uint8_t*)Source;
    uint32_t result = (uint32_t)(src[0] << 24 | src[1] << 16 | src[2] << 8 | src[3]);

    if (Destination != nullptr) {
        uint8_t* dst = (uint8_t*)Destination;
        dst[0] = src[3];
        dst[1] = src[2];
        dst[2] = src[1];
        dst[3] = src[0];
    }
    return result;
}

/**
 * @brief Sinc 函数实现 (sin(x)/x)
 */
float Math_Sinc(float x) {
    if (Math_Abs(x) <= FLT_EPSILON) {
        return 1.0f;
    }
    return (arm_sin_f32(x) / x);
}

/**
 * @brief 浮点数映射到整型区间
 */
int32_t Math_Float_To_Int(float x, float Float_1, float Float_2, int32_t Int_1, int32_t Int_2) {
    if (Math_Abs(Float_2 - Float_1) <= FLT_EPSILON) return Int_1;
    float ratio = (x - Float_1) / (Float_2 - Float_1);
    return (int32_t)(ratio * (float)(Int_2 - Int_1) + (float)Int_1);
}

/**
 * @brief 整型映射到浮点数区间
 */
float Math_Int_To_Float(int32_t x, int32_t Int_1, int32_t Int_2, float Float_1, float Float_2) {
    if (Int_2 == Int_1) return Float_1;
    float ratio = (float)(x - Int_1) / (float)(Int_2 - Int_1);
    return ratio * (Float_2 - Float_1) + Float_1;
}