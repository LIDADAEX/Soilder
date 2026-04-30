/**
 * @brief TIM 定时器驱动
 * @note 统一管理所有定时器中断回调，支持 TIM1 - TIM14
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_tim.h"

/* Private variables ---------------------------------------------------------*/

// 使用数组管理对象，以便通过索引或辅助函数快速定位
Struct_TIM_Manage_Object TIM1_Manage_Object;
Struct_TIM_Manage_Object TIM2_Manage_Object;
Struct_TIM_Manage_Object TIM3_Manage_Object;
Struct_TIM_Manage_Object TIM4_Manage_Object;
Struct_TIM_Manage_Object TIM5_Manage_Object;
Struct_TIM_Manage_Object TIM6_Manage_Object;
Struct_TIM_Manage_Object TIM7_Manage_Object;
Struct_TIM_Manage_Object TIM8_Manage_Object;
Struct_TIM_Manage_Object TIM9_Manage_Object;
Struct_TIM_Manage_Object TIM10_Manage_Object;
Struct_TIM_Manage_Object TIM11_Manage_Object;
Struct_TIM_Manage_Object TIM12_Manage_Object;
Struct_TIM_Manage_Object TIM13_Manage_Object;
Struct_TIM_Manage_Object TIM14_Manage_Object;

/* Private Functions ---------------------------------------------------------*/

/**
 * @brief 根据 TIM 实例获取对应的管理对象
 * @param htim 定时器句柄
 * @return Struct_TIM_Manage_Object* 对应的管理对象指针，若未找到则返回 nullptr
 */
static Struct_TIM_Manage_Object* Get_TIM_Manage_Object(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM1)  return &TIM1_Manage_Object;
    if (htim->Instance == TIM2)  return &TIM2_Manage_Object;
    if (htim->Instance == TIM3)  return &TIM3_Manage_Object;
    if (htim->Instance == TIM4)  return &TIM4_Manage_Object;
    if (htim->Instance == TIM5)  return &TIM5_Manage_Object;
    if (htim->Instance == TIM6)  return &TIM6_Manage_Object;
    if (htim->Instance == TIM7)  return &TIM7_Manage_Object;
    if (htim->Instance == TIM8)  return &TIM8_Manage_Object;
    if (htim->Instance == TIM9)  return &TIM9_Manage_Object;
    if (htim->Instance == TIM10) return &TIM10_Manage_Object;
    if (htim->Instance == TIM11) return &TIM11_Manage_Object;
    if (htim->Instance == TIM12) return &TIM12_Manage_Object;
    if (htim->Instance == TIM13) return &TIM13_Manage_Object;
    if (htim->Instance == TIM14) return &TIM14_Manage_Object;
    return nullptr;
}

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief 初始化 TIM 定时器
 * @param htim 定时器句柄
 * @param Callback_Function 处理回调函数
 */
void TIM_Init(TIM_HandleTypeDef* htim, TIM_Call_Back Callback_Function) {
    if (htim == nullptr) return;

    Struct_TIM_Manage_Object* obj = Get_TIM_Manage_Object(htim);
    
    if (obj != nullptr) {
        obj->TIM_Handler = htim;
        obj->Callback_Function = Callback_Function;
    }
}

/**
 * @brief HAL 库 TIM 定时器周期溢出中断回调
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    // 卫语句：判断初始化状态
    if (!init_finished || htim == nullptr) {
        return;
    }

    Struct_TIM_Manage_Object* obj = Get_TIM_Manage_Object(htim);

    // 执行回调
    if (obj != nullptr && obj->Callback_Function != nullptr) {
        obj->Callback_Function();
    }
}