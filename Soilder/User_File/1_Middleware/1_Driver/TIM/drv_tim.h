#pragma once

/**
 * @brief TIM 定时器底层驱动配置
 * @note 统一管理所有定时器中断回调的映射与初始化
 */

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include "tim.h"
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief TIM 定时器回调函数指针类型
 */
typedef void (*TIM_Call_Back)();

/**
 * @brief TIM 管理对象结构体
 */
struct Struct_TIM_Manage_Object {
    TIM_HandleTypeDef* TIM_Handler;
    TIM_Call_Back Callback_Function;
};

/* Exported variables --------------------------------------------------------*/

// 全局初始化标志位
extern bool init_finished;

// 定时器硬件句柄 (按需在外部声明)
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;

// TIM 管理对象列表
extern Struct_TIM_Manage_Object TIM1_Manage_Object;
extern Struct_TIM_Manage_Object TIM2_Manage_Object;
extern Struct_TIM_Manage_Object TIM3_Manage_Object;
extern Struct_TIM_Manage_Object TIM4_Manage_Object;
extern Struct_TIM_Manage_Object TIM5_Manage_Object;
extern Struct_TIM_Manage_Object TIM6_Manage_Object;
extern Struct_TIM_Manage_Object TIM7_Manage_Object;
extern Struct_TIM_Manage_Object TIM8_Manage_Object;
extern Struct_TIM_Manage_Object TIM9_Manage_Object;
extern Struct_TIM_Manage_Object TIM10_Manage_Object;
extern Struct_TIM_Manage_Object TIM11_Manage_Object;
extern Struct_TIM_Manage_Object TIM12_Manage_Object;
extern Struct_TIM_Manage_Object TIM13_Manage_Object;
extern Struct_TIM_Manage_Object TIM14_Manage_Object;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化 TIM 定时器并绑定回调函数
 */
void TIM_Init(TIM_HandleTypeDef* htim, TIM_Call_Back Callback_Function);