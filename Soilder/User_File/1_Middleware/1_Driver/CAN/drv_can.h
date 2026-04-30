#pragma once

/**
 * @brief CAN 通信底层驱动配置
 * @note 包含滤波器配置、基础收发接口及各电机控制数据缓冲区
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "can.h"
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN 接收信息缓冲结构体
 */
struct Struct_CAN_Rx_Buffer {
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
};

/**
 * @brief CAN 通信接收回调函数指针类型
 */
typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer*);

/**
 * @brief CAN 管理对象结构体
 */
struct Struct_CAN_Manage_Object {
    CAN_HandleTypeDef* CAN_Handler;
    Struct_CAN_Rx_Buffer Rx_Buffer;
    CAN_Call_Back Callback_Function;
};

/* Exported variables ---------------------------------------------------------*/

// 全局初始化标志位
extern bool init_finished;

// CAN 硬件句柄
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// CAN 管理对象
extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;

/**
 * @brief CAN1 发送缓冲区
 */
extern uint8_t CAN1_0x1fe_Tx_Data[8];
extern uint8_t CAN1_0x1ff_Tx_Data[8];
extern uint8_t CAN1_0x200_Tx_Data[8];
extern uint8_t CAN1_0x2fe_Tx_Data[8];
extern uint8_t CAN1_0x2ff_Tx_Data[8];
extern uint8_t CAN1_0x3fe_Tx_Data[8];
extern uint8_t CAN1_0x4fe_Tx_Data[8];

/**
 * @brief CAN2 发送缓冲区
 */
extern uint8_t CAN2_0x1fe_Tx_Data[8];
extern uint8_t CAN2_0x1ff_Tx_Data[8];
extern uint8_t CAN2_0x200_Tx_Data[8];
extern uint8_t CAN2_0x2fe_Tx_Data[8];
extern uint8_t CAN2_0x2ff_Tx_Data[8];
extern uint8_t CAN2_0x3fe_Tx_Data[8];
extern uint8_t CAN2_0x4fe_Tx_Data[8];

/**
 * @brief 超级电容发送缓冲区
 */
extern uint8_t CAN_Supercap_Tx_Data[8];

/* Exported function declarations ---------------------------------------------*/

/**
 * @brief 初始化 CAN 总线及回调配置
 */
void CAN_Init(CAN_HandleTypeDef* hcan, CAN_Call_Back Callback_Function);

/**
 * @brief 发送 CAN 数据帧(标准)
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef* hcan, uint16_t ID, uint8_t* Data, uint16_t Length);

/**
 * @brief 发送 CAN 数据帧(扩展)
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef* hcan, uint32_t ID, uint8_t* Data, uint16_t Length);

/**
 * @brief 配置 CAN 过滤器 (展开参数版)
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef* hcan, uint8_t Bank, uint32_t FIFO, uint32_t IdType, uint32_t ID, uint32_t Mask_ID);

/**
 * @brief 1ms 定时任务中调用的发送逻辑
 */
void TIM_1ms_CAN_PeriodElapsedCallback();