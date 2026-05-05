#pragma once

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "i2c.h"
#include <string.h>

/* Exported macros -----------------------------------------------------------*/

// 缓冲区字节长度
#define I2C_BUFFER_SIZE 256

/* Exported types ------------------------------------------------------------*/

/**
 * @brief SPI通信接收回调函数数据类型
 *
 */
typedef void (*I2C_Call_Back)(uint16_t DevAddress, uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length);
typedef void (*I2C_Error_Call_Back)(uint16_t DevAddress);

/**
 * @brief CAN通信处理结构体
 *
 */
struct Struct_I2C_Manage_Object
{
    I2C_HandleTypeDef *I2C_Handler;
    I2C_Call_Back Callback_Function;
    I2C_Error_Call_Back Error_Callback_Fuction;
    uint16_t Now_DevAddress;
    uint8_t Tx_Buffer[I2C_BUFFER_SIZE];
    uint16_t Tx_Length;
    uint8_t Rx_Buffer[I2C_BUFFER_SIZE];
    uint16_t Rx_Length;
};

/* Exported variables ---------------------------------------------------------*/

extern bool init_finished;

extern Struct_I2C_Manage_Object I2C1_Manage_Object;
extern Struct_I2C_Manage_Object I2C2_Manage_Object;
extern Struct_I2C_Manage_Object I2C3_Manage_Object;

extern uint8_t SPI5_PF6_Tx_Data[];

/* Exported function declarations ---------------------------------------------*/

Struct_I2C_Manage_Object* Get_I2C_Obj(I2C_HandleTypeDef* h);

void I2C_Init(I2C_HandleTypeDef *hi2c, I2C_Call_Back Callback_Function, I2C_Error_Call_Back Error_Callback_Fuction);

uint8_t I2C_Send_Data(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);

uint8_t I2C_Reccive_Data(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t Size);

void TIM_1ms_I2C_PeriodElapsedCallback();

