//
// Created by 谭恩泽 on 2025/10/24.
//

#ifndef C_BOARD_DRV_SPI_H
#define C_BOARD_DRV_SPI_H

#include "stm32f4xx_hal.h"
#include "spi.h"

// 缓冲区字节长度
#define SPI_BUFFER_SIZE 32

/* Exported types ------------------------------------------------------------*/

/**
 * @brief SPI通信接收回调函数数据类型
 *
 */
typedef void (*SPI_Call_Back)( uint16_t Length);

/**
 * @brief SPI通信处理结构体
 *
 */
struct Struct_SPI_Manage_Object
{
    SPI_HandleTypeDef *SPI_Handler;
    GPIO_TypeDef *Now_GPIOx;
    uint16_t Now_GPIO_Pin;
    uint16_t Now_TxRx_Length;
    SPI_Call_Back Callback_Function;
};

/* Exported variables ---------------------------------------------------------*/

extern bool init_finished;

extern Struct_SPI_Manage_Object SPI1_Manage_Object;
extern Struct_SPI_Manage_Object SPI2_Manage_Object;
extern Struct_SPI_Manage_Object SPI3_Manage_Object;




/* Exported function declarations ---------------------------------------------*/

void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Call_Back Callback_Function);

HAL_StatusTypeDef SPI_Send_Receive_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Rx_Length);


#endif //C_BOARD_DRV_SPI_H
