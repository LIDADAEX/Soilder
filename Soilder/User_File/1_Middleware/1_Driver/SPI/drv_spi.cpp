//
// Created by 谭恩泽 on 2025/10/24.
//

#include "drv_spi.h"

Struct_SPI_Manage_Object SPI1_Manage_Object = {0};
Struct_SPI_Manage_Object SPI2_Manage_Object = {0};
Struct_SPI_Manage_Object SPI3_Manage_Object = {0};


/**
 * @brief 初始化SPI
 *
 * @param hspi SPI编号
 * @param Callback_Function 处理回调函数
 */
void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Call_Back Callback_Function)
{
    if (hspi->Instance == SPI1)
    {
        SPI1_Manage_Object.SPI_Handler = hspi;
        SPI1_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (hspi->Instance == SPI2)
    {
        SPI2_Manage_Object.SPI_Handler = hspi;
        SPI2_Manage_Object.Callback_Function = Callback_Function;
    }
    else if (hspi->Instance == SPI3)
    {
        SPI3_Manage_Object.SPI_Handler = hspi;
        SPI3_Manage_Object.Callback_Function = Callback_Function;
    }

}

/**
 * @brief 交互数据帧
 *
 * @param hspi SPI编号
 * @param GPIOx 片选GPIO引脚编组
 * @param GPIO_Pin 片选GPIO引脚号
 * @param Length 长度
 * @return uint8_t 执行状态
 */
HAL_StatusTypeDef SPI_Send_Receive_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Rx_Length)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin,GPIO_PIN_RESET);

    if (hspi->Instance == SPI1)
    {
        SPI1_Manage_Object.Now_GPIOx = GPIOx;
        SPI1_Manage_Object.Now_GPIO_Pin = GPIO_Pin;
        SPI1_Manage_Object.Now_TxRx_Length = Tx_Rx_Length;

        return (HAL_SPI_TransmitReceive(hspi, Tx_Buffer, Rx_Buffer, Tx_Rx_Length,1000));
    }
    else if (hspi->Instance == SPI2)
    {
        SPI2_Manage_Object.Now_GPIOx = GPIOx;
        SPI2_Manage_Object.Now_GPIO_Pin = GPIO_Pin;
        SPI2_Manage_Object.Now_TxRx_Length = Tx_Rx_Length;

        return (HAL_SPI_TransmitReceive(hspi, Tx_Buffer, Rx_Buffer, Tx_Rx_Length,1000));
    }
    else if (hspi->Instance == SPI3)
    {
        SPI3_Manage_Object.Now_GPIOx = GPIOx;
        SPI3_Manage_Object.Now_GPIO_Pin = GPIO_Pin;
        SPI3_Manage_Object.Now_TxRx_Length = Tx_Rx_Length;

        return (HAL_SPI_TransmitReceive(hspi, Tx_Buffer, Rx_Buffer, Tx_Rx_Length,1000));
    }
    return HAL_ERROR;
}


/**
 * @brief HAL库SPI交互DMA中断
 *
 * @param hspi SPI编号
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // 判断程序初始化完成
    if (init_finished == 0)
    {
        return;
    }

    // 选择回调函数
    if (hspi->Instance == SPI1)
    {
        if(SPI1_Manage_Object.Callback_Function != nullptr)
        {
            SPI1_Manage_Object.Callback_Function( SPI1_Manage_Object.Now_TxRx_Length);
        }
    }
    else if (hspi->Instance == SPI2)
    {
        if(SPI2_Manage_Object.Callback_Function != nullptr)
        {
            SPI2_Manage_Object.Callback_Function(SPI2_Manage_Object.Now_TxRx_Length);
        }
    }
    else if (hspi->Instance == SPI3)
    {
        if(SPI3_Manage_Object.Callback_Function != nullptr)
        {
            SPI3_Manage_Object.Callback_Function(SPI3_Manage_Object.Now_TxRx_Length);
        }
    }
}
