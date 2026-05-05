/* Includes ------------------------------------------------------------------*/

#include "drv_iic.h"
#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_I2C_Manage_Object I2C1_Manage_Object = {0};
Struct_I2C_Manage_Object I2C2_Manage_Object = {0};
Struct_I2C_Manage_Object I2C3_Manage_Object = {0};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

Struct_I2C_Manage_Object* Get_I2C_Obj(I2C_HandleTypeDef* h) {
    if (h->Instance == I2C1) return &I2C1_Manage_Object;
    if (h->Instance == I2C2) return &I2C2_Manage_Object;
    if (h->Instance == I2C3) return &I2C3_Manage_Object;
    return nullptr;
}

/**
 * @brief 初始化I2C
 *
 * @param hi2c I2C编号
 * @param Callback_Function 处理回调函数
 */
void I2C_Init(I2C_HandleTypeDef *hi2c, I2C_Call_Back Callback_Function, I2C_Error_Call_Back Error_Callback_Fuction)
{
    Struct_I2C_Manage_Object* obj = Get_I2C_Obj(hi2c);
    if (obj == nullptr) return; 
    obj->Callback_Function = Callback_Function;
    obj->Error_Callback_Fuction = Error_Callback_Fuction;
}

/**
 * @brief 交互数据帧
 * 
 */
uint8_t I2C_Send_Data(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
	if (hi2c->State != HAL_I2C_STATE_READY) {
        return HAL_BUSY; 
    }

    Struct_I2C_Manage_Object* obj = Get_I2C_Obj(hi2c);
    memcpy(obj->Tx_Buffer, pData, Size);
    obj->Tx_Length = Size;

    obj->Now_DevAddress = DevAddress;
    return HAL_I2C_Master_Transmit_DMA(hi2c, DevAddress, obj->Tx_Buffer, obj->Tx_Length);
}

uint8_t I2C_Reccive_Data(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t Size){
    if (hi2c->State != HAL_I2C_STATE_READY) {
        return HAL_BUSY; 
    }

    Struct_I2C_Manage_Object* obj = Get_I2C_Obj(hi2c);
    obj->Rx_Length = Size;

    obj->Now_DevAddress = DevAddress;
    return HAL_I2C_Master_Receive_DMA(hi2c, DevAddress, obj->Rx_Buffer, obj->Rx_Length);
}

/**
 * @brief I2C的TIM定时器中断交互回调函数
 * 
 */
void TIM_1ms_I2C_PeriodElapsedCallback()
{

}

/**
 * @brief HAL库I2C交互DMA中断
 * 
 * @param hi2c I2C编号
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	Struct_I2C_Manage_Object* obj = Get_I2C_Obj(hi2c);

    // 选择回调函数
    if (obj && obj->Callback_Function != nullptr) {
        obj->Callback_Function(obj->Now_DevAddress, obj->Tx_Buffer, obj->Rx_Buffer, obj->Tx_Length, obj->Rx_Length);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    Struct_I2C_Manage_Object* obj = Get_I2C_Obj(hi2c);
    if (obj && obj->Callback_Function != nullptr) {
        // 发送完成时，Rx_Length 设为 0
        obj->Callback_Function(obj->Now_DevAddress, obj->Tx_Buffer, obj->Rx_Buffer, obj->Tx_Length, 0);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
    Struct_I2C_Manage_Object* obj = Get_I2C_Obj(hi2c);

    if (obj && obj->Callback_Function != nullptr) {
    obj->Error_Callback_Fuction(obj->Now_DevAddress);
}
}
