/* Includes ------------------------------------------------------------------*/

#include "drv_spi.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_IIC_Manage_Object SPI1_Manage_Object = {0};
Struct_IIC_Manage_Object SPI2_Manage_Object = {0};
Struct_IIC_Manage_Object SPI3_Manage_Object = {0};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

Struct_IIC_Manage_Object* Get_SPI_Obj(SPI_HandleTypeDef* h) {
    if (h->Instance == SPI1) return &SPI1_Manage_Object;
    if (h->Instance == SPI2) return &SPI2_Manage_Object;
    if (h->Instance == SPI3) return &SPI3_Manage_Object;
    return nullptr;
}

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
uint8_t SPI_Send_Receive_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint16_t Tx_Length, uint16_t Rx_Length)
{
	if (hspi->State != HAL_SPI_STATE_READY) {
        return HAL_BUSY; 
    }

	Struct_IIC_Manage_Object* obj = Get_SPI_Obj(hspi);

	if (obj && obj->Callback_Function != nullptr) {
		obj->Now_GPIOx = GPIOx;
        obj->Now_GPIO_Pin = GPIO_Pin;

        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
        obj->Now_Tx_Length = Tx_Length;
        obj->Now_Rx_Length = Rx_Length;
        uint8_t number = HAL_SPI_TransmitReceive_DMA(hspi, obj->Tx_Buffer, obj->Rx_Buffer, Tx_Length + Rx_Length);
		__HAL_DMA_DISABLE_IT(hspi->hdmarx, DMA_IT_HT);
		
		return number;
    }
	
	return 0;
}

/**
 * @brief SPI的TIM定时器中断交互回调函数
 * 
 */
void TIM_100us_SPI_PeriodElapsedCallback()
{

}

/**
 * @brief HAL库SPI交互DMA中断
 * 
 * @param hspi SPI编号
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	Struct_IIC_Manage_Object* obj = Get_SPI_Obj(hspi);
    if (obj) {
        // 无论如何，传输结束必须立刻释放片选
        HAL_GPIO_WritePin(obj->Now_GPIOx, obj->Now_GPIO_Pin, GPIO_PIN_SET);
    }

    // 选择回调函数
    if (obj && obj->Callback_Function != nullptr) {
        obj->Callback_Function(obj->Tx_Buffer, obj->Rx_Buffer, obj->Now_Tx_Length + obj->Now_Rx_Length);
    }

}
