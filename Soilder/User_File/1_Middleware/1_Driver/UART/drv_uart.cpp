/**
 * @brief UART 通信驱动
 * @note 支持 USART1-6 的 DMA 闲时中断接收与错误重连逻辑
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"

/* Private variables ---------------------------------------------------------*/

Struct_UART_Manage_Object UART1_Manage_Object = {0};
Struct_UART_Manage_Object UART2_Manage_Object = {0};
Struct_UART_Manage_Object UART3_Manage_Object = {0};
Struct_UART_Manage_Object UART4_Manage_Object = {0};
Struct_UART_Manage_Object UART5_Manage_Object = {0};
Struct_UART_Manage_Object UART6_Manage_Object = {0};

/* Private Functions ---------------------------------------------------------*/

/**
 * @brief 根据 UART 实例获取对应的管理对象
 */
static Struct_UART_Manage_Object* Get_UART_Manage_Object(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART1) return &UART1_Manage_Object;
    if (huart->Instance == USART2) return &UART2_Manage_Object;
    if (huart->Instance == USART3) return &UART3_Manage_Object;
    if (huart->Instance == UART4)  return &UART4_Manage_Object;
    if (huart->Instance == UART5)  return &UART5_Manage_Object;
    if (huart->Instance == USART6) return &UART6_Manage_Object;
    return nullptr;
}

/**
 * @brief 内部函数：启动 UART 的 DMA 闲时接收并禁用半完成中断
 */
static void Internal_UART_Start_Receive(Struct_UART_Manage_Object* obj) {
    if (obj == nullptr || obj->UART_Handler == nullptr) return;

    HAL_UARTEx_ReceiveToIdle_DMA(obj->UART_Handler, obj->Rx_Buffer, obj->Rx_Buffer_Length);
    // 禁用 DMA 半完成中断，防止数据接收到一半触发回调
    __HAL_DMA_DISABLE_IT(obj->UART_Handler->hdmarx, DMA_IT_HT);
}

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief 初始化 UART
 * @param huart UART 编号
 * @param Callback_Function 处理回调函数
 * @param Rx_Buffer_Length 接收缓冲区长度
 */
void UART_Init(UART_HandleTypeDef* huart, UART_Call_Back Callback_Function, uint16_t Rx_Buffer_Length) {
    if (huart == nullptr) return;

    Struct_UART_Manage_Object* obj = Get_UART_Manage_Object(huart);
    if (obj != nullptr) {
        obj->UART_Handler = huart;
        obj->Callback_Function = Callback_Function;
        obj->Rx_Buffer_Length = Rx_Buffer_Length;

        Internal_UART_Start_Receive(obj);
    }
}

/**
 * @brief 掉线重新初始化 UART
 */
void UART_Reinit(UART_HandleTypeDef* huart) {
    Struct_UART_Manage_Object* obj = Get_UART_Manage_Object(huart);
    if (obj != nullptr) {
        Internal_UART_Start_Receive(obj);
    }
}

/**
 * @brief 发送数据帧 (DMA 方式)
 */
uint8_t UART_Send_Data(UART_HandleTypeDef* huart, uint8_t* Data, uint16_t Length) {
    if (huart == nullptr) return HAL_ERROR;
    return HAL_UART_Transmit_DMA(huart, Data, Length);
}

/**
 * @brief 1ms 定时器回调 (预留)
 */
void TIM_1ms_UART_Period_Elapsed_Callback() {
    // 可以在此处理串口超时或心跳逻辑
}

/**
 * @brief HAL 库 UART 接收回调 (RxEvent 对应 Idle 中断)
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (!init_finished || huart == nullptr) return;

    Struct_UART_Manage_Object* obj = Get_UART_Manage_Object(huart);
    if (obj == nullptr) return;

    // 停止 DMA 接收以处理数据
    HAL_UART_DMAStop(huart);

    // 执行业务回调
    if (obj->Callback_Function != nullptr) {
        obj->Callback_Function(obj->Rx_Buffer, Size);
    }

    // 重新开启接收
    Internal_UART_Start_Receive(obj);
}

/**
 * @brief HAL 库 UART 错误中断
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    Struct_UART_Manage_Object* obj = Get_UART_Manage_Object(huart);
    if (obj != nullptr) {
        Internal_UART_Start_Receive(obj);
    }
}