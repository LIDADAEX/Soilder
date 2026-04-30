/**
 * @brief CAN 通信驱动模块
 * @note 适配了电机控制、超级电容及达妙电机，支持标准/扩展帧自动识别
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_can.h"

/* Private variables ---------------------------------------------------------*/

Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

// CAN 通信发送缓冲区 (电机共享区)
uint8_t CAN1_0x1fe_Tx_Data[8], CAN1_0x1ff_Tx_Data[8], CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x2fe_Tx_Data[8], CAN1_0x2ff_Tx_Data[8], CAN1_0x3fe_Tx_Data[8], CAN1_0x4fe_Tx_Data[8];

uint8_t CAN2_0x1fe_Tx_Data[8], CAN2_0x1ff_Tx_Data[8], CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x2fe_Tx_Data[8], CAN2_0x2ff_Tx_Data[8], CAN2_0x3fe_Tx_Data[8], CAN2_0x4fe_Tx_Data[8];

uint8_t CAN_Supercap_Tx_Data[8];

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief 1ms 定时器回调中调用的 CAN 发送逻辑
 */
void TIM_1ms_CAN_PeriodElapsedCallback() {
    CAN_Send_Data(&hcan1, (uint16_t)0x200, CAN1_0x200_Tx_Data, 8);
}


/**
 * @brief 配置 CAN 过滤器
 * @param hcan CAN 句柄
 * @param Bank 过滤器组编号 (0-27)
 * @param FIFO 接收 FIFO (CAN_FILTER_FIFO0 或 CAN_FILTER_FIFO1)
 * @param IdType ID 类型 (CAN_ID_STD 或 CAN_ID_EXT)
 * @param ID 目标 ID
 * @param Mask_ID 屏蔽位
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef* hcan, uint8_t Bank, uint32_t FIFO, uint32_t IdType, uint32_t ID, uint32_t Mask_ID) {
    if (hcan == NULL) return;

    CAN_FilterTypeDef config;

    // 清零初始化结构体
    memset(&config, 0, sizeof(config));

    if (IdType == CAN_ID_STD) {
        // 标准帧配置 (左对齐处理)
        config.FilterIdHigh         = ID << 5;
        config.FilterIdLow          = 0x0000;
        config.FilterMaskIdHigh     = Mask_ID << 5;
        config.FilterMaskIdLow      = 0x0000;
    } else {
        // 扩展帧配置
        config.FilterIdHigh         = (ID << 3) >> 16;
        config.FilterIdLow          = (ID << 3) | CAN_ID_EXT;
        config.FilterMaskIdHigh     = (Mask_ID << 3) >> 16;
        config.FilterMaskIdLow      = (Mask_ID << 3) | CAN_ID_EXT;
    }

    config.FilterBank           = Bank;
    config.FilterFIFOAssignment = FIFO;
    config.FilterActivation     = ENABLE;
    config.FilterMode           = CAN_FILTERMODE_IDMASK;
    config.FilterScale          = CAN_FILTERSCALE_32BIT;
    config.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &config);
}

/**
 * @brief 初始化 CAN 总线并使能中断
 */
void CAN_Init(CAN_HandleTypeDef* hcan, CAN_Call_Back Callback_Function) {
    if (hcan == NULL) return;

    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

    Struct_CAN_Manage_Object* obj = (hcan->Instance == CAN1) ? &CAN1_Manage_Object : &CAN2_Manage_Object;
    
    obj->CAN_Handler = hcan;
    obj->Callback_Function = Callback_Function;

    // 根据 CAN1 或 CAN2 分配不同的过滤器 Bank
    uint8_t start_bank = (hcan->Instance == CAN1) ? 0 : 14;
    
    CAN_Filter_Mask_Config(hcan, start_bank,     CAN_FILTER_FIFO0, CAN_ID_STD, 0, 0);
    CAN_Filter_Mask_Config(hcan, start_bank + 1, CAN_FILTER_FIFO1, CAN_ID_STD, 0, 0);
}

/**
 * @brief 发送标准帧数据
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef* hcan, uint16_t ID, uint8_t* Data, uint16_t Length) {
    if (hcan == NULL) return HAL_ERROR;

    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = Length;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox);
}

/**
 * @brief 发送扩展帧数据
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef* hcan, uint32_t ID, uint8_t* Data, uint16_t Length){
    if (hcan == NULL) return HAL_ERROR;

    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    tx_header.StdId = 0;
    tx_header.ExtId = ID;             // 填入 29 位 ID
    tx_header.IDE   = CAN_ID_EXT;     // 指定为扩展帧
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = Length;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox);
}

/**
 * @brief 内部私有函数：处理 CAN 接收中断逻辑
 */
static void Internal_CAN_Rx_Callback(CAN_HandleTypeDef* hcan, uint32_t Fifo) {
    Struct_CAN_Manage_Object* obj = (hcan->Instance == CAN1) ? &CAN1_Manage_Object : &CAN2_Manage_Object;

    // 获取消息
    if (HAL_CAN_GetRxMessage(hcan, Fifo, &obj->Rx_Buffer.Header, obj->Rx_Buffer.Data) != HAL_OK) {
        return;
    }

    // 执行回调
    if (init_finished && obj->Callback_Function != nullptr) {
        obj->Callback_Function(&obj->Rx_Buffer);
    }
}

/**
 * @brief HAL 库 CAN 接收中断回调 (FIFO0)
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    Internal_CAN_Rx_Callback(hcan, CAN_RX_FIFO0);
}

/**
 * @brief HAL 库 CAN 接收中断回调 (FIFO1)
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    Internal_CAN_Rx_Callback(hcan, CAN_RX_FIFO1);
}