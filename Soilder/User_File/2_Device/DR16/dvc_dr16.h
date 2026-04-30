#pragma once

/**
 * @brief 遥控器 DR16 驱动
 * @note 摇杆/鼠标归一化处理，支持按键边沿检测
 */

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/1_Driver/UART/drv_uart.h"

/* Exported macros -----------------------------------------------------------*/

// 原始拨动开关值
#define DR16_RAW_SWITCH_UP     (1)
#define DR16_RAW_SWITCH_DOWN   (2)
#define DR16_RAW_SWITCH_MIDDLE (3)

/* Exported types ------------------------------------------------------------*/

enum Enum_DR16_Status {
    DR16_Status_DISABLE = 0,
    DR16_Status_ENABLE,
};

enum Enum_DR16_Switch_Status {
    DR16_Switch_Status_UP = 0,
    DR16_Switch_Status_TRIG_UP_MIDDLE,
    DR16_Switch_Status_TRIG_MIDDLE_UP,
    DR16_Switch_Status_MIDDLE,
    DR16_Switch_Status_TRIG_MIDDLE_DOWN,
    DR16_Switch_Status_TRIG_DOWN_MIDDLE,
    DR16_Switch_Status_DOWN,
};

enum Enum_DR16_Key_Status {
    DR16_Key_Status_FREE = 0,
    DR16_Key_Status_TRIG_FREE_PRESSED,
    DR16_Key_Status_TRIG_PRESSED_FREE,
    DR16_Key_Status_PRESSED,
};

/**
 * @brief DR16 原始协议数据结构
 */
struct Struct_DR16_UART_Data {
    uint64_t Channel_0 : 11;
    uint64_t Channel_1 : 11;
    uint64_t Channel_2 : 11;
    uint64_t Channel_3 : 11;
    uint64_t Switch_Right : 2;
    uint64_t Switch_Left : 2;
    int16_t  Mouse_X;
    int16_t  Mouse_Y;
    int16_t  Mouse_Z;
    uint8_t  Mouse_Left_Key;
    uint8_t  Mouse_Right_Key;
    uint16_t Keyboard_Key;
    uint16_t Channel_Side_Wheel : 11;
    uint16_t Reserved : 5;
} __attribute__((packed));

/**
 * @brief 处理后的 DR16 数据
 */
struct Struct_DR16_Data {
    float Right_X, Right_Y;
    float Left_X, Left_Y;
    Enum_DR16_Switch_Status Left_Switch;
    Enum_DR16_Switch_Status Right_Switch;
    float Mouse_X, Mouse_Y, Mouse_Z;
    Enum_DR16_Key_Status Mouse_Left_Key;
    Enum_DR16_Key_Status Mouse_Right_Key;
    Enum_DR16_Key_Status Keyboard_Key[16];
    float Side_Wheel; // 原 Yaw 变量，通常为侧边拨轮
};

/* Class Definition ----------------------------------------------------------*/

class Class_DR16 {
public:
    void Init(UART_HandleTypeDef* huart);
    
    // 周期回调
    void UART_RxCpltCallback(uint8_t* Rx_Data, uint16_t Length);
    void TIM_Alive_PeriodElapsedCallback(); // 存活检测
    void Calculate_PeriodElapsedCallback(); // 逻辑计算

    // Getters
    Enum_DR16_Status Get_Status() const { return m_DR16_Status; }
    const Struct_DR16_Data& Get_Data() const { return m_Data; }
    
    // 常用按键快捷访问
    Enum_DR16_Key_Status Get_Key(int Key_Index) const { return m_Data.Keyboard_Key[Key_Index]; }

private:
    // 硬件绑定
    Struct_UART_Manage_Object* m_UART_Manage_Object = nullptr;

    // 常量
    const float m_Rocker_Offset = 1024.0f;
    const float m_Rocker_Num = 660.0f;

    // 内部状态
    Enum_DR16_Status m_DR16_Status = DR16_Status_DISABLE;
    Struct_DR16_UART_Data m_Pre_UART_Rx_Data;
    Struct_DR16_Data m_Data;
    
    uint32_t m_Flag = 0;
    uint32_t m_Pre_Flag = 0;

    // 内部处理函数
    void Data_Process();
    void _Judge_Switch(Enum_DR16_Switch_Status* Target, uint8_t Now, uint8_t Pre);
    void _Judge_Key(Enum_DR16_Key_Status* Target, uint8_t Now, uint8_t Pre);
};