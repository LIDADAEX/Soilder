#include "dvc_dr16.h"
#include <string.h>

void Class_DR16::Init(UART_HandleTypeDef* huart) {
    if (huart->Instance == USART1) m_UART_Manage_Object = &UART1_Manage_Object;
    else if (huart->Instance == USART2) m_UART_Manage_Object = &UART2_Manage_Object;
    else if (huart->Instance == USART3) m_UART_Manage_Object = &UART3_Manage_Object;
    else if (huart->Instance == UART4) m_UART_Manage_Object = &UART4_Manage_Object;
    else if (huart->Instance == UART5) m_UART_Manage_Object = &UART5_Manage_Object;
    else if (huart->Instance == USART6) m_UART_Manage_Object = &UART6_Manage_Object;
}

void Class_DR16::UART_RxCpltCallback(uint8_t* Rx_Data, uint16_t Length) {
    if (Length == 18) {
        m_Flag++;
		Calculate_PeriodElapsedCallback();
    }
}

void Class_DR16::TIM_Alive_PeriodElapsedCallback() {
    if (m_Flag == m_Pre_Flag) {
        m_DR16_Status = DR16_Status_DISABLE;
        // 如果断连，复位 UART 接收
        if (m_UART_Manage_Object) UART_Reinit(m_UART_Manage_Object->UART_Handler);
    } else {
        m_DR16_Status = DR16_Status_ENABLE;
    }
    m_Pre_Flag = m_Flag;
}

void Class_DR16::Calculate_PeriodElapsedCallback() {
    if (m_DR16_Status == DR16_Status_DISABLE || m_UART_Manage_Object == nullptr) return;

    // 1. 拷贝原始数据
    Struct_DR16_UART_Data raw;
    memcpy(&raw, m_UART_Manage_Object->Rx_Buffer, 18);

    // 2. 摇杆与鼠标归一化
    m_Data.Right_X = (raw.Channel_0 - m_Rocker_Offset) / m_Rocker_Num;
    m_Data.Right_Y = (raw.Channel_1 - m_Rocker_Offset) / m_Rocker_Num;
    m_Data.Left_X  = (raw.Channel_2 - m_Rocker_Offset) / m_Rocker_Num;
    m_Data.Left_Y  = (raw.Channel_3 - m_Rocker_Offset) / m_Rocker_Num;
    m_Data.Side_Wheel = (raw.Channel_Side_Wheel - m_Rocker_Offset) / m_Rocker_Num;

    m_Data.Mouse_X = raw.Mouse_X / 32768.0f;
    m_Data.Mouse_Y = raw.Mouse_Y / 32768.0f;
    m_Data.Mouse_Z = raw.Mouse_Z / 32768.0f;

    // 3. 状态机判断（边沿检测）
    _Judge_Switch(&m_Data.Left_Switch, raw.Switch_Left, m_Pre_UART_Rx_Data.Switch_Left);
    _Judge_Switch(&m_Data.Right_Switch, raw.Switch_Right, m_Pre_UART_Rx_Data.Switch_Right);
    
    _Judge_Key(&m_Data.Mouse_Left_Key, raw.Mouse_Left_Key, m_Pre_UART_Rx_Data.Mouse_Left_Key);
    _Judge_Key(&m_Data.Mouse_Right_Key, raw.Mouse_Right_Key, m_Pre_UART_Rx_Data.Mouse_Right_Key);

    for (int i = 0; i < 16; i++) {
        uint8_t now = (raw.Keyboard_Key >> i) & 0x01;
        uint8_t pre = (m_Pre_UART_Rx_Data.Keyboard_Key >> i) & 0x01;
        _Judge_Key(&m_Data.Keyboard_Key[i], now, pre);
    }

    // 4. 更新前一时刻数据
    m_Pre_UART_Rx_Data = raw;
}

void Class_DR16::_Judge_Switch(Enum_DR16_Switch_Status* Target, uint8_t Now, uint8_t Pre) {
    if (Now == Pre) {
        if (Now == DR16_RAW_SWITCH_UP) *Target = DR16_Switch_Status_UP;
        else if (Now == DR16_RAW_SWITCH_DOWN) *Target = DR16_Switch_Status_DOWN;
        else *Target = DR16_Switch_Status_MIDDLE;
    } else {
        // 触发逻辑：前一时刻 -> 当前时刻
        if (Pre == DR16_RAW_SWITCH_UP && Now == DR16_RAW_SWITCH_MIDDLE) *Target = DR16_Switch_Status_TRIG_UP_MIDDLE;
        else if (Pre == DR16_RAW_SWITCH_MIDDLE && Now == DR16_RAW_SWITCH_UP) *Target = DR16_Switch_Status_TRIG_MIDDLE_UP;
        else if (Pre == DR16_RAW_SWITCH_MIDDLE && Now == DR16_RAW_SWITCH_DOWN) *Target = DR16_Switch_Status_TRIG_MIDDLE_DOWN;
        else if (Pre == DR16_RAW_SWITCH_DOWN && Now == DR16_RAW_SWITCH_MIDDLE) *Target = DR16_Switch_Status_TRIG_DOWN_MIDDLE;
    }
}

void Class_DR16::_Judge_Key(Enum_DR16_Key_Status* Target, uint8_t Now, uint8_t Pre) {
    if (Now == Pre) {
        *Target = (Now == 0) ? DR16_Key_Status_FREE : DR16_Key_Status_PRESSED;
    } else {
        *Target = (Now == 1) ? DR16_Key_Status_TRIG_FREE_PRESSED : DR16_Key_Status_TRIG_PRESSED_FREE;
    }
}