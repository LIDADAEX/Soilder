#include "dvc_referee.h"
#include <string.h>
// [CN] 静态指针，用于回调函数中访问具体的类实例
static Class_Referee* Global_Referee_Instance = nullptr;

// [CN] 驱动层所需的回调函数桥接
static void Referee_Bridge_Callback(uint8_t* buf, uint16_t len) {
    if (Global_Referee_Instance) Global_Referee_Instance->UART_RxCpltCallback(buf, len);
}

// --- 官方 CRC8 查找表 ---
static const uint8_t CRC8_TAB[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

// --- 官方 CRC16 查找表 ---
static const uint16_t CRC16_TAB[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

// --- 类实现 ---

void Class_Referee::Init(UART_HandleTypeDef *huart) {
    Global_Referee_Instance = this;
    this->huart = huart;
    UART_Init(huart, Referee_Bridge_Callback, 512);

    if (huart->Instance == USART1) {
        UART_Obj = &UART1_Manage_Object;
    } else if (huart->Instance == USART2) {
        UART_Obj = &UART2_Manage_Object;
    } else if (huart->Instance == USART3) {
        UART_Obj = &UART3_Manage_Object;
    } else if (huart->Instance == UART4) {
        UART_Obj = &UART4_Manage_Object;
    } else if (huart->Instance == UART5) {
        UART_Obj = &UART5_Manage_Object;
    } else if (huart->Instance == USART6) {
        UART_Obj = &UART6_Manage_Object;
    }
}

void Class_Referee::UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length) {
    Flag++;
    Data_Process(Rx_Data, Length);
}

void Class_Referee::TIM_Alive_PeriodElapsedCallback() {
    Pre_Flag = Flag;
}

void Class_Referee::Data_Process(uint8_t *pData, uint16_t Len) {
    uint16_t i = 0;
    while (i + 9 <= Len) {
        if (pData[i] != 0xA5) { i++; continue; }
        // 校验头 CRC8
        if (Get_CRC8(&pData[i], 4, 0xFF) != pData[i + 4]) { i++; continue; }

        uint16_t data_len = (pData[i + 2] << 8) | pData[i + 1];
        uint16_t packet_len = data_len + 9;
        if (i + packet_len > Len) break;

        // 校验整包 CRC16
        uint16_t rx_crc16 = (pData[i + packet_len - 1] << 8) | pData[i + packet_len - 2];
        if (Get_CRC16(&pData[i], packet_len - 2, 0xFFFF) != rx_crc16) {
            i++; continue;
        }

        uint16_t cmd_id = (pData[i + 6] << 8) | pData[i + 5];
        uint8_t *data_ptr = &pData[i + 7];

        switch (cmd_id) {
            case Referee_Command_ID_GAME_STATUS:   memcpy(&Game_Status,  data_ptr, sizeof(Game_Status));   break;          // 0x0001 比赛状态 3Hz
            case Referee_Command_ID_GAME_RESULT:   memcpy(&Game_Result,  data_ptr, sizeof(Game_Result));   break;          // 0x0002 比赛结果 比赛结束
            case Referee_Command_ID_GAME_ROBOT_HP: memcpy(&Robot_HP,     data_ptr, sizeof(Robot_HP));     break;          // 0x0003 机器人血量 1Hz
            case Referee_Command_ID_EVENT_SELF_DATA:    memcpy(&Event_Data,   data_ptr, sizeof(Event_Data));    break;        // 0x0101 场地事件 1Hz
            case Referee_Command_ID_EVENT_SELF_REFEREE_WARNING: memcpy(&Referee_Warning, data_ptr, sizeof(Referee_Warning)); break; // 0x0104 裁判警告 判罚发生
            case Referee_Command_ID_EVENT_SELF_DART_STATUS: memcpy(&Dart_Status, data_ptr, sizeof(Dart_Status)); break;     // 0x0105 飞镖15s倒计时 1Hz
            case Referee_Command_ID_ROBOT_STATUS:  memcpy(&Robot_Status, data_ptr, sizeof(Robot_Status));  break;          // 0x0201 机器人状态 10Hz
            case Referee_Command_ID_ROBOT_POWER_HEAT:    memcpy(&Power_Heat,   data_ptr, sizeof(Power_Heat));    break;        // 0x0202 功率热量 50Hz
            case Referee_Command_ID_ROBOT_POSITION:     memcpy(&Robot_Pos,    data_ptr, sizeof(Robot_Pos));     break;         // 0x0203 机器人位置 10Hz
            case Referee_Command_ID_ROBOT_BUFF:    memcpy(&Robot_Buff,   data_ptr, sizeof(Robot_Buff));    break;          // 0x0204 机器人增益 1Hz
            case Referee_Command_ID_ROBOT_AERIAL_STATUS: memcpy(&Aerial_Status, data_ptr, sizeof(Aerial_Status)); break;     // 0x0205 空中机器人状态 10Hz
            case Referee_Command_ID_ROBOT_DAMAGE:     memcpy(&Hurt_Data,    data_ptr, sizeof(Hurt_Data));     break;          // 0x0206 伤害情况 伤害发生
            case Referee_Command_ID_ROBOT_BOOSTER:    memcpy(&Shoot_Data,   data_ptr, sizeof(Shoot_Data));    break;          // 0x0207 子弹信息 射击发生
            case Referee_Command_ID_ROBOT_REMAINING_AMMO:   memcpy(&Ammo_Remain,  data_ptr, sizeof(Ammo_Remain));   break;      // 0x0208 子弹剩余 10Hz
            case Referee_Command_ID_ROBOT_RFID:   memcpy(&RFID_Status,  data_ptr, sizeof(RFID_Status));   break;           // 0x0209 RFID状态 1Hz
            case Referee_Command_ID_ROBOT_DART_COMMAND: memcpy(&Dart_Command, data_ptr, sizeof(Dart_Command)); break;       // 0x020a 飞镖命令 10Hz
            case Referee_Command_ID_ROBOT_SENTRY_LOCATION: memcpy(&Sentry_Location, data_ptr, sizeof(Sentry_Location)); break; // 0x020b 哨兵位置 1Hz
            case Referee_Command_ID_ROBOT_RADAR_MARK: memcpy(&Radar_Mark, data_ptr, sizeof(Radar_Mark)); break;          // 0x020c 雷达标记 1Hz
            case Referee_Command_ID_ROBOT_SENTRY_DECISION: memcpy(&Sentry_Decision, data_ptr, sizeof(Sentry_Decision)); break; // 0x020d 哨兵决策 1Hz
            case Referee_Command_ID_ROBOT_RADAR_DECISION: memcpy(&Radar_Decision, data_ptr, sizeof(Radar_Decision)); break;  // 0x020e 雷达决策 1Hz
            case Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CUSTOM_CONTROLLER: 
                memcpy(&Custom_Controller_Data, data_ptr, sizeof(Custom_Controller_Data)); break; // 0x0302 自定义控制器 30Hz
            case Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_MINIMAP:
                memcpy(&Client_Minimap, data_ptr, sizeof(Client_Minimap)); break; // 0x0303 客户端小地图 2Hz
            case Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_REMOTE_CONTROL:
                memcpy(&Client_Remote_Control, data_ptr, sizeof(Client_Remote_Control)); break; // 0x0304 图传键鼠遥控 30Hz
            default: break;
        }
        i += packet_len;
    }
}

void Class_Referee::Send_UI_Graphic_1(uint8_t layer, const char name[3], Struct_Referee_Data_Interaction_Graphic_Config *graphic) {
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 15;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_UI_GRAPHIC_1;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);
    memcpy(&Tx_Buffer[13], graphic, 15);

    Append_CRC8_CRC16(Tx_Buffer, total_len);
    UART_Send_Data(huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_UI_String(uint8_t layer, const char name[3], const char* str, uint16_t start_x, uint16_t start_y) {
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 15 + 30;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_UI_GRAPHIC_STRING;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);

    Struct_Referee_Data_Interaction_Graphic_Config config{};
    memcpy(config.Index, name, 3);
    config.Figure_Type = 7; config.Layer = layer; config.Start_X = start_x; config.Start_Y = start_y;
    config.Details_A = 20; config.Details_B = strlen(str); config.Details_C = 2; config.Operation = 1;

    memcpy(&Tx_Buffer[13], &config, 15);
    strncpy((char*)&Tx_Buffer[28], str, 30);

    Append_CRC8_CRC16(Tx_Buffer, total_len);
    UART_Send_Data(huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_UI_Graphic_2(uint8_t layer, const char name[3], Struct_Referee_Data_Interaction_Graphic_Config *graphic) {
    // [CN] 发送绘制2个图形的UI交互数据
    // cmd_id: 0x0301, sub_cmd_id: 0x0101
    // 数据段: 6字节帧头 + 30字节图形数据(2个图形)
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 30;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_UI_GRAPHIC_2;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);
    memcpy(&Tx_Buffer[13], graphic, 30);

    Append_CRC8_CRC16(Tx_Buffer, total_len);
    UART_Send_Data(huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_UI_Graphic_5(uint8_t layer, const char name[3], Struct_Referee_Data_Interaction_Graphic_Config *graphic) {
    // [CN] 发送绘制5个图形的UI交互数据
    // cmd_id: 0x0301, sub_cmd_id: 0x0102
    // 数据段: 6字节帧头 + 75字节图形数据(5个图形)
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 75;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_UI_GRAPHIC_5;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);
    memcpy(&Tx_Buffer[13], graphic, 75);

    Append_CRC8_CRC16(Tx_Buffer, total_len);
    UART_Send_Data(huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_UI_Graphic_7(uint8_t layer, const char name[3], Struct_Referee_Data_Interaction_Graphic_Config *graphic) {
    // [CN] 发送绘制7个图形的UI交互数据
    // cmd_id: 0x0301, sub_cmd_id: 0x0103
    // 数据段: 6字节帧头 + 105字节图形数据(7个图形)
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 105;
    uint16_t total_len = data_len + 9;

    if (total_len > sizeof(Tx_Buffer)) return;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_UI_GRAPHIC_7;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);
    memcpy(&Tx_Buffer[13], graphic, 105);

    Append_CRC8_CRC16(Tx_Buffer, total_len);
    UART_Send_Data(huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_UI_Layer_Delete(uint8_t layer, uint8_t operation, uint8_t delete_serial) {
    // [CN] 删除指定图层的UI交互数据
    // cmd_id: 0x0301, sub_cmd_id: 0x0100
    // 数据段: 6字节帧头 + 3字节操作数据(operation + layer + delete_serial)
    // operation: 1-删除单个, 2-删除全部
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 3;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_UI_LAYER_DELETE;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);
    Tx_Buffer[13] = operation;
    Tx_Buffer[14] = layer;
    Tx_Buffer[15] = delete_serial;

    Append_CRC8_CRC16(Tx_Buffer, total_len);
    UART_Send_Data(huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_Sentry_Decision(uint8_t confirm_respawn, uint8_t confirm_exchange_respawn, 
                                         uint16_t ammo_number, uint8_t remote_exchange_ammo_times, 
                                         uint8_t remote_exchange_hp_times, uint8_t sentry_mode,
                                         uint8_t confirm_activate_energy) {
    // [CN] 哨兵自主决策交互信息发送
    // cmd_id: 0x0301, sub_cmd_id: 0x0120
    // 数据段：6 字节帧头 + 4 字节决策数据
    // 决策数据位域: 
    //   bit 0: 确认复活
    //   bit 1: 确认兑换立即复活
    //   bit 2-12: 弹药数量 (0-2047)
    //   bit 13-16: 远程兑换弹药量请求次数
    //   bit 17-20: 远程兑换血量请求次数
    //   bit 21-22: 哨兵姿态 (1-进攻，2-防御，3-移动)
    //   bit 23: 确认激活能量机关
    //   bit 24-31: 保留位
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 4;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_SENTRY;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Referee_Data_Robots_Client_ID_Server;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);

    uint32_t decision_data = 0;
    decision_data |= (confirm_respawn & 0x01);                                    // bit 0
    decision_data |= ((confirm_exchange_respawn & 0x01) << 1);                    // bit 1
    decision_data |= ((ammo_number & 0x7FF) << 2);                                // bit 2-12
    decision_data |= ((remote_exchange_ammo_times & 0x0F) << 13);                 // bit 13-16
    decision_data |= ((remote_exchange_hp_times & 0x0F) << 17);                   // bit 17-20
    decision_data |= ((sentry_mode & 0x03) << 21);                                // bit 21-22
    decision_data |= ((confirm_activate_energy & 0x01) << 23);                    // bit 23
    // bit 24-31: Reserved (already 0)
    memcpy(&Tx_Buffer[13], &decision_data, 4);

    Append_CRC8_CRC16(Tx_Buffer, total_len);
    UART_Send_Data(huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_Radar_Decision(uint8_t request_double_damage) {
    // [CN] 雷达自主决策交互信息发送
    // cmd_id: 0x0301, sub_cmd_id: 0x0121
    // 数据段: 6字节帧头 + 1字节请求数据
    // 请求数据: bit0-1双倍伤害几率(0-3), bit2双倍伤害敌方状态
    if (UART_Obj == nullptr) return;
    uint16_t data_len = 6 + 1;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sub_cmd_id = Referee_Interaction_Command_ID_RADAR;
    memcpy(&Tx_Buffer[7], &sub_cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Referee_Data_Robots_Client_ID_Server;
    memcpy(&Tx_Buffer[9], &sender_id, 2);
    memcpy(&Tx_Buffer[11], &receiver_id, 2);
    Tx_Buffer[13] = request_double_damage;

    Append_CRC8_CRC16(Tx_Buffer, total_len);
    UART_Send_Data(huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_Custom_Controller_Data(uint8_t *data, uint8_t len) {
    // [CN] 自定义控制器交互数据发送
    // cmd_id: 0x0302
    // 数据段: 6字节帧头 + 自定义数据(最大30字节)
    // 发送频率: 最高30Hz
    if (UART_Obj == nullptr || len > 30) return;
    uint16_t data_len = 6 + len;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CUSTOM_CONTROLLER;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Robot_Status.Robot_ID + 0x0100;
    memcpy(&Tx_Buffer[7], &sender_id, 2);
    memcpy(&Tx_Buffer[9], &receiver_id, 2);
    memcpy(&Tx_Buffer[11], data, len);

    Append_CRC8_CRC16(Tx_Buffer, total_len);
    UART_Send_Data(huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_Client_Receive_Radar(uint16_t *robot_positions, uint8_t len) {
    // [CN] 雷达发送机器人位置信息给客户端
    // cmd_id: 0x0305
    // 数据段：6 字节帧头 + 24 字节位置数据
    // 位置数据：每个机器人 4 字节 (x 2 字节 + y 2 字节)，单位 cm
    // 发送频率：最高 10Hz
    // 参数：robot_positions - uint16_t 数组，包含 12 个位置坐标 (6 个机器人的 x,y)
    //       len - 数据长度 (固定为 24 字节)
    if (UART_Obj == nullptr || len != 24) return;
    uint16_t data_len = 6 + 24;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_RADAR;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    uint16_t sender_id = Robot_Status.Robot_ID;
    uint16_t receiver_id = Referee_Data_Robots_Client_ID_All;  // 发送给所有客户端
    memcpy(&Tx_Buffer[7], &sender_id, 2);
    memcpy(&Tx_Buffer[9], &receiver_id, 2);
    
    // 复制 24 字节位置数据
    memcpy(&Tx_Buffer[11], robot_positions, 24);

    Append_CRC8_CRC16(Tx_Buffer, total_len);
    UART_Send_Data(huart, Tx_Buffer, total_len);
}

void Class_Referee::Send_Robot_Minimap(uint16_t sender_id, uint16_t receiver_id, uint8_t *data, uint8_t len) {
    // [CN] 机器人发送小地图数据给客户端
    // cmd_id: 0x0308
    // 数据段: 6字节帧头 + 4字节标识 + 自定义数据(最大30字节)
    // 发送频率: 最高10Hz
    if (UART_Obj == nullptr || len > 30) return;
    uint16_t data_len = 6 + 4 + len;
    uint16_t total_len = data_len + 9;

    memset(Tx_Buffer, 0, total_len);
    Tx_Buffer[0] = 0xA5;
    Tx_Buffer[1] = data_len & 0xFF;
    Tx_Buffer[2] = (data_len >> 8) & 0xFF;
    Tx_Buffer[3] = Sequence++;

    uint16_t cmd_id = Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_ROBOT_MINIMAP;
    memcpy(&Tx_Buffer[5], &cmd_id, 2);
    memcpy(&Tx_Buffer[7], &sender_id, 2);
    memcpy(&Tx_Buffer[9], &receiver_id, 2);
    memcpy(&Tx_Buffer[11], data, len);

    Append_CRC8_CRC16(Tx_Buffer, total_len);
    UART_Send_Data(huart, Tx_Buffer, total_len);
}

// --- CRC 核心计算函数 (禁止修改多项式) ---

uint8_t Class_Referee::Get_CRC8(uint8_t *p, uint32_t len, uint8_t init) {
    uint8_t crc = init;
    while (len--) crc = CRC8_TAB[crc ^ (*p++)];
    return crc;
}

uint16_t Class_Referee::Get_CRC16(uint8_t *p, uint32_t len, uint16_t init) {
    uint16_t crc = init;
    while (len--) crc = (crc >> 8) ^ CRC16_TAB[(crc ^ (*p++)) & 0xFF];
    return crc;
}

void Class_Referee::Append_CRC8_CRC16(uint8_t *pData, uint16_t TotalLen) {
    pData[4] = Get_CRC8(pData, 4, 0xFF);
    uint16_t crc16 = Get_CRC16(pData, TotalLen - 2, 0xFFFF);
    pData[TotalLen - 2] = crc16 & 0xFF;
    pData[TotalLen - 1] = (crc16 >> 8) & 0xFF;
}


