//
// Created by 谭恩泽 on 2025/11/1.
// 移植到 2026 协议 - 保留详细位域注释，移除 CRC16

#ifndef DVC_REFEREE_H
#define DVC_REFEREE_H

#include "1_Middleware/1_Driver/UART/drv_uart.h"
#include "string.h"

/**
 * Command_ID分全局, 己方, 指定机器人
 * 0x00xx, 全局是Game
 * 0x01xx, 己方是Event_Self
 * 0x02xx, 指定机器人是Robot_x, 没有x则是任意己方机器人
 * 0x03xx的指令帧直接具体指明收发方
 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 裁判系统状态
 *
 */
enum Enum_Referee_Status {
  Referee_Status_DISABLE = 0,
  Referee_Status_ENABLE,
};

/**
 * @brief 各种标签, 场地, 相关设施激活与存活状态
 *
 */
enum Enum_Referee_Data_Status : uint8_t {
  Referee_Data_Status_DISABLE = 0,
  Referee_Data_Status_ENABLE,
};

/**
 * @brief 裁判系统命令码类型
 *
 */
enum Enum_Referee_Command_ID : uint16_t {
  Referee_Command_ID_GAME_STATUS = 0x0001,
  Referee_Command_ID_GAME_RESULT,
  Referee_Command_ID_GAME_ROBOT_HP,
  Referee_Command_ID_EVENT_SELF_DATA = 0x0101,
  Referee_Command_ID_EVENT_SELF_REFEREE_WARNING = 0x0104,
  Referee_Command_ID_EVENT_SELF_DART_STATUS,
  Referee_Command_ID_ROBOT_STATUS = 0x0201,
  Referee_Command_ID_ROBOT_POWER_HEAT,
  Referee_Command_ID_ROBOT_POSITION,
  Referee_Command_ID_ROBOT_BUFF,
  Referee_Command_ID_ROBOT_AERIAL_STATUS,
  Referee_Command_ID_ROBOT_DAMAGE,
  Referee_Command_ID_ROBOT_BOOSTER,
  Referee_Command_ID_ROBOT_REMAINING_AMMO,
  Referee_Command_ID_ROBOT_RFID,
  Referee_Command_ID_ROBOT_DART_COMMAND,
  Referee_Command_ID_ROBOT_SENTRY_LOCATION,
  Referee_Command_ID_ROBOT_RADAR_MARK,
  Referee_Command_ID_ROBOT_SENTRY_DECISION,
  Referee_Command_ID_ROBOT_RADAR_DECISION,
  Referee_Command_ID_INTERACTION = 0x0301,
  Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CUSTOM_CONTROLLER,
  Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_MINIMAP,
  Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_REMOTE_CONTROL,
  Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_RADAR,
  Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_CUSTOM_CONTROLLER,
  Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_SENTRY_SEMIAUTOMATIC_MINIMAP,
  Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_ROBOT_MINIMAP,
};

/**
 * @brief 裁判系统 interaction 0x0301子命令码类型
 *
 */
enum Enum_Referee_Interaction_Command_ID : uint16_t {
  Referee_Interaction_Command_ID_UI_LAYER_DELETE = 0x0100,
  Referee_Interaction_Command_ID_UI_GRAPHIC_1,
  Referee_Interaction_Command_ID_UI_GRAPHIC_2,
  Referee_Interaction_Command_ID_UI_GRAPHIC_5,
  Referee_Interaction_Command_ID_UI_GRAPHIC_7,
  Referee_Interaction_Command_ID_UI_GRAPHIC_STRING = 0x0110,
  Referee_Interaction_Command_ID_SENTRY = 0x0120,
  Referee_Interaction_Command_ID_RADAR = 0x0121,
};

/**
 * @brief 通用单方机器人ID
 *
 */
enum Enum_Referee_Data_Robot_ID : uint8_t {
  Referee_Data_Robot_ID_NULL = 0,
  Referee_Data_Robot_ID_HERO_1,
  Referee_Data_Robot_ID_ENGINEER_2,
  Referee_Data_Robot_ID_INFANTRY_3,
  Referee_Data_Robot_ID_INFANTRY_4,
  Referee_Data_Robot_ID_INFANTRY_5,
  Referee_Data_Robot_ID_AERIAL_6,
  Referee_Data_Robot_ID_SENTRY_7,
  Referee_Data_Robot_ID_DART_8,
  Referee_Data_Robot_ID_RADAR_9,
  Referee_Data_Robot_ID_BASE_10,
  Referee_Data_Robot_ID_OUTPOST_11,
};

/**
 * @brief 通用双方机器人ID
 *
 */
enum Enum_Referee_Data_Robots_ID : uint8_t {
  Referee_Data_Robots_ID_NO = 0,
  Referee_Data_Robots_ID_RED_HERO_1,
  Referee_Data_Robots_ID_RED_ENGINEER_2,
  Referee_Data_Robots_ID_RED_INFANTRY_3,
  Referee_Data_Robots_ID_RED_INFANTRY_4,
  Referee_Data_Robots_ID_RED_INFANTRY_5,
  Referee_Data_Robots_ID_RED_AERIAL_6,
  Referee_Data_Robots_ID_RED_SENTRY_7,
  Referee_Data_Robots_ID_RED_DART_8,
  Referee_Data_Robots_ID_RED_RADAR_9,
  Referee_Data_Robots_ID_RED_BASE_10,
  Referee_Data_Robots_ID_RED_OUTPOST_11,
  Referee_Data_Robots_ID_BLUE_HERO_1 = 101,
  Referee_Data_Robots_ID_BLUE_ENGINEER_2,
  Referee_Data_Robots_ID_BLUE_INFANTRY_3,
  Referee_Data_Robots_ID_BLUE_INFANTRY_4,
  Referee_Data_Robots_ID_BLUE_INFANTRY_5,
  Referee_Data_Robots_ID_BLUE_AERIAL_6,
  Referee_Data_Robots_ID_BLUE_SENTRY_7,
  Referee_Data_Robots_ID_BLUE_DART_8,
  Referee_Data_Robots_ID_BLUE_RADAR_9,
  Referee_Data_Robots_ID_BLUE_BASE_10,
  Referee_Data_Robots_ID_BLUE_OUTPOST_11,
};

/**
 * @brief 通用双方机器人客户端ID
 *
 */
enum Enum_Referee_Data_Robots_Client_ID : uint16_t {
  Referee_Data_Robots_Client_ID_NO = 0,
  Referee_Data_Robots_Client_ID_RED_HERO_1 = 0x0101,
  Referee_Data_Robots_Client_ID_RED_ENGINEER_2,
  Referee_Data_Robots_Client_ID_RED_INFANTRY_3,
  Referee_Data_Robots_Client_ID_RED_INFANTRY_4,
  Referee_Data_Robots_Client_ID_RED_INFANTRY_5,
  Referee_Data_Robots_Client_ID_RED_AERIAL_6,
  Referee_Data_Robots_Client_ID_BLUE_HERO_1 = 0x0165,
  Referee_Data_Robots_Client_ID_BLUE_ENGINEER_2,
  Referee_Data_Robots_Client_ID_BLUE_INFANTRY_3,
  Referee_Data_Robots_Client_ID_BLUE_INFANTRY_4,
  Referee_Data_Robots_Client_ID_BLUE_INFANTRY_5,
  Referee_Data_Robots_Client_ID_BLUE_AERIAL_6,
  Referee_Data_Robots_Client_ID_Server = 0x8080,
  Referee_Data_Robots_Client_ID_All = 0xFFFF,  // 发送给所有客户端
};

/**
 * @brief 比赛类型
 *
 */
enum Enum_Referee_Game_Status_Type {
  Referee_Game_Status_Type_RMUC = 1,
  Referee_Game_Status_Type_SINGLE,
  Referee_Game_Status_Type_ICRA,
  Referee_Game_Status_Type_RMUL_3V3,
  Referee_Game_Status_Type_RMUL_1V1,
};

/**
 * @brief 比赛阶段
 *
 */
enum Enum_Referee_Game_Status_Stage {
  Referee_Game_Status_Stage_NOT_STARTED = 0,
  Referee_Game_Status_Stage_READY,
  Referee_Game_Status_Stage_15s_SELF_TESTING,
  Referee_Game_Status_Stage_5S_COUNTDOWN,
  Referee_Game_Status_Stage_BATTLE,
  Referee_Game_Status_Stage_SETTLEMENT,
};

/**
 * @brief 比赛结果
 *
 */
enum Enum_Referee_Game_Result : uint8_t {
  Referee_Game_Result_DRAW = 0,
  Referee_Game_Result_RED_WIN,
  Referee_Game_Result_BLUE_WIN,
};

/**
 * @brief 飞镖命中目标
 *
 */
enum Enum_Referee_Dart_Hit_Target : uint8_t {
  Referee_Dart_Hit_Target_NULL = 0,
  Referee_Dart_Hit_Target_OUTPOST,
  Referee_Dart_Hit_Target_BASE_STATIC,
  Referee_Dart_Hit_Target_BASE_RANDOM,
};

/**
 * @brief 补给站状态
 *
 */
enum Enum_Referee_Data_Event_Supply_Status : uint8_t {
  Referee_Data_Event_Supply_Status_CLOSED = 0,
  Referee_Data_Event_Supply_Status_READY,
  Referee_Data_Event_Supply_Status_DROPPING,
};

/**
 * @brief 补给站提供子弹数量
 *
 */
enum Enum_Referee_Data_Event_Supply_Ammo_Number : uint8_t {
  Referee_Data_Event_Supply_Ammo_Number_50 = 50,
  Referee_Data_Event_Supply_Ammo_Number_100 = 100,
  Referee_Data_Event_Supply_Ammo_Number_150 = 150,
  Referee_Data_Event_Supply_Ammo_Number_200 = 200,
};

/**
 * @brief 裁判警告等级
 *
 */
enum Enum_Referee_Data_Event_Referee_Warning_Level : uint8_t {
  Referee_Data_Referee_Warning_Level_BOTH_YELLOW = 1,
  Referee_Data_Referee_Warning_Level_YELLOW,
  Referee_Data_Referee_Warning_Level_RED,
  Referee_Data_Referee_Warning_Level_FAIL,
};

/**
 * @brief 空中机器人状态
 *
 */
enum Enum_Referee_Data_Event_Aerial_Status : uint8_t {
  Referee_Data_Event_Aerial_Status_COOLING = 0,
  Referee_Data_Event_Aerial_Status_READY,
  Referee_Data_Event_Aerial_Status_EXECUTING,
};

/**
 * @brief 伤害类型
 *
 */
enum Enum_Referee_Data_Event_Robot_Damage_Type {
  Referee_Data_Robot_Damage_Type_ARMOR_ATTACKED = 0,
  Referee_Data_Robot_Damage_Type_MODULE_OFFLINE,
  Referee_Data_Robot_Damage_Type_BOOSTER_SPEED,
  Referee_Data_Robot_Damage_Type_BOOSTER_HEAT,
  Referee_Data_Robot_Damage_Type_CHASSIS_POWER,
  Referee_Data_Robot_Damage_Type_ARMOR_COLLISION,
};

/**
 * @brief 子弹类型
 *
 */
enum Enum_Referee_Data_Robot_Ammo_Type : uint8_t {
  Referee_Data_Robot_Ammo_Type_BOOSTER_17MM = 1,
  Referee_Data_Robot_Ammo_Type_BOOSTER_42mm,
};

/**
 * @brief 发射机构类型
 *
 */
enum Enum_Referee_Data_Robot_Booster_Type : uint8_t {
  Referee_Data_Robot_Booster_Type_BOOSTER_17MM_1 = 1,
  Referee_Data_Robot_Booster_Type_BOOSTER_17MM_2,
  Referee_Data_Robot_Booster_Type_BOOSTER_42mm,
};

/**
 * @brief 飞镖发射口状态
 *
 */
enum Enum_Referee_Data_Robot_Dart_Command_Status : uint8_t {
  Referee_Data_Robot_Dart_Command_Status_OPEN = 0,
  Referee_Data_Robot_Dart_Command_Status_CLOSED,
  Referee_Data_Robot_Dart_Command_Status_EXECUTING,
};

/**
 * @brief 飞镖击打目标
 *
 */
enum Enum_Referee_Data_Robot_Dart_Command_Target : uint8_t {
  Referee_Data_Robot_Dart_Command_Target_OUTPOST = 0,
  Referee_Data_Robot_Dart_Command_Target_BASE,
};

/**
 * @brief 图形操作交互信息
 *
 */
enum Enum_Referee_Data_Interaction_Layer_Delete_Operation : uint8_t {
  Referee_Data_Interaction_Layer_Delete_Operation_NULL = 0,
  Referee_Data_Interaction_Layer_Delete_Operation_CLEAR_ONE,
  Referee_Data_Interaction_Layer_Delete_Operation_CLEAR_ALL,
};

/**
 * @brief 图形操作
 *
 */
enum Enum_Referee_Data_Interaction_Graphic_Operation {
  Referee_Data_Interaction_Graphic_Operation_NULL = 0,
  Referee_Data_Interaction_Graphic_Operation_ADD,
  Referee_Data_Interaction_Graphic_Operation_CHANGE,
  Referee_Data_Interaction_Graphic_Operation_DELETE,
};

/**
 * @brief 图形类型
 *
 */
enum Enum_Referee_Data_Interaction_Graphic_Type {
  Referee_Data_Interaction_Graphic_Type_LINE = 0,
  Referee_Data_Interaction_Graphic_Type_RECTANGLE,
  Referee_Data_Interaction_Graphic_Type_CIRCLE,
  Referee_Data_Interaction_Graphic_Type_OVAL,
  Referee_Data_Interaction_Graphic_Type_ARC,
  Referee_Data_Interaction_Graphic_Type_FLOAT,
  Referee_Data_Interaction_Graphic_Type_INTEGER,
  Referee_Data_Interaction_Graphic_Type_STRING,
};

/**
 * @brief 图形颜色
 *
 */
enum Enum_Referee_Data_Interaction_Graphic_Color {
  Referee_Data_Interaction_Graphic_Color_MAIN = 0,
  Referee_Data_Interaction_Graphic_Color_YELLOW,
  Referee_Data_Interaction_Graphic_Color_GREEN,
  Referee_Data_Interaction_Graphic_Color_ORANGE,
  Referee_Data_Interaction_Graphic_Color_PURPLE,
  Referee_Data_Interaction_Graphic_Color_PINK,
  Referee_Data_Interaction_Graphic_Color_CYAN,
  Referee_Data_Interaction_Graphic_Color_BLACK,
  Referee_Data_Interaction_Graphic_Color_WHITE,
};

/**
 * @brief 图形操作交互信息
 *
 */
enum Enum_Referee_Data_Interaction_Semiautomatic_Command : uint8_t {
  Referee_Data_Interaction_Semiautomatic_Command_ATTACK = 1,
  Referee_Data_Interaction_Semiautomatic_Command_DEFENCE,
  Referee_Data_Interaction_Semiautomatic_Command_MOVE,
};

/**
 * @brief 图形配置结构体
 * 根据协议表 1-26 和表 1-27，总长度 15 字节
 * 
 */
struct Struct_Referee_Data_Interaction_Graphic_Config {
  uint8_t Index[3];             // 字节 0-2: 图形名 (3 字节 ASCII 码)
  uint32_t Operation : 3;       // 字节 3, bit 0-2: 图形操作 (0-空操作，1-增加，2-修改，3-删除)
  uint32_t Figure_Type : 3;     // 字节 3, bit 3-5: 图形类型 (0-直线，1-矩形，2-正圆，3-椭圆，4-圆弧，5-浮点数，6-整型数，7-字符)
  uint32_t Layer : 4;           // 字节 3, bit 6-9: 图层数 (0-9)
  uint32_t Color : 4;           // 字节 3, bit 10-13: 颜色 (0-红/蓝，1-黄色，2-绿色，3-橙色，4-紫红色，5-粉色，6-青色，7-黑色，8-白色)
  uint32_t Details_A : 9;       // 字节 4-5, bit 0-8: 根据图形类型不同含义不同 (线宽/字体大小/起始角度等)
  uint32_t Details_B : 9;       // 字节 5-6, bit 9-17: 根据图形类型不同含义不同 (无作用/终止角度/字符长度等)
  uint32_t Details_C : 10;      // 字节 6-7, bit 18-27: 半径/x 半轴长度/32 位整型数低 10 位等
  uint32_t Details_D : 11;      // 字节 7-8, bit 28-38: 终点 x/对角顶点 x/y 半轴长度/32 位整型数中间 11 位等
  uint32_t Details_E : 11;      // 字节 8-9, bit 39-49: 终点 y/对角顶点 y/32 位整型数高 11 位等
  uint32_t Start_X : 11;        // 字节 9-10, bit 50-60: 起点/圆心 x 坐标
  uint32_t Start_Y : 11;        // 字节 10-11, bit 61-71: 起点/圆心 y 坐标
  uint32_t Reserved : 16;       // 字节 12-13: 保留位 (协议中图形配置 3 为保留)
} __attribute__((packed));

/**
 * @brief 裁判系统源数据
 *
 */
struct Struct_Referee_UART_Data {
  uint8_t Frame_Header = 0xa5;
  uint16_t Data_Length;
  uint8_t Sequence;
  uint8_t CRC_8;
  Enum_Referee_Command_ID Referee_Command_ID;
  uint8_t Data[121];
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0001比赛状态, 3Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Game_Status {
  uint8_t Type_Enum : 4;
  uint8_t Stage_Enum : 4;
  uint16_t Remaining_Time;
  uint64_t Timestamp;  // 2026协议更新为8字节
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0002比赛结果, 比赛结束后发送
 *
 */
struct Struct_Referee_Rx_Data_Game_Result {
  Enum_Referee_Game_Result Result;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0003机器人血量, 1Hz
 *
 */
struct Struct_Referee_Rx_Data_Game_Robot_HP {
  uint16_t Red_Hero_1;
  uint16_t Red_Engineer_2;
  uint16_t Red_Infantry_3;
  uint16_t Red_Infantry_4;
  uint16_t Red_Infantry_5;
  uint16_t Red_Sentry_7;
  uint16_t Red_Outpost_11;
  uint16_t Red_Base_10;
  uint16_t Blue_Hero_1;
  uint16_t Blue_Engineer_2;
  uint16_t Blue_Infantry_3;
  uint16_t Blue_Infantry_4;
  uint16_t Blue_Infantry_5;
  uint16_t Blue_Sentry_7;
  uint16_t Blue_Outpost_11;
  uint16_t Blue_Base_10;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0101场地事件, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Event_Self_Data {
  uint32_t Supply_Front_Status_Enum : 1;    // bit 0
  uint32_t Supply_Inner_Status_Enum : 1;    // bit 1
  uint32_t Supply_Status_Enum : 1;          // bit 2
  uint32_t Power_Rune_Status_Enum : 4;      // bit 3-6
  uint32_t Highland_Center_Status_Enum : 2; // bit 7-8
  uint32_t Highland_Trap_Status_Enum : 2;   // bit 9-10
  uint32_t Enemy_Dart_Hit_Time : 9;         // bit 11-19
  uint32_t Enemy_Dart_Hit_Target_Enum : 3;  // bit 20-22
  uint32_t Middle_Buff_Status_Enum : 2;     // bit 23-24
  uint32_t Fortress_Buff_Status_Enum : 2;   // bit 25-26
  uint32_t Outpost_Buff_Status_Enum : 2;    // bit 27-28
  uint32_t Base_Buff_Status_Enum : 1;       // bit 29
  uint32_t Reserved : 2;                    // bit 30-31
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0104裁判警告信息, 判罚发生后发送
 *
 */
struct Struct_Referee_Rx_Data_Event_Referee_Warning {
  Enum_Referee_Data_Event_Referee_Warning_Level Level;
  Enum_Referee_Data_Robot_ID Robot_ID;
  uint8_t Count;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0105飞镖15s倒计时, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Event_Dart_Status {
  uint8_t Dart_Remaining_Time;
  uint16_t Dart_Hit_Target_Enum_Last : 2;
  uint16_t Dart_Hit_Target_Count : 3;
  uint16_t Dart_Hit_Target_Enum_Now : 2;
  uint16_t Reserved : 9;
  uint16_t Dart_Target_Change_Time;
  uint16_t Dart_Latest_Launch_Cmd_Time;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0201机器人状态, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Status {
  Enum_Referee_Data_Robots_ID Robot_ID;
  uint8_t Level;
  uint16_t HP;
  uint16_t HP_Max;
  uint16_t Booster_Heat_CD;
  uint16_t Booster_Heat_Max;
  uint16_t Chassis_Power_Max;
  uint8_t PM01_Gimbal_Status_Enum : 1;
  uint8_t PM01_Chassis_Status_Enum : 1;
  uint8_t PM01_Booster_Status_Enum : 1;
  uint8_t Reserved : 5;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0202当前机器人实时功率热量, 50Hz发送
 * 电压mV, 电流mA
 *
 */
struct Struct_Referee_Rx_Data_Robot_Power_Heat {
  uint16_t Chassis_Voltage;
  uint16_t Chassis_Current;
  float Chassis_Power;
  uint16_t Chassis_Energy_Buffer;
  uint16_t Booster_17mm_1_Heat;
  uint16_t Booster_17mm_2_Heat;
  uint16_t Booster_42mm_Heat;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0203当前机器人实时位置, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Position {
  float Location_X;
  float Location_Y;
  float Location_Yaw;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0204当前机器人增益, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Buff {
  uint8_t HP_Buff_Percent;
  uint16_t Booster_Heat_CD_Buff_Value;
  uint8_t Defend_Buff_Percent;
  uint8_t Defend_Debuff_Percent;
  uint16_t Damage_Buff_Percent;
  uint8_t Remaining_Energy_Back;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0205无人机可攻击时间, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Aerial_Status {
  Enum_Referee_Data_Event_Aerial_Status Aerial_Status;
  uint8_t Remaining_Time;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0206伤害情况, 伤害发生后发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Damage {
  uint8_t Armor_ID : 4;
  uint8_t Type_Enum : 4;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0207子弹信息, 射击发生后发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Booster {
  Enum_Referee_Data_Robot_Ammo_Type Ammo_Type;
  Enum_Referee_Data_Robot_Booster_Type Booster_Type;
  uint8_t Frequency;
  float Speed;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0208子弹剩余信息, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Remaining_Ammo {
  uint16_t Booster_17mm;
  uint16_t Booster_42mm;
  uint16_t Money;
  uint16_t Projectile_Allowance_Fortess;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x0209RFID状态信息, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_RFID {
  // === 第1字节 (bit 0 ~ 7) ===
  uint32_t Base_Status_Enum : 1;             // bit 0: 己方基地增益点
  uint32_t Highland_2_Self_Status_Enum : 1;  // bit 1: 对方中央高地增益点
  uint32_t Highland_2_Enemy_Status_Enum : 1; // bit 2: 己方桥底高地增益点
  uint32_t Highland_3_Self_Status_Enum : 1;  // bit 3: 对方桥底高地增益点
  uint32_t Highland_3_Enemy_Status_Enum
      : 1; // bit 4: 己方地形防地增益点(飞坡) (靠近己方一侧飞坡前)
  uint32_t Highland_4_Self_Status_Enum
      : 1; // bit 5: 己方地形防地增益点(飞坡) (靠近己方一侧飞坡后)
  uint32_t Highland_4_Enemy_Status_Enum
      : 1; // bit 6: 对方地形防地增益点(飞坡) (靠近对方一侧飞坡前)
  uint32_t Energy_Status_Enum
      : 1; // bit 7: 对方地形防地增益点(飞坡) (靠近对方一侧飞坡后)

  // === 第2字节 (bit 8 ~ 15) ===
  uint32_t Flyover_1_Self_Status_Enum
      : 1; // bit 8: 己方地形防地增益点(中央高地下方)
  uint32_t Flyover_2_Self_Status_Enum
      : 1; // bit 9: 己方地形防地增益点(中央高地左方)
  uint32_t Flyover_1_Enemy_Status_Enum
      : 1; // bit 10: 对方地形防地增益点(中央高地下方)
  uint32_t Flyover_2_Enemy_Status_Enum
      : 1;                          // bit 11: 对方地形防地增益点(中央高地左方)
  uint32_t Outpost_Status_Enum : 1; // bit 12: 己方地形防地增益点(公路下方)
  uint32_t HP_Status_Enum : 1;      // bit 13: 己方地形防地增益点(公路上方)
  uint32_t Sentry_Self_Status_Enum : 1;  // bit 14: 对方地形防地增益点(公路下方)
  uint32_t Sentry_Enemy_Status_Enum : 1; // bit 15: 对方地形防地增益点(公路上方)

  // === 第3字节 (bit 16 ~ 23) ===
  uint32_t Engineer_Self_Status_Enum : 1;  // bit 16: 己方堡垒增益点
  uint32_t Engineer_Enemy_Status_Enum : 1; // bit 17: 对方翻转增益点
  uint32_t Engineer_Exchange_Status_Enum
      : 1; // bit 18: 己方与资源区不重叠的补给区(RMUL 补给区)
  uint32_t Middle_Status_Enum : 1;      // bit 19: 己方与资源区重叠的补给区
  uint32_t Match_Point_Status_Enum : 1; // bit 20: 己方匹配增益点
  uint32_t RFID_Match_Point_Self_Status_Enum : 1; // bit 21: 对方匹配增益点
  uint32_t RFID_Match_Point_Enemy_Status_Enum
      : 1;                                // bit 22: 中心增益点 (仅 RMUL 适用)
  uint32_t Fortress_Self_Status_Enum : 1; // bit 23: 对方堡垒增益点

  // === 第4字节 (bit 24 ~ 31) ===
  uint32_t Flip_Self_Status_Enum : 1; // bit 24: 对方翻转增益点
  uint32_t Flip_Enemy_Status_Enum
      : 1; // bit 25: 对方地形防地增益点(隧道) (靠近己方一侧公路区下方)
  uint32_t Tunnel_Self_Low_Road_Status_Enum
      : 1; // bit 26: 己方地形跨越增益点(隧道) (靠近己方一侧公路区下方)
  uint32_t Tunnel_Self_Mid_Road_Status_Enum
      : 1; // bit 27: 己方地形跨越增益点(隧道) (靠近己方一侧公路区中间)
  uint32_t Tunnel_Self_High_Road_Status_Enum
      : 1; // bit 28: 己方地形跨越增益点(隧道) (靠近己方一侧公路区上方)
  uint32_t Tunnel_Self_Low_Hill_Status_Enum
      : 1; // bit 29: 己方地形跨越增益点(隧道) (靠近己方梯形高地较低处)
  uint32_t Tunnel_Self_Mid_Hill_Status_Enum
      : 1; // bit 30: 己方地形跨越增益点(隧道) (靠近己方梯形高地较中间)
  uint32_t Tunnel_Self_High_Hill_Status_Enum
      : 1; // bit 31: 己方地形跨越增益点(隧道) (靠近己方梯形高地较高处)

  // === 第5字节 (扩展状态，bit 0 ~ 5 有效) ===
  uint8_t Tunnel_Enemy_Low_Road_Status_Enum
      : 1; // bit 0: 对方地形跨越增益点(隧道) (靠近对方公路一侧下方)
  uint8_t Tunnel_Enemy_Mid_Road_Status_Enum
      : 1; // bit 1: 对方地形跨越增益点(隧道) (靠近对方公路一侧中间)
  uint8_t Tunnel_Enemy_High_Road_Status_Enum
      : 1; // bit 2: 对方地形跨越增益点(隧道) (靠近对方公路一侧上方)
  uint8_t Tunnel_Enemy_Low_Hill_Status_Enum
      : 1; // bit 3: 对方地形跨越增益点(隧道) (靠近对方梯形高地较低处)
  uint8_t Tunnel_Enemy_Mid_Hill_Status_Enum
      : 1; // bit 4: 对方地形跨越增益点(隧道) (靠近对方梯形高地较中间)
  uint8_t Tunnel_Enemy_High_Hill_Status_Enum
      : 1; // bit 5: 对方地形跨越增益点(隧道) (靠近对方梯形高地较高处)
  uint8_t Reserved_Byte5 : 2; // bit 6~7: 保留位（未使用）
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x020a飞镖状态, 10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Dart_Command {
  Enum_Referee_Data_Robot_Dart_Command_Status Status;
  uint8_t Reserved;
  uint16_t Switch_Remaining_Time;
  uint16_t Launch_Remaining_Time;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x020b哨兵获取己方位置信息, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Sentry_Location {
  float Hero_1_X;
  float Hero_1_Y;
  float Engineer_2_X;
  float Engineer_2_Y;
  float Infantry_3_X;
  float Infantry_3_Y;
  float Infantry_4_X;
  float Infantry_4_Y;
  float Infantry_5_X;
  float Infantry_5_Y;
  float Reserved_1;
  float Reserved_2;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x020c雷达标记进度, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Radar_Mark {
  uint8_t Opposing_Hero_1_Mark : 1;
  uint8_t Opposing_Engineer_2_Mark : 1;
  uint8_t Opposing_Infantry_3_Mark : 1;
  uint8_t Opposing_Infantry_4_Mark : 1;
  uint8_t Opposing_Infantry_5_Mark : 1;
  uint8_t Opposing_Sentry_7_Mark : 1;
  uint8_t Our_Hero_1_Mark : 1;
  uint8_t Our_Engineer_2_Mark : 1;
  uint8_t Our_Infantry_3_Mark : 1;
  uint8_t Our_Infantry_4_Mark : 1;
  uint8_t Our_Infantry_5_Mark : 1;
  uint8_t Our_Sentry_7_Mark : 1;
  uint8_t Reserved : 4;
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x020d哨兵决策信息, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Sentry_Decision {
  uint32_t Ammo_Exchange_Number : 11;          // bit 0-10: 除远程兑换外，哨兵机器人成功兑换的允许发弹量
  uint32_t Remote_Exchange_Ammo_Times : 4;     // bit 11-14: 哨兵机器人成功远程兑换允许发弹量的次数
  uint32_t Remote_Exchange_HP_Times : 4;       // bit 15-18: 哨兵机器人成功远程兑换血量的次数
  uint32_t Free_Resurgence_Enable : 1;         // bit 19: 哨兵机器人当前是否可以确认免费复活
  uint32_t Resurgence_Now_Enable : 1;          // bit 20: 哨兵机器人当前是否可以兑换立即复活
  uint32_t Resurgence_Consume : 10;            // bit 21-30: 哨兵机器人当前若兑换立即复活需要花费的金币数
  uint32_t Reserved : 1;                       // bit 31: 保留位
  uint16_t Sentry_Off_Combat : 1;              // bit 0: 哨兵当前是否处于脱战状态
  uint16_t Ammo_Remaining_Exchange_Number : 11; // bit 1-11: 队伍 17mm 允许发弹量的剩余可兑换数
  uint16_t Now_Posture : 2;                    // bit 12-13: 哨兵当前姿态 (1-进攻，2-防御，3-移动)
  uint16_t Buff_Enable : 1;                    // bit 14: 己方能量机关是否能够进入正在激活状态
  uint16_t Reserved_2 : 1;                     // bit 15: 保留位
} __attribute__((packed));

/**
 * @brief 裁判系统经过处理的数据, 0x020e雷达决策信息, 1Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Robot_Radar_Decision {
  uint8_t Double_Damage_Chance : 2;        // bit 0-1: 雷达是否拥有触发双倍易伤的机会 (0-2 次)
  uint8_t Double_Damage_Enemy_Status : 1;  // bit 2: 对方是否正在被触发双倍易伤 (0-未触发，1-触发)
  uint8_t Encrypt_Level : 2;               // bit 3-4: 己方加密等级 (1-3 级)
  uint8_t Password_Modify_Enable : 1;      // bit 5: 当前是否可以修改密钥
  uint8_t Reserved : 2;                    // bit 6-7: 保留位
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301图形删除交互信息, 用户自主发送
 * Header 0x0100
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Layer_Delete {
  uint16_t Header = Referee_Interaction_Command_ID_UI_LAYER_DELETE;
  Enum_Referee_Data_Robots_ID Sender;
  uint8_t Reserved;
  Enum_Referee_Data_Robots_Client_ID Receiver;
  Enum_Referee_Data_Interaction_Layer_Delete_Operation Operation;
  uint8_t Delete_Serial;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301画一个图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_1 {
  uint16_t Header = Referee_Interaction_Command_ID_UI_GRAPHIC_1;
  Enum_Referee_Data_Robots_ID Sender;
  uint8_t Reserved;
  Enum_Referee_Data_Robots_Client_ID Receiver;
  Struct_Referee_Data_Interaction_Graphic_Config Graphic[1];
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301画两个图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_2 {
  uint16_t Header = Referee_Interaction_Command_ID_UI_GRAPHIC_2;
  Enum_Referee_Data_Robots_ID Sender;
  uint8_t Reserved;
  Enum_Referee_Data_Robots_Client_ID Receiver;
  Struct_Referee_Data_Interaction_Graphic_Config Graphic[2];
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301画五个图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_5 {
  uint16_t Header = Referee_Interaction_Command_ID_UI_GRAPHIC_5;
  Enum_Referee_Data_Robots_ID Sender;
  uint8_t Reserved;
  Enum_Referee_Data_Robots_Client_ID Receiver;
  Struct_Referee_Data_Interaction_Graphic_Config Graphic[5];
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301画七个图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_7 {
  uint16_t Header = Referee_Interaction_Command_ID_UI_GRAPHIC_7;
  Enum_Referee_Data_Robots_ID Sender;
  uint8_t Reserved;
  Enum_Referee_Data_Robots_Client_ID Receiver;
  Struct_Referee_Data_Interaction_Graphic_Config Graphic[7];
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301画字符图形交互信息, 用户自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Graphic_String {
  uint16_t Header = Referee_Interaction_Command_ID_UI_GRAPHIC_STRING;
  Enum_Referee_Data_Robots_ID Sender;
  uint8_t Reserved;
  Enum_Referee_Data_Robots_Client_ID Receiver;
  Struct_Referee_Data_Interaction_Graphic_Config Graphic_String;
  uint8_t String[30];
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301哨兵自主决策交互信息, 哨兵自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Sentry {
  uint16_t Header = Referee_Interaction_Command_ID_SENTRY;
  uint32_t Confirm_Respawn : 1;           // bit 0: 确认复活 (0-不复活，1-复活)
  uint32_t Confirm_Exchange_Respawn : 1;  // bit 1: 确认兑换立即复活
  uint32_t Request_Exchange_Ammo_Number : 11;  // bit 2-12: 弹药数量 (0-2047)
  uint32_t Request_Remote_Exchange_Ammo_Times : 4;  // bit 13-16: 远程兑换弹药量请求次数
  uint32_t Request_Remote_Exchange_HP_Times : 4;    // bit 17-20: 远程兑换血量请求次数
  uint32_t Sentry_Mode : 2;               // bit 21-22: 哨兵姿态 (1-进攻，2-防御，3-移动)
  uint32_t Confirm_Activate_Energy : 1;   // bit 23: 确认激活能量机关
  uint32_t Reserved : 8;                  // bit 24-31: 保留位
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0301雷达自主决策交互信息, 雷达自主发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Radar {
  uint16_t Header = Referee_Interaction_Command_ID_RADAR;
  Enum_Referee_Data_Status Request_Double_Damage;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0302自定义控制器交互信息, 用户自主发送
 * TODO 视情况赋予Data含义
 *
 */
struct Struct_Referee_Rx_Data_Interaction_Custom_Controller {
  uint8_t Data[30];
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0303客户端发送小地图交互信息,
 * 用户自主最高2Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Interaction_Robot_Receive_Client_Minimap {
  float Target_Position_X;       // 字节 0-3: 目标位置 x 轴坐标，单位 m
  float Target_Position_Y;       // 字节 4-7: 目标位置 y 轴坐标，单位 m
  uint8_t Cmd_Keyboard;          // 字节 8: 云台手按下的键盘按键通用键值
  uint8_t Target_Robot_ID;       // 字节 9: 对方机器人 ID
  uint16_t Cmd_Source;           // 字节 10-11: 信息来源 ID
  uint16_t Reserved;             // 字节 12-13: 保留位
} __attribute__((packed));

/**
 * @brief 裁判系统发送的数据, 0x0304图传键鼠遥控交互信息, 30Hz发送
 * TODO 等待扩展
 *
 */
struct Struct_Referee_Rx_Data_Interaction_Robot_Receive_Client_Remote_Control {
  uint16_t Mouse_X;
  uint16_t Mouse_Y;
  uint16_t Mouse_Z;
  Enum_Referee_Data_Status Mouse_Left_Key_Status;
  Enum_Referee_Data_Status Mouse_Right_Key_Status;
  uint16_t Keyboard_Key;
  uint16_t Reserved;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0305客户端接收小地图交互信息,
 * 用户自主最高10Hz发送
 *
 */
struct Struct_Referee_Rx_Data_Interaction_Client_Receive_Radar {
  uint16_t Hero_Position_X;        // 字节 0-1: 英雄机器人 x 位置坐标，单位 cm
  uint16_t Hero_Position_Y;        // 字节 2-3: 英雄机器人 y 位置坐标，单位 cm
  uint16_t Engineer_Position_X;    // 字节 4-5: 工程机器人 x 位置坐标，单位 cm
  uint16_t Engineer_Position_Y;    // 字节 6-7: 工程机器人 y 位置坐标，单位 cm
  uint16_t Infantry_3_Position_X;  // 字节 8-9: 3 号步兵机器人 x 位置坐标，单位 cm
  uint16_t Infantry_3_Position_Y;  // 字节 10-11: 3 号步兵机器人 y 位置坐标，单位 cm
  uint16_t Infantry_4_Position_X;  // 字节 12-13: 4 号步兵机器人 x 位置坐标，单位 cm
  uint16_t Infantry_4_Position_Y;  // 字节 14-15: 4 号步兵机器人 y 位置坐标，单位 cm
  uint16_t Reserved_1;             // 字节 16-17: 保留位
  uint16_t Reserved_2;             // 字节 18-19: 保留位
  uint16_t Sentry_Position_X;      // 字节 20-21: 哨兵机器人 x 位置坐标，单位 cm
  uint16_t Sentry_Position_Y;      // 字节 22-23: 哨兵机器人 y 位置坐标，单位 cm
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0306客户端接收模拟键鼠遥控交互信息,
 * 用户自主最高30Hz发送
 * TODO 等待扩展
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Client_Receive_Custom_Controller {
  uint16_t Keyboard_Key;
  uint16_t Mouse_X : 12;
  Enum_Referee_Data_Status Mouse_Left_Key_Status : 4;
  uint16_t Mouse_Y : 12;
  Enum_Referee_Data_Status Mouse_Right_Key_Status : 4;
  uint16_t Reserved;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0307客户端接收模拟键鼠遥控交互信息,
 * 用户自主最高30Hz发送
 *
 */
struct
    Struct_Referee_Tx_Data_Interaction_Client_Receive_Sentry_Semiautomatic_Minimap {
  Enum_Referee_Data_Interaction_Semiautomatic_Command Command;
  uint16_t Start_X;
  uint16_t Start_Y;
  int8_t Delta_X_List[49];
  int8_t Delta_Y_List[49];
  Enum_Referee_Data_Robots_Client_ID Sender_ID;
} __attribute__((packed));

/**
 * @brief 裁判系统接收的数据, 0x0308客户端接收模拟键鼠遥控交互信息,
 * 用户自主最高30Hz发送
 *
 */
struct Struct_Referee_Tx_Data_Interaction_Client_Receive_Robot_Minimap {
  Enum_Referee_Data_Robots_Client_ID Sender_ID;
  Enum_Referee_Data_Robots_Client_ID Receiver_ID;
  uint8_t Data[30];
} __attribute__((packed));

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

// --- 核心类定义 ---

class Class_Referee {
public:
    void Init(UART_HandleTypeDef *huart);
    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);
    void TIM_Alive_PeriodElapsedCallback();

    // UI 发送
    void Send_UI_Graphic_1(uint8_t layer, const char name[3], Struct_Referee_Data_Interaction_Graphic_Config *graphic);
    void Send_UI_Graphic_2(uint8_t layer, const char name[3], Struct_Referee_Data_Interaction_Graphic_Config *graphic);
    void Send_UI_Graphic_5(uint8_t layer, const char name[3], Struct_Referee_Data_Interaction_Graphic_Config *graphic);
    void Send_UI_Graphic_7(uint8_t layer, const char name[3], Struct_Referee_Data_Interaction_Graphic_Config *graphic);
    void Send_UI_String(uint8_t layer, const char name[3], const char* str, uint16_t start_x, uint16_t start_y);
    void Send_UI_Layer_Delete(uint8_t layer, uint8_t operation = 1, uint8_t delete_serial = 0);

    // 哨兵/雷达决策发送
    void Send_Sentry_Decision(uint8_t confirm_respawn, uint8_t confirm_exchange_respawn, 
                              uint16_t ammo_number, uint8_t remote_exchange_ammo_times, 
                              uint8_t remote_exchange_hp_times, uint8_t sentry_mode,
                              uint8_t confirm_activate_energy);
    void Send_Radar_Decision(uint8_t request_double_damage);

    // 自定义控制器数据发送
    void Send_Custom_Controller_Data(uint8_t *data, uint8_t len);

    // 小地图数据发送
    void Send_Client_Receive_Radar(uint16_t *robot_positions, uint8_t len);
    void Send_Robot_Minimap(uint16_t sender_id, uint16_t receiver_id, uint8_t *data, uint8_t len);

    // 全量公共数据
    Struct_Referee_Rx_Data_Game_Status      Game_Status{};
    Struct_Referee_Rx_Data_Game_Result      Game_Result{};
    Struct_Referee_Rx_Data_Game_Robot_HP    Robot_HP{};
    Struct_Referee_Rx_Data_Event_Self_Data       Event_Data{};
    Struct_Referee_Rx_Data_Event_Referee_Warning Referee_Warning{};
    Struct_Referee_Rx_Data_Robot_Status     Robot_Status{};
    Struct_Referee_Rx_Data_Robot_Power_Heat       Power_Heat{};
    Struct_Referee_Rx_Data_Robot_Position        Robot_Pos{};
    Struct_Referee_Rx_Data_Robot_Buff       Robot_Buff{};
    Struct_Referee_Rx_Data_Robot_Aerial_Status   Aerial_Status{};
    Struct_Referee_Rx_Data_Robot_Damage      Hurt_Data{};
    Struct_Referee_Rx_Data_Robot_Booster      Shoot_Data{};
    Struct_Referee_Rx_Data_Robot_Remaining_Ammo      Ammo_Remain{};
    Struct_Referee_Rx_Data_Robot_RFID      RFID_Status{};
    Struct_Referee_Rx_Data_Robot_Dart_Command    Dart_Command{};
    Struct_Referee_Rx_Data_Robot_Sentry_Location Sentry_Location{};
    Struct_Referee_Rx_Data_Robot_Radar_Mark      Radar_Mark{};
    Struct_Referee_Rx_Data_Robot_Sentry_Decision Sentry_Decision{};
    Struct_Referee_Rx_Data_Robot_Radar_Decision  Radar_Decision{};

    Struct_Referee_Rx_Data_Event_Dart_Status      Dart_Status{};

    // 交互数据接收
    Struct_Referee_Rx_Data_Interaction_Custom_Controller Custom_Controller_Data{};
    Struct_Referee_Rx_Data_Interaction_Robot_Receive_Client_Minimap Client_Minimap{};
    Struct_Referee_Rx_Data_Interaction_Robot_Receive_Client_Remote_Control Client_Remote_Control{};
    Struct_Referee_Rx_Data_Interaction_Client_Receive_Radar Client_Receive_Radar{};

    // Inline getter methods for data access
    inline uint8_t Get_Game_Type() { return Game_Status.Type_Enum; }
    inline uint8_t Get_Game_Progress() { return Game_Status.Stage_Enum; }
    inline uint16_t Get_Stage_Remain_Time() { return Game_Status.Remaining_Time; }
    inline uint64_t Get_Sync_Time_Stamp() { return Game_Status.Timestamp; }
    inline uint8_t Get_Winner() { return Game_Result.Result; }
    inline uint16_t Get_Red_Hero_HP() { return Robot_HP.Red_Hero_1; }
    inline uint16_t Get_Red_Engineer_HP() { return Robot_HP.Red_Engineer_2; }
    inline uint16_t Get_Red_Infantry_3_HP() { return Robot_HP.Red_Infantry_3; }
    inline uint16_t Get_Red_Infantry_4_HP() { return Robot_HP.Red_Infantry_4; }
    inline uint16_t Get_Red_Infantry_5_HP() { return Robot_HP.Red_Infantry_5; }
    inline uint16_t Get_Red_Sentry_HP() { return Robot_HP.Red_Sentry_7; }
    inline uint16_t Get_Red_Outpost_HP() { return Robot_HP.Red_Outpost_11; }
    inline uint16_t Get_Red_Base_HP() { return Robot_HP.Red_Base_10; }
    inline uint16_t Get_Blue_Hero_HP() { return Robot_HP.Blue_Hero_1; }
    inline uint16_t Get_Blue_Engineer_HP() { return Robot_HP.Blue_Engineer_2; }
    inline uint16_t Get_Blue_Infantry_3_HP() { return Robot_HP.Blue_Infantry_3; }
    inline uint16_t Get_Blue_Infantry_4_HP() { return Robot_HP.Blue_Infantry_4; }
    inline uint16_t Get_Blue_Infantry_5_HP() { return Robot_HP.Blue_Infantry_5; }
    inline uint16_t Get_Blue_Sentry_HP() { return Robot_HP.Blue_Sentry_7; }
    inline uint16_t Get_Blue_Outpost_HP() { return Robot_HP.Blue_Outpost_11; }
    inline uint16_t Get_Blue_Base_HP() { return Robot_HP.Blue_Base_10; }
    inline uint32_t Get_Event_Data_Raw() { 
        uint32_t result;
        memcpy(&result, &Event_Data, sizeof(uint32_t));
        return result; 
    }
    inline uint8_t Get_Robot_ID() { return Robot_Status.Robot_ID; }
    inline uint8_t Get_Robot_Level() { return Robot_Status.Level; }
    inline uint16_t Get_Current_HP() { return Robot_Status.HP; }
    inline uint16_t Get_Maximum_HP() { return Robot_Status.HP_Max; }
    inline uint16_t Get_Shooter_Barrel_Cooling_Value() { return Robot_Status.Booster_Heat_CD; }
    inline uint16_t Get_Shooter_Barrel_Heat_Limit() { return Robot_Status.Booster_Heat_Max; }
    inline uint16_t Get_Chassis_Power_Limit() { return Robot_Status.Chassis_Power_Max; }
    inline bool Get_Power_Gimbal_Output() { return Robot_Status.PM01_Gimbal_Status_Enum; }
    inline bool Get_Power_Chassis_Output() { return Robot_Status.PM01_Chassis_Status_Enum; }
    inline bool Get_Power_Shooter_Output() { return Robot_Status.PM01_Booster_Status_Enum; }
    inline uint16_t Get_Chassis_Voltage() { return Power_Heat.Chassis_Voltage; }
    inline uint16_t Get_Chassis_Current() { return Power_Heat.Chassis_Current; }
    inline float Get_Chassis_Power() { return Power_Heat.Chassis_Power; }
    inline uint16_t Get_Buffer_Energy() { return Power_Heat.Chassis_Energy_Buffer; }
    inline uint16_t Get_Shooter_17mm_1_Heat() { return Power_Heat.Booster_17mm_1_Heat; }
    inline uint16_t Get_Shooter_17mm_2_Heat() { return Power_Heat.Booster_17mm_2_Heat; }
    inline uint16_t Get_Shooter_42mm_Heat() { return Power_Heat.Booster_42mm_Heat; }
    inline float Get_Robot_Pos_X() { return Robot_Pos.Location_X; }
    inline float Get_Robot_Pos_Y() { return Robot_Pos.Location_Y; }
    inline float Get_Robot_Pos_Angle() { return Robot_Pos.Location_Yaw; }
    inline uint8_t Get_Recovery_Buff() { return Robot_Buff.HP_Buff_Percent; }
    inline uint16_t Get_Cooling_Buff() { return Robot_Buff.Booster_Heat_CD_Buff_Value; }
    inline uint8_t Get_Defence_Buff() { return Robot_Buff.Defend_Buff_Percent; }
    inline uint8_t Get_Vulnerability_Buff() { return Robot_Buff.Defend_Debuff_Percent; }
    inline uint16_t Get_Attack_Buff() { return Robot_Buff.Damage_Buff_Percent; }
    inline uint8_t Get_Remaining_Energy() { return Robot_Buff.Remaining_Energy_Back; }
    inline uint8_t Get_Hurt_Armor_ID() { return Hurt_Data.Armor_ID; }
    inline uint8_t Get_Hurt_Type() { return Hurt_Data.Type_Enum; }
    inline uint8_t Get_Shoot_Bullet_Type() { return Shoot_Data.Ammo_Type; }
    inline uint8_t Get_Shoot_Shooter_Number() { return Shoot_Data.Booster_Type; }
    inline uint8_t Get_Shoot_Launching_Frequency() { return Shoot_Data.Frequency; }
    inline float Get_Shoot_Initial_Speed() { return Shoot_Data.Speed; }
    inline uint16_t Get_Ammo_17mm_1_Remain() { return Ammo_Remain.Booster_17mm; }
    inline uint16_t Get_Ammo_17mm_2_Remain() { return 0; }
    inline uint16_t Get_Ammo_42mm_Remain() { return Ammo_Remain.Booster_42mm; }
    inline uint16_t Get_Coin_Remain() { return Ammo_Remain.Money; }
    inline uint32_t Get_RFID_Status() { 
        uint32_t result;
        memcpy(&result, &RFID_Status, sizeof(uint32_t));
        return result; 
    }
    inline uint8_t Get_RFID_Status_2() { return RFID_Status.Tunnel_Enemy_Low_Road_Status_Enum; }
    inline uint8_t Get_Dart_Launch_Opening_Status() { return Dart_Status.Dart_Remaining_Time; }
    inline uint16_t Get_Dart_Target_Change_Time() { return Dart_Status.Dart_Target_Change_Time; }
    inline uint16_t Get_Dart_Latest_Launch_Cmd_Time() { return Dart_Status.Dart_Latest_Launch_Cmd_Time; }

    // 新增getter方法
    inline uint8_t Get_Referee_Warning_Level() { return Referee_Warning.Level; }
    inline uint8_t Get_Referee_Warning_Robot_ID() { return Referee_Warning.Robot_ID; }
    inline uint8_t Get_Aerial_Status() { return Aerial_Status.Aerial_Status; }
    inline uint8_t Get_Aerial_Remaining_Time() { return Aerial_Status.Remaining_Time; }
    inline uint8_t Get_Dart_Command_Status() { return Dart_Command.Status; }
    inline uint16_t Get_Dart_Switch_Remaining_Time() { return Dart_Command.Switch_Remaining_Time; }
    inline uint16_t Get_Dart_Launch_Remaining_Time() { return Dart_Command.Launch_Remaining_Time; }

    // 哨兵位置信息
    inline float Get_Hero_1_X() { return Sentry_Location.Hero_1_X; }
    inline float Get_Hero_1_Y() { return Sentry_Location.Hero_1_Y; }
    inline float Get_Engineer_2_X() { return Sentry_Location.Engineer_2_X; }
    inline float Get_Engineer_2_Y() { return Sentry_Location.Engineer_2_Y; }
    inline float Get_Infantry_3_X() { return Sentry_Location.Infantry_3_X; }
    inline float Get_Infantry_3_Y() { return Sentry_Location.Infantry_3_Y; }
    inline float Get_Infantry_4_X() { return Sentry_Location.Infantry_4_X; }
    inline float Get_Infantry_4_Y() { return Sentry_Location.Infantry_4_Y; }
    inline float Get_Infantry_5_X() { return Sentry_Location.Infantry_5_X; }
    inline float Get_Infantry_5_Y() { return Sentry_Location.Infantry_5_Y; }

    // 雷达标记信息
    inline uint8_t Get_Opposing_Hero_1_Mark() { return Radar_Mark.Opposing_Hero_1_Mark; }
    inline uint8_t Get_Opposing_Engineer_2_Mark() { return Radar_Mark.Opposing_Engineer_2_Mark; }
    inline uint8_t Get_Opposing_Infantry_3_Mark() { return Radar_Mark.Opposing_Infantry_3_Mark; }
    inline uint8_t Get_Opposing_Infantry_4_Mark() { return Radar_Mark.Opposing_Infantry_4_Mark; }
    inline uint8_t Get_Opposing_Infantry_5_Mark() { return Radar_Mark.Opposing_Infantry_5_Mark; }
    inline uint8_t Get_Opposing_Sentry_7_Mark() { return Radar_Mark.Opposing_Sentry_7_Mark; }

    // 哨兵决策信息
    inline uint16_t Get_Sentry_Ammo_Exchange_Number() { return Sentry_Decision.Ammo_Exchange_Number; }
    inline uint8_t Get_Sentry_Remote_Exchange_Ammo_Times() { return Sentry_Decision.Remote_Exchange_Ammo_Times; }
    inline uint8_t Get_Sentry_Remote_Exchange_HP_Times() { return Sentry_Decision.Remote_Exchange_HP_Times; }
    inline uint8_t Get_Sentry_Free_Resurgence_Enable() { return Sentry_Decision.Free_Resurgence_Enable; }
    inline uint8_t Get_Sentry_Resurgence_Now_Enable() { return Sentry_Decision.Resurgence_Now_Enable; }
    inline uint16_t Get_Sentry_Resurgence_Consume() { return Sentry_Decision.Resurgence_Consume; }
    inline uint8_t Get_Sentry_Off_Combat() { return Sentry_Decision.Sentry_Off_Combat; }
    inline uint16_t Get_Sentry_Ammo_Remaining_Exchange_Number() { return Sentry_Decision.Ammo_Remaining_Exchange_Number; }
    inline uint8_t Get_Sentry_Now_Posture() { return Sentry_Decision.Now_Posture; }
    inline uint8_t Get_Sentry_Buff_Enable() { return Sentry_Decision.Buff_Enable; }

    // 雷达决策信息
    inline uint8_t Get_Radar_Double_Damage_Chance() { return Radar_Decision.Double_Damage_Chance; }
    inline uint8_t Get_Radar_Double_Damage_Enemy_Status() { return Radar_Decision.Double_Damage_Enemy_Status; }
    inline uint8_t Get_Radar_Encrypt_Level() { return Radar_Decision.Encrypt_Level; }
    inline uint8_t Get_Radar_Password_Modify_Enable() { return Radar_Decision.Password_Modify_Enable; }

    // 交互数据
    inline uint8_t* Get_Custom_Controller_Data() { return Custom_Controller_Data.Data; }
    inline float Get_Client_Minimap_Target_X() { return Client_Minimap.Target_Position_X; }
    inline float Get_Client_Minimap_Target_Y() { return Client_Minimap.Target_Position_Y; }
    inline uint8_t Get_Client_Minimap_Cmd_Keyboard() { return Client_Minimap.Cmd_Keyboard; }
    inline uint8_t Get_Client_Minimap_Target_Robot_ID() { return Client_Minimap.Target_Robot_ID; }
    inline uint16_t Get_Client_Remote_Mouse_X() { return Client_Remote_Control.Mouse_X; }
    inline uint16_t Get_Client_Remote_Mouse_Y() { return Client_Remote_Control.Mouse_Y; }
    inline uint16_t Get_Client_Remote_Mouse_Z() { return Client_Remote_Control.Mouse_Z; }
    inline uint8_t Get_Client_Remote_Mouse_Left_Key() { return Client_Remote_Control.Mouse_Left_Key_Status; }
    inline uint8_t Get_Client_Remote_Mouse_Right_Key() { return Client_Remote_Control.Mouse_Right_Key_Status; }
    inline uint16_t Get_Client_Remote_Keyboard_Key() { return Client_Remote_Control.Keyboard_Key; }
    
    // 客户端接收雷达小地图数据 (0x0305)
    inline uint16_t Get_Radar_Minimap_Hero_X() { return Client_Receive_Radar.Hero_Position_X; }
    inline uint16_t Get_Radar_Minimap_Hero_Y() { return Client_Receive_Radar.Hero_Position_Y; }
    inline uint16_t Get_Radar_Minimap_Engineer_X() { return Client_Receive_Radar.Engineer_Position_X; }
    inline uint16_t Get_Radar_Minimap_Engineer_Y() { return Client_Receive_Radar.Engineer_Position_Y; }
    inline uint16_t Get_Radar_Minimap_Infantry_3_X() { return Client_Receive_Radar.Infantry_3_Position_X; }
    inline uint16_t Get_Radar_Minimap_Infantry_3_Y() { return Client_Receive_Radar.Infantry_3_Position_Y; }
    inline uint16_t Get_Radar_Minimap_Infantry_4_X() { return Client_Receive_Radar.Infantry_4_Position_X; }
    inline uint16_t Get_Radar_Minimap_Infantry_4_Y() { return Client_Receive_Radar.Infantry_4_Position_Y; }
    inline uint16_t Get_Radar_Minimap_Sentry_X() { return Client_Receive_Radar.Sentry_Position_X; }
    inline uint16_t Get_Radar_Minimap_Sentry_Y() { return Client_Receive_Radar.Sentry_Position_Y; }

protected:
    UART_HandleTypeDef *huart;
    Struct_UART_Manage_Object *UART_Obj = nullptr;
    uint8_t Tx_Buffer[256];
    uint8_t Sequence = 0;
    uint32_t Flag = 0, Pre_Flag = 0;

    void Data_Process(uint8_t *pData, uint16_t Len);

    // CRC 函数
    uint8_t Get_CRC8(uint8_t *p, uint32_t len, uint8_t init);
    uint16_t Get_CRC16(uint8_t *p, uint32_t len, uint16_t init);
    void Append_CRC8_CRC16(uint8_t *pData, uint16_t TotalLen);
};



#endif // C_BOARD_REFEREE_H
