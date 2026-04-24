/**
 * @file debug_cmd.h
 * @brief 调试指令解析系统的定义与映射
 * @details 包含所有支持的指令枚举、字符串映射表以及解析函数的声明
 */

#ifndef DEBUG_CMD_H
#define DEBUG_CMD_H

#include <string>

#include "2_Device/DR16/dvc_dr16.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "3_Chariot/1_Module/Chassis/crt_chassis.h"
#include "stm32f407xx.h"

extern Class_Motor_DJI_C620 motor_x_p;  // ID: 0
extern Class_Motor_DJI_C620 motor_x_m;  // ID: 1
extern Class_Motor_DJI_C620 motor_y_p;  // ID: 2
extern Class_Motor_DJI_C620 motor_y_m;  // ID: 3
extern Chassis chassis;                 // 底盘控制实例
extern Class_DR16 dr16;

/**
 * @brief 电机实例指针数组，将 ID 与具体对象绑定
 */
static Class_Motor_DJI_C620* motor_list[] = {&motor_x_p, &motor_x_m, &motor_y_p, &motor_y_m};

// 获取电机数量的宏
#define MOTOR_COUNT (sizeof(motor_list) / sizeof(motor_list[0]))

/**
 * @brief 方便自动计算数组长度的宏
 */
#define CMD_COUNT(arr) (uint8_t)(sizeof(arr) / sizeof(char*))

/* ========================================================================= */
/* =                          1. 指令枚举定义                              = */
/* ========================================================================= */

/** @brief 顶级根指令枚举 */
enum class EnumCmdList : uint8_t {
    error = 0,
    motor,    // 电机调试
    chassis,  // 底盘调试
    remote,
    help  // 帮助菜单
};

/** @brief 电机子指令枚举 */
enum class EnumCmdMotorList : uint8_t {
    error = 0,
    get,  // 读取参数
    put   // 写入参数
};

/** @brief 电机状态项枚举 */
enum class EnumCmdMotorStateList : uint8_t {
    error = 0,
    target_angle,     // 目标角度
    target_omega,     // 目标速度
    target_current,   // 目标电流
    current_angle,    // 当前角度
    current_omega,    // 当前速度
    current_current,  // 当前电流
    current_power     // 当前功率
};

/** @brief PID 闭环类型 */
enum class EnumCmdPIDTypeList : uint8_t {
    error = 0,
    angle,  // 位置环/角度环
    omega   // 速度环
};

/** @brief PID 详细参数项 */
enum class EnumCmdPIDParamList : uint8_t {
    error = 0,
    p,
    i,
    d,
    f,
    i_limit,    // 积分限幅
    out_limit,  // 输出限幅
    dead_zone,  // 死区
    speed_a,    // 变速积分阈值A
    speed_b,    // 变速积分区间B
    separate    // 积分分离阈值
};

/** @brief 底盘子指令枚举 */
enum class EnumCmdChassisList : uint8_t { error = 0, get, put };

/** @brief 底盘状态/配置项枚举 */
enum class EnumCmdChassisStateList : uint8_t {
    error = 0,
    now_angle,    // 当前世界坐标角度
    target_v,     // 目标三轴速度 (vx, vy, vw)
    world_frame,  // 世界/相对坐标切换
    delay_comp,   // 相位延迟补偿时间 (ms)
    deadzone,     // 控制死区
    correction,   // 获取打滑修正值 (X, Y)
    raw_ticks,     // 获取四路电机原始脉冲
    IMU_Paw
};

enum class EnumCmdRemoteStateList : uint8_t {
    error = 0,
    stick,  // 一次性获取四个摇杆值
};

enum class EnumCmdRemoteList : uint8_t {
    error = 0,
    get,
};

/* ========================================================================= */
/* =                          2. 指令类定义与映射                          = */
/* ========================================================================= */

/**
 * @brief 指令字典类：将字符串数组与长度绑定
 */
class CmdClass {
   public:
    const char* const* cmdStr;  // 指向常量字符串数组的指针
    uint8_t cmdStrLength;       // 数组长度
};

/* --- 静态字符串数组 (词库) --- */

static const char* topStrings[] = {"motor", "chassis", "remote", "help"};
static const char* motorStrings[] = {"get", "put"};
static const char* motorStateStrings[] = {"target_angle",
                                          "target_omega",
                                          "target_current",
                                          "current_angle",
                                          "current_omega",
                                          "current_current",
                                          "current_power"};
static const char* PIDTypeStrings[] = {"angle", "omega"};
static const char* PIDParamStrings[] =
    {"p", "i", "d", "f", "i_limit", "out_limit", "dead_zone", "speed_a", "speed_b", "separate"};
static const char* chassisStrings[] = {"get", "put"};
static const char* chassisStateStrings[] =
    {"now_angle", "target_v", "world_frame", "delay_comp", "deadzone", "correction", "raw_ticks", "IMU_Paw"};

static const char* remoteStrings[] = {"get"};
static const char* remoteStateStrings[] = {"stick"};

/* --- 实例化指令对象 (供解析器查询) --- */

static const CmdClass CmdTopList = {topStrings, CMD_COUNT(topStrings)};
static const CmdClass CmdMotoList = {motorStrings, CMD_COUNT(motorStrings)};
static const CmdClass CmdMotorStateList = {motorStateStrings, CMD_COUNT(motorStateStrings)};
static const CmdClass CmdPIDTypeList = {PIDTypeStrings, CMD_COUNT(PIDTypeStrings)};
static const CmdClass CmdPIDParamList = {PIDParamStrings, CMD_COUNT(PIDParamStrings)};
static const CmdClass CmdChassisList = {chassisStrings, CMD_COUNT(chassisStrings)};
static const CmdClass CmdChassisStateList = {chassisStateStrings, CMD_COUNT(chassisStateStrings)};
static const CmdClass CmdRemoteList = {remoteStrings, CMD_COUNT(remoteStrings)};
static const CmdClass CmdRemoteStateList = {remoteStateStrings, CMD_COUNT(remoteStateStrings)};

/* ========================================================================= */
/* =                          3. 外部接口声明                              = */
/* ========================================================================= */

/**
 * @brief 解析入口
 * @param src 串口接收到的完整指令字符串
 */
void anysisCmd(const std::string src);

#endif  // DEBUG_CMD_H