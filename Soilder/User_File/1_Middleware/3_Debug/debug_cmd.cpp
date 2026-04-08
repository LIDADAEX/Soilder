/**
 * @file debug_cmd.cpp
 * @brief 调试指令解析系统
 * @details 支持通过串口指令实时读写电机状态、调整 PID 参数（含变速积分与积分分离）
 */

#include "debug_cmd.h"
#include <string>
#include <sstream>
#include <iostream>
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"

/* --- 类型转换工具函数 --- */

/**
 * @brief 基础转换模板：将字符串尝试转换为数值类型
 * @param s 输入字符串
 * @param result 转换结果存放引用
 * @return true 转换成功且无残留字符，false 格式错误或为空
 */
template <typename T>
bool tryStringToNumber(const std::string& s, T& result) {
    if (s.empty()) return false;
    std::stringstream ss(s);
    // 读入目标类型并检查是否到达流末尾，防止 "123a" 这种非法数据
    return (ss >> result) && (ss.eof());
}

// 转换为 int16_t (针对索引等整数)
bool toNumber(const std::string& s, int16_t& val) {
    return tryStringToNumber(s, val);
}

// 转换为 uint16_t (针对电机ID，带负号检查)
bool toNumber(const std::string& s, uint16_t& val) {
    if (!s.empty() && s[0] == '-') return false;
    return tryStringToNumber(s, val);
}

// 转换为 float (针对PID参数、角度、电流等)
bool toNumber(const std::string& s, float& val) {
    return tryStringToNumber(s, val);
}

/* --- 指令预处理函数 --- */

/**
 * @brief 将原始字符串按空格切分为子字符串数组
 * @param src 原始输入行
 * @param subSrcList 存放切分结果的数组
 * @param maxCount 最大切分数量
 * @param actualCount 实际切分出的单词数
 */
void splitCmd(const std::string& src, std::string subSrcList[], uint8_t maxCount, uint8_t& actualCount) {
    std::stringstream ss(src);
    std::string item;
    actualCount = 0;
    while (ss >> item && actualCount < maxCount) {
        subSrcList[actualCount++] = item;
    }
}

/**
 * @brief 在给定的指令类中查找匹配项
 * @param subSrc 待匹配的单词
 * @param cmdClass 指令定义对象（含字符串列表和长度）
 * @return uint8_t 匹配到的索引+1 (对应枚举值)，未找到返回 0 (error)
 */
uint8_t findCmd(const std::string& subSrc, const CmdClass* const cmdClass) {
    for (uint8_t i = 0; i < cmdClass->cmdStrLength; i++) {
        if (subSrc == cmdClass->cmdStr[i]) {
            return i + 1; // 匹配成功，返回非零枚举值
        }
    }
    return 0; // 匹配失败，对应枚举中的 error
}

/* --- 错误提示与帮助 --- */

/**
 * @brief 统一错误反馈处理
 * @param lastCmd 上一级指令名称
 * @param cmdClass 当前预期的子指令集
 */
void errorHandle(const char* lastCmd, const CmdClass* const cmdClass){
    if(lastCmd != nullptr){
        printf("未知指令，%s 的下级指令应当为以下：\r\n", lastCmd);
        for(uint8_t i = 0; i < cmdClass->cmdStrLength; i ++){
            printf("  - %s\r\n", cmdClass->cmdStr[i]);
        }
    }
    else{
        printf("未知指令，请输入 .help 查看根指令集\r\n");
    }
}

/**
 * @brief 打印顶层指令帮助列表
 */
inline void Cmd_help(){
    printf("可用根指令列表：\r\n");
    for(uint8_t i = 0; i < CmdTopList.cmdStrLength; i ++){
        printf(" [%d] %s\r\n", i+1, CmdTopList.cmdStr[i]);
    }
}

/* --- 电机底层操作函数 (Getter/Setter) --- */

extern Class_Motor_DJI_C620 motor_x_p; // 挂载的电机实例

// 目标状态读取
inline void cmd_motor_get_target_angle(uint16_t motorNumber)   { printf("电机%d 目标角度: %f\r\n", motorNumber, motor_x_p.Get_Target_Angle()); }
inline void cmd_motor_get_target_omega(uint16_t motorNumber)   { printf("电机%d 目标角速度: %f\r\n", motorNumber, motor_x_p.Get_Target_Omega()); }
inline void cmd_motor_get_target_current(uint16_t motorNumber) { printf("电机%d 目标电流: %f\r\n", motorNumber, motor_x_p.Get_Target_Current()); }

// 实时状态读取
inline void cmd_motor_get_current_angle(uint16_t motorNumber)   { printf("电机%d 当前角度: %f\r\n", motorNumber, motor_x_p.Get_Now_Angle()); }
inline void cmd_motor_get_current_omega(uint16_t motorNumber)   { printf("电机%d 当前角速度: %f\r\n", motorNumber, motor_x_p.Get_Now_Omega()); }
inline void cmd_motor_get_current_current(uint16_t motorNumber) { printf("电机%d 当前电流: %f\r\n", motorNumber, motor_x_p.Get_Now_Current()); }
inline void cmd_motor_get_now_power(uint16_t motorNumber)       { printf("电机%d 实时功率: %f W\r\n", motorNumber, motor_x_p.Get_Now_Power()); }

// 目标状态写入
inline void cmd_motor_put_target_angle(uint16_t motorNumber, float angle)     { motor_x_p.Set_Target_Angle(angle); printf("OK. 电机%d 目标角度 -> %f\r\n", motorNumber, angle); }
inline void cmd_motor_put_target_omega(uint16_t motorNumber, float omega)     { motor_x_p.Set_Target_Omega(omega); printf("OK. 电机%d 目标角速度 -> %f\r\n", motorNumber, omega); }
inline void cmd_motor_put_target_current(uint16_t motorNumber, float current) { motor_x_p.Set_Target_Current(current); printf("OK. 电机%d 目标电流 -> %f\r\n", motorNumber, current); }

/* --- 核心业务逻辑处理器 --- */

/**
 * @brief 专门处理 PID 相关的参数读写
 * @param pid 目标 PID 对象的引用
 * @param op "get" 或 "put"
 * @param paramStr 参数名称字符串（p, i, d, speed_a 等）
 * @param value 如果是 put 操作，传入的新值
 */
inline void handle_pid_param_op(Class_PID& pid, const std::string& op, const std::string& paramStr, float value) {
    auto paramIdx = static_cast<EnumCmdPIDParamList>(findCmd(paramStr, &CmdPIDParamList));

    if (op == "get") {
        switch (paramIdx) {
            case EnumCmdPIDParamList::p:         printf("P: %f\r\n", pid.Get_K_P()); break;
            case EnumCmdPIDParamList::i:         printf("I: %f\r\n", pid.Get_K_I()); break;
            case EnumCmdPIDParamList::d:         printf("D: %f\r\n", pid.Get_K_D()); break;
            case EnumCmdPIDParamList::f:         printf("F: %f\r\n", pid.Get_K_F()); break;
            case EnumCmdPIDParamList::i_limit:   printf("积分限幅: %f\r\n", pid.Get_I_Out_Max()); break;
            case EnumCmdPIDParamList::out_limit: printf("输出限幅: %f\r\n", pid.Get_Out_Max()); break;
            case EnumCmdPIDParamList::dead_zone: printf("控制死区: %f\r\n", pid.Get_Dead_Zone()); break;
            case EnumCmdPIDParamList::speed_a:   printf("变速积分阈值A: %f\r\n", pid.Get_I_Variable_Speed_A()); break;
            case EnumCmdPIDParamList::speed_b:   printf("变速积分区间B: %f\r\n", pid.Get_I_Variable_Speed_B()); break;
            case EnumCmdPIDParamList::separate:  printf("积分分离阈值: %f\r\n", pid.Get_I_Separate_Threshold()); break;
            default: printf("错误: 未知的 PID 参数项\r\n"); break;
        }
    } else { // put 操作
        switch (paramIdx) {
            case EnumCmdPIDParamList::p:         pid.Set_K_P(value); break;
            case EnumCmdPIDParamList::i:         pid.Set_K_I(value); break;
            case EnumCmdPIDParamList::d:         pid.Set_K_D(value); break;
            case EnumCmdPIDParamList::f:         pid.Set_K_F(value); break;
            case EnumCmdPIDParamList::i_limit:   pid.Set_I_Out_Max(value); break;
            case EnumCmdPIDParamList::out_limit: pid.Set_Out_Max(value); break;
            case EnumCmdPIDParamList::dead_zone: pid.Set_Dead_Zone(value); break;
            case EnumCmdPIDParamList::speed_a:   pid.Set_I_Variable_Speed_A(value); break;
            case EnumCmdPIDParamList::speed_b:   pid.Set_I_Variable_Speed_B(value); break;
            case EnumCmdPIDParamList::separate:  pid.Set_I_Separate_Threshold(value); break;
            default: printf("错误: PID 参数设置失败\r\n"); return;
        }
        printf("Success: PID 参数已更新并生效\r\n");
    }
}

/**
 * @brief 解析 motor get 指令逻辑
 * 指令格式 1: motor get <ID> <StateName>
 * 指令格式 2: motor get <ID> <angle/omega> <PidParam>
 */
inline void handle_motor_get(const std::string subSrcList[], uint8_t count) {
    if (count < 4) return;

    uint16_t motorNumber;
    if (!toNumber(subSrcList[2], motorNumber) || motorNumber >= 4) return;

    // 优先识别是否为 PID 环路标识 (angle/omega)
    auto pidType = static_cast<EnumCmdPIDTypeList>(findCmd(subSrcList[3], &CmdPIDTypeList));
    
    if (pidType != EnumCmdPIDTypeList::error) {
        if (count < 5) { printf("用法: motor get <ID> <angle/omega> <param>\r\n"); return; }
        Class_PID& targetPid = (pidType == EnumCmdPIDTypeList::angle) ? motor_x_p.PID_Angle : motor_x_p.PID_Omega;
        handle_pid_param_op(targetPid, "get", subSrcList[4], 0);
    } else {
        // 普通状态读取
        switch (static_cast<EnumCmdMotorStateList>(findCmd(subSrcList[3], &CmdMotorStateList))) {
            case EnumCmdMotorStateList::target_angle:    cmd_motor_get_target_angle(motorNumber);    break;
            case EnumCmdMotorStateList::target_omega:    cmd_motor_get_target_omega(motorNumber);    break;
            case EnumCmdMotorStateList::target_current:  cmd_motor_get_target_current(motorNumber);  break;
            case EnumCmdMotorStateList::current_angle:   cmd_motor_get_current_angle(motorNumber);   break;
            case EnumCmdMotorStateList::current_omega:   cmd_motor_get_current_omega(motorNumber);   break;
            case EnumCmdMotorStateList::current_current: cmd_motor_get_current_current(motorNumber); break;
            case EnumCmdMotorStateList::current_power:   cmd_motor_get_now_power(motorNumber);       break;
            default: errorHandle("motor get", &CmdMotorStateList); break;
        }
    }
}

/**
 * @brief 解析 motor put 指令逻辑
 * 指令格式 1: motor put <ID> <StateName> <Value>
 * 指令格式 2: motor put <ID> <angle/omega> <PidParam> <Value>
 */
inline void handle_motor_put(const std::string subSrcList[], uint8_t count) {
    if (count < 5) return;

    uint16_t motorNumber;
    if (!toNumber(subSrcList[2], motorNumber) || motorNumber >= 4) return;

    auto pidType = static_cast<EnumCmdPIDTypeList>(findCmd(subSrcList[3], &CmdPIDTypeList));

    if (pidType != EnumCmdPIDTypeList::error) {
        if (count < 6) { printf("用法: motor put <ID> <angle/omega> <param> <value>\r\n"); return; }
        float val;
        if (!toNumber(subSrcList[5], val)) return;
        Class_PID& targetPid = (pidType == EnumCmdPIDTypeList::angle) ? motor_x_p.PID_Angle : motor_x_p.PID_Omega;
        handle_pid_param_op(targetPid, "put", subSrcList[4], val);
    } else {
        float motorValue;
        if (!toNumber(subSrcList[4], motorValue)) return;
        switch (static_cast<EnumCmdMotorStateList>(findCmd(subSrcList[3], &CmdMotorStateList))) {
            case EnumCmdMotorStateList::target_angle:   cmd_motor_put_target_angle(motorNumber, motorValue);   break;
            case EnumCmdMotorStateList::target_omega:   cmd_motor_put_target_omega(motorNumber, motorValue);   break;
            case EnumCmdMotorStateList::target_current: cmd_motor_put_target_current(motorNumber, motorValue); break;
            default: errorHandle("motor put", &CmdMotorStateList); break;
        }
    }
}

/**
 * @brief 电机类指令分发器
 */
inline void handle_motor_cmd(const std::string subSrcList[], uint8_t count) {
    if (count < 2) {
        errorHandle("motor", &CmdMotoList);
        return;
    }

    switch (static_cast<EnumCmdMotorList>(findCmd(subSrcList[1], &CmdMotoList))) {
        case EnumCmdMotorList::get:   handle_motor_get(subSrcList, count); break;
        case EnumCmdMotorList::put:   handle_motor_put(subSrcList, count); break;
        default:                      errorHandle("motor", &CmdMotoList);  break;
    }
}

/* --- 主入口函数 --- */

/**
 * @brief 调试指令解析总入口
 * @param src 串口接收到的完整字符串行
 */
void anysisCmd(const std::string src) {
    std::string subSrcList[8]; // 最大支持 8 个单词的复合指令
    uint8_t count = 0;
    splitCmd(src, subSrcList, 8, count);

    if (count == 0) return;

    // 匹配顶级指令 (motor, .help 等)
    switch (static_cast<EnumCmdList>(findCmd(subSrcList[0], &CmdTopList))) {
        case EnumCmdList::motor:
            handle_motor_cmd(subSrcList, count);
            break;

        case EnumCmdList::help:
            Cmd_help();
            break;

        default:
            errorHandle(nullptr, nullptr); // 未识别的顶级指令
            break;
    }
}