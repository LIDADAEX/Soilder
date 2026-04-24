/**
 * @file debug_cmd.cpp
 * @brief 调试指令解析系统实现
 * @author
 * @date 2026-04-12
 * * @details
 * 该系统支持通过字符串指令对机器人进行实时调参。
 * 支持功能：
 * 1. 读写大疆电机 (C620/C610等) 的目标值与状态。
 * 2. 动态调整 PID 参数（支持位置环/速度环、变速积分、积分分离）。
 * 3. 监控底盘世界坐标、目标速度及相位补偿。
 */

#include "debug_cmd.h"

#include <iostream>
#include <sstream>
#include <string>

/* 外部实例声明 */

/* ========================================================================= */
/* =                         1. 类型转换与字符串工具                         = */
/* ========================================================================= */

/**
 * @brief 模板函数：将字符串安全转换为数值
 * @return true 转换成功且无残留字符
 */
template <typename T>
static bool tryStringToNumber(const std::string& s, T& result) {
    if (s.empty())
        return false;
    std::stringstream ss(s);
    return (ss >> result) && (ss.eof());
}

bool toNumber(const std::string& s, int16_t& val) {
    return tryStringToNumber(s, val);
}
bool toNumber(const std::string& s, float& val) {
    return tryStringToNumber(s, val);
}

/** @brief 转换为 uint16_t，增加负号检查防止溢出错误 */
bool toNumber(const std::string& s, uint16_t& val) {
    if (!s.empty() && s[0] == '-')
        return false;
    return tryStringToNumber(s, val);
}

/**
 * @brief 指令切分函数
 * @param src 原始字符串（如 "motor get 0 p"）
 * @param subSrcList 存放切分后单词的数组
 * @param maxCount 最大切分数量
 * @param actualCount 实际切分出的单词数引用
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
 * @brief 在命令字典中查找匹配的枚举索引
 * @return 匹配到的枚举值 (Index + 1)，失败返回 0
 */
uint8_t findCmd(const std::string& subSrc, const CmdClass* const cmdClass) {
    for (uint8_t i = 0; i < cmdClass->cmdStrLength; i++) {
        if (subSrc == cmdClass->cmdStr[i])
            return i + 1;
    }
    return 0;
}

/* ========================================================================= */
/* =                         2. 交互反馈与帮助系统                         = */
/* ========================================================================= */

/** @brief 处理未知指令并打印提示信息 */
void errorHandle(const char* lastCmd, const CmdClass* const cmdClass) {
    if (lastCmd != nullptr && cmdClass != nullptr) {
        printf("错误: 未知指令。 [%s] 的子指令应为:\r\n", lastCmd);
        for (uint8_t i = 0; i < cmdClass->cmdStrLength; i++) {
            printf("  > %s\r\n", cmdClass->cmdStr[i]);
        }
    } else {
        printf("未知指令！输入 '.help' 查看根指令集。\r\n");
    }
}

/** @brief 打印根指令帮助列表 */
inline void Cmd_help() {
    printf("--- 可用根指令列表 ---\r\n");
    for (uint8_t i = 0; i < CmdTopList.cmdStrLength; i++) {
        printf(" [%d] %s\r\n", i + 1, CmdTopList.cmdStr[i]);
    }
}

/* ========================================================================= */
/* =                         3. PID 参数核心处理单元                        = */
/* ========================================================================= */

/**
 * @brief PID 参数统一读写接口
 * @param pid 目标 PID 类引用
 * @param op 操作类型 ("get" 或 "put")
 * @param paramStr 参数项字符串 (如 "p", "i", "speed_a")
 * @param value 写入值 (仅在 put 时有效)
 */
inline void handle_pid_param_op(Class_PID& pid, const std::string& op, const std::string& paramStr, float value) {
    auto paramIdx = static_cast<EnumCmdPIDParamList>(findCmd(paramStr, &CmdPIDParamList));

    if (op == "get") {
        switch (paramIdx) {
            case EnumCmdPIDParamList::p:
                printf("P: %f\r\n", pid.Get_K_P());
                break;
            case EnumCmdPIDParamList::i:
                printf("I: %f\r\n", pid.Get_K_I());
                break;
            case EnumCmdPIDParamList::d:
                printf("D: %f\r\n", pid.Get_K_D());
                break;
            case EnumCmdPIDParamList::f:
                printf("F: %f\r\n", pid.Get_K_F());
                break;
            case EnumCmdPIDParamList::i_limit:
                printf("积分限幅: %f\r\n", pid.Get_I_Out_Max());
                break;
            case EnumCmdPIDParamList::out_limit:
                printf("输出限幅: %f\r\n", pid.Get_Out_Max());
                break;
            case EnumCmdPIDParamList::dead_zone:
                printf("控制死区: %f\r\n", pid.Get_Dead_Zone());
                break;
            case EnumCmdPIDParamList::speed_a:
                printf("变速积分阈值A: %f\r\n", pid.Get_I_Variable_Speed_A());
                break;
            case EnumCmdPIDParamList::speed_b:
                printf("变速积分区间B: %f\r\n", pid.Get_I_Variable_Speed_B());
                break;
            case EnumCmdPIDParamList::separate:
                printf("积分分离阈值: %f\r\n", pid.Get_I_Separate_Threshold());
                break;
            default:
                printf("错误: PID 参数项 [%s] 不存在\r\n", paramStr.c_str());
                break;
        }
    } else {  // put
        switch (paramIdx) {
            case EnumCmdPIDParamList::p:
                pid.Set_K_P(value);
                break;
            case EnumCmdPIDParamList::i:
                pid.Set_K_I(value);
                break;
            case EnumCmdPIDParamList::d:
                pid.Set_K_D(value);
                break;
            case EnumCmdPIDParamList::f:
                pid.Set_K_F(value);
                break;
            case EnumCmdPIDParamList::i_limit:
                pid.Set_I_Out_Max(value);
                break;
            case EnumCmdPIDParamList::out_limit:
                pid.Set_Out_Max(value);
                break;
            case EnumCmdPIDParamList::dead_zone:
                pid.Set_Dead_Zone(value);
                break;
            case EnumCmdPIDParamList::speed_a:
                pid.Set_I_Variable_Speed_A(value);
                break;
            case EnumCmdPIDParamList::speed_b:
                pid.Set_I_Variable_Speed_B(value);
                break;
            case EnumCmdPIDParamList::separate:
                pid.Set_I_Separate_Threshold(value);
                break;
            default:
                printf("错误: PID 参数设置失败\r\n");
                return;
        }
        printf("Success: 参数已更新。\r\n");
    }
}

/* ========================================================================= */
/* =                         4. 电机指令业务处理                           = */
/* ========================================================================= */

/**
 * @brief 处理 motor get 指令
 * 格式: motor get <ID> <StateName> 或 motor get <ID> <angle/omega> <PidParam>
 */
inline void handle_motor_get(const std::string subSrcList[], uint8_t count) {
    if (count < 4) {
        printf("用法: motor get <ID(0-3)> <State/PIDType> [Param]\r\n");
        return;
    }

    uint16_t motorNumber;
    // 转换并检查 ID 是否合法
    if (!toNumber(subSrcList[2], motorNumber) || motorNumber >= MOTOR_COUNT) {
        printf("错误: 电机 ID %d 不存在 (有效范围: 0-%d)\r\n", motorNumber, MOTOR_COUNT - 1);
        return;
    }

    // 获取目标电机指针
    Class_Motor_DJI_C620* targetMotor = motor_list[motorNumber];

    auto pidType = static_cast<EnumCmdPIDTypeList>(findCmd(subSrcList[3], &CmdPIDTypeList));
    if (pidType != EnumCmdPIDTypeList::error) {
        if (count < 5)
            return;
        // 根据 ID 动态选择该电机的 PID 环路
        Class_PID& targetPid = (pidType == EnumCmdPIDTypeList::angle) ? targetMotor->PID_Angle : targetMotor->PID_Omega;
        handle_pid_param_op(targetPid, "get", subSrcList[4], 0);
    } else {
        auto state = static_cast<EnumCmdMotorStateList>(findCmd(subSrcList[3], &CmdMotorStateList));
        switch (state) {
            case EnumCmdMotorStateList::target_angle:
                printf("电机%d 目标角度: %f\r\n", motorNumber, targetMotor->Get_Target_Angle());
                break;
            case EnumCmdMotorStateList::target_omega:
                printf("电机%d 目标速度: %f\r\n", motorNumber, targetMotor->Get_Target_Omega());
                break;
            case EnumCmdMotorStateList::target_current:
                printf("电机%d 目标电流: %f\r\n", motorNumber, targetMotor->Get_Target_Current());
                break;
            case EnumCmdMotorStateList::current_angle:
                printf("电机%d 当前角度: %f\r\n", motorNumber, targetMotor->Get_Now_Angle());
                break;
            case EnumCmdMotorStateList::current_omega:
                printf("电机%d 当前速度: %f\r\n", motorNumber, targetMotor->Get_Now_Omega());
                break;
            case EnumCmdMotorStateList::current_current:
                printf("电机%d 当前电流: %f\r\n", motorNumber, targetMotor->Get_Now_Current());
                break;
            case EnumCmdMotorStateList::current_power:
                printf("电机%d 实时功率: %f W\r\n", motorNumber, targetMotor->Get_Now_Power());
                break;
            default:
                errorHandle("motor get", &CmdMotorStateList);
                break;
        }
    }
}

inline void handle_motor_put(const std::string subSrcList[], uint8_t count) {
    if (count < 5) {
        printf("用法: motor put <ID> <State/PIDType> <Val/Param> [Val]\r\n");
        return;
    }

    uint16_t motorNumber;
    if (!toNumber(subSrcList[2], motorNumber) || motorNumber >= MOTOR_COUNT) {
        printf("错误: 电机 ID 无效\r\n");
        return;
    }

    Class_Motor_DJI_C620* targetMotor = motor_list[motorNumber];

    auto pidType = static_cast<EnumCmdPIDTypeList>(findCmd(subSrcList[3], &CmdPIDTypeList));
    if (pidType != EnumCmdPIDTypeList::error) {
        if (count < 6)
            return;
        float val;
        if (!toNumber(subSrcList[5], val))
            return;
        Class_PID& targetPid = (pidType == EnumCmdPIDTypeList::angle) ? targetMotor->PID_Angle : targetMotor->PID_Omega;
        handle_pid_param_op(targetPid, "put", subSrcList[4], val);
    } else {
        float val;
        if (!toNumber(subSrcList[4], val))
            return;
        auto state = static_cast<EnumCmdMotorStateList>(findCmd(subSrcList[3], &CmdMotorStateList));
        switch (state) {
            case EnumCmdMotorStateList::target_angle:
                targetMotor->Set_Target_Angle(val);
                break;
            case EnumCmdMotorStateList::target_omega:
                targetMotor->Set_Target_Omega(val);
                break;
            default:
                errorHandle("motor put", &CmdMotorStateList);
                return;
        }
        printf("OK. 电机%d 设置成功\r\n", motorNumber);
    }
}

/** @brief 电机指令二级分发 */
inline void handle_motor_cmd(const std::string subSrcList[], uint8_t count) {
    if (count < 2) {
        errorHandle("motor", &CmdMotoList);
        return;
    }

    auto subCmd = static_cast<EnumCmdMotorList>(findCmd(subSrcList[1], &CmdMotoList));
    if (subCmd == EnumCmdMotorList::get)
        handle_motor_get(subSrcList, count);
    else if (subCmd == EnumCmdMotorList::put)
        handle_motor_put(subSrcList, count);
    else
        errorHandle("motor", &CmdMotoList);
}

/* ========================================================================= */
/* =                         5. 底盘指令业务处理                           = */
/* ========================================================================= */

/** @brief 处理 chassis get 指令 */
inline void handle_chassis_get(const std::string subSrcList[], uint8_t count) {
    if (count < 3) {
        errorHandle("chassis get", &CmdChassisStateList);
        return;
    }

    // 获取枚举索引
    auto stateIdx = static_cast<EnumCmdChassisStateList>(findCmd(subSrcList[2], &CmdChassisStateList));

    switch (stateIdx) {
        case EnumCmdChassisStateList::now_angle:
            printf("当前角度: %.4f rad (%.2f°)\r\n",
                   chassis.m_worldPosition.getAngle(),
                   chassis.m_worldPosition.getAngle() * 57.3f);
            break;
        case EnumCmdChassisStateList::target_v:
            printf("目标速度: VX:%.2f, VY:%.2f, VW:%.2f\r\n",
                   chassis.Get_Target_VX(),
                   chassis.Get_Target_VY(),
                   chassis.Get_Target_VW());
            break;
        case EnumCmdChassisStateList::world_frame:
            printf("坐标系模式: %s\r\n", chassis.Get_World_Frame_Status() ? "World" : "Relative");
            break;
        case EnumCmdChassisStateList::delay_comp:
            printf("相位补偿: %.2f ms\r\n", chassis.Get_Delay_Comp_Ms());
            break;
        case EnumCmdChassisStateList::deadzone:
            printf("控制死区: %.4f\r\n", chassis.Get_Deadzone());
            break;
        case EnumCmdChassisStateList::correction:
            printf("打滑修正量 -> X: %.4f m, Y: %.4f m\r\n",
                   chassis.m_worldPosition.getXCorrection(),
                   chassis.m_worldPosition.getYCorrection());
            break;
        case EnumCmdChassisStateList::raw_ticks: {
            int32_t ticks[4];
            chassis.m_worldPosition.getRawTicks(ticks);
            printf("原始脉冲 -> XP:%d, XM:%d, YP:%d, YM:%d\r\n", ticks[0], ticks[1], ticks[2], ticks[3]);
            break;
        }
        case EnumCmdChassisStateList::IMU_Paw:{
            printf("陀螺仪角度：%.2f\r\n",chassis.Get_IMU_Paw());
            break;
        }
        // 默认处理：如果指令不在枚举列表中，统一报错并打印可用列表
        default:
            errorHandle("chassis get", &CmdChassisStateList);
            break;
    }
}

/** @brief 处理 chassis put 指令 */
inline void handle_chassis_put(const std::string subSrcList[], uint8_t count) {
    if (count < 4) {
        printf("用法: chassis put <参数名> <数值> ...\r\n");
        errorHandle("chassis put", &CmdChassisStateList);
        return;
    }

    auto stateIdx = static_cast<EnumCmdChassisStateList>(findCmd(subSrcList[2], &CmdChassisStateList));

    switch (stateIdx) {
        case EnumCmdChassisStateList::target_v: {
            float vx, vy, vw;
            if (count >= 6 && toNumber(subSrcList[3], vx) && toNumber(subSrcList[4], vy) &&
                toNumber(subSrcList[5], vw)) {
                chassis.Set_Target_VX(vx);
                chassis.Set_Target_VY(vy);
                chassis.Set_Target_VW(vw);
                printf("OK. 目标速度已更新\r\n");
            } else {
                printf("错误: 目标速度格式为 <VX> <VY> <VW>\r\n");
            }
            break;
        }
        case EnumCmdChassisStateList::delay_comp: {
            float ms;
            if (toNumber(subSrcList[3], ms)) {
                chassis.Set_Delay_Comp_Ms(ms);
                printf("OK. 相位补偿已更新: %.2f ms\r\n", ms);
            }
            break;
        }
        case EnumCmdChassisStateList::world_frame: {
            float val;
            if (toNumber(subSrcList[3], val)) {
                chassis.Set_World_Frame_Status(val > 0.5f);
                printf("OK. 坐标系已切换为: %s\r\n", (val > 0.5f) ? "World" : "Relative");
            }
            break;
        }
        case EnumCmdChassisStateList::deadzone: {
            float dz;
            if (toNumber(subSrcList[3], dz)) {
                chassis.Set_Deadzone(dz);
                printf("OK. 底盘死区已更新: %.4f\r\n", dz);
            }
            break;
        }
        case EnumCmdChassisStateList::correction:
        case EnumCmdChassisStateList::raw_ticks:
        case EnumCmdChassisStateList::now_angle:
        case EnumCmdChassisStateList::IMU_Paw:
            printf("错误: [%s] 为只读项，不可修改。\r\n", subSrcList[2].c_str());
            break;
        // 默认处理：未知参数项
        default:
            errorHandle("chassis put", &CmdChassisStateList);
            break;
    }
}

/** @brief 底盘指令二级分发 */
inline void handle_chassis_cmd(const std::string subSrcList[], uint8_t count) {
    if (count < 2) {
        errorHandle("chassis", &CmdChassisList);
        return;
    }

    auto subCmd = static_cast<EnumCmdChassisList>(findCmd(subSrcList[1], &CmdChassisList));
    if (subCmd == EnumCmdChassisList::get)
        handle_chassis_get(subSrcList, count);
    else if (subCmd == EnumCmdChassisList::put)
        handle_chassis_put(subSrcList, count);
    else
        errorHandle("chassis", &CmdChassisList);
}

inline void handle_remote_get(const std::string subSrcList[], uint8_t count) {
    if (count < 3) {
        errorHandle("remote get", &CmdRemoteStateList);
        return;
    }

    // 使用 findCmd 查找要获取的具体项
    auto stateIdx = static_cast<EnumCmdRemoteStateList>(findCmd(subSrcList[2], &CmdRemoteStateList));

    switch (stateIdx) {
        case EnumCmdRemoteStateList::stick:
            if (dr16.Get_Status() != DR16_Status_ENABLE) {
                printf("Remote Offline!\r\n");
                break;
            }
            printf("Stick_LX: %.3f, Stick_LY: %.3f | Stick_RX: %.3f, Stick_RY: %.3f, Yaw: %.3f\r\n",
                   dr16.Get_Left_X(),
                   dr16.Get_Left_Y(),
                   dr16.Get_Right_X(),
                   dr16.Get_Right_Y(),
                   dr16.Get_Yaw());
            break;

        default:
            errorHandle("remote get", &CmdRemoteStateList);
            break;
    }
}

/**
 * @brief 遥控器指令顶级分发
 */
inline void handle_remote_cmd(const std::string subSrcList[], uint8_t count) {
    if (count < 2) {
        errorHandle("remote", &CmdRemoteList);
        return;
    }

    // 使用 findCmd 查找动作 (get/put/...)
    auto actionIdx = static_cast<EnumCmdRemoteList>(findCmd(subSrcList[1], &CmdRemoteList));

    switch (actionIdx) {
        case EnumCmdRemoteList::get:
            handle_remote_get(subSrcList, count);
            break;

        default:
            errorHandle("remote", &CmdRemoteList);
            break;
    }
}

/* ========================================================================= */
/* =                         6. 主入口解析函数                             = */
/* ========================================================================= */

/**
 * @brief 调试解析总入口
 * @param src 串口接收到的原始字符串行
 */
void anysisCmd(const std::string src) {
    std::string subSrcList[8];
    uint8_t count = 0;

    // 1. 切分指令
    splitCmd(src, subSrcList, 8, count);
    if (count == 0)
        return;

    // 2. 顶级指令分发
    auto topCmd = static_cast<EnumCmdList>(findCmd(subSrcList[0], &CmdTopList));

    switch (topCmd) {
        case EnumCmdList::motor:
            handle_motor_cmd(subSrcList, count);
            break;
        case EnumCmdList::chassis:
            handle_chassis_cmd(subSrcList, count);
            break;
        case EnumCmdList::remote:
            handle_remote_cmd(subSrcList, count);
            break;  // 统一的分发风格
        case EnumCmdList::help:
            Cmd_help();
            break;
        default:
            errorHandle(nullptr, nullptr);
            break;
    }
}