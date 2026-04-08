#ifndef DEBUG_CMD_H
#define DEBUG_CMD_H

#include "stm32f407xx.h"
#include <string>

enum class EnumCmdList : uint8_t{
    error = 0,
    motor = 1,
    help,
    
};

enum class EnumCmdMotorList : uint8_t{
    error = 0,
    get,
    put,
    
};

enum class EnumCmdMotorStateList : uint8_t{
    error = 0,
    target_angle,
    target_omega,
    target_current,
    current_angle,
    current_omega,
    current_current,
    current_power,
    
};

// PID 环类型
enum class EnumCmdPIDTypeList : uint8_t {
    error = 0,
    angle,
    omega
};

// PID 具体参数项
enum class EnumCmdPIDParamList : uint8_t {
    error = 0,
    p, i, d, f,
    i_limit,    // 对应 I_Out_Max
    out_limit,  // 对应 Out_Max
    dead_zone,
    speed_a,
    speed_b,
    separate    // 对应 I_Separate_Threshold
};

class CmdClass{
public:
    const char ** cmdStr;
    uint8_t cmdStrLength;
};

static const char* topStrings[] = {"motor", "help"};
static const char* motorStrings[] = {"get", "put"};
static const char* motorStateStrings[] = { "target_angle",   
                                            "target_omega",   
                                            "target_current", 
                                            "current_angle",  
                                            "current_omega",  
                                            "current_current",
                                            "current_power"};
static const char* PIDTypeStrings[] = {"angle", "omega"};
static const char* PIDParamStrings[] = {"p", "i", "d", "f", "i_limit", "out_limit", "dead_zone", "speed_a", "speed_b", "separate"};


static const CmdClass CmdTopList = {topStrings, 1};
static const CmdClass CmdMotoList = {motorStrings, 2};
static const CmdClass CmdMotorStateList = {motorStateStrings, 7};
static const CmdClass CmdPIDTypeList = {PIDTypeStrings, 2};
static const CmdClass CmdPIDParamList = {PIDParamStrings, 10};

void anysisCmd(const std::string src);
#endif
