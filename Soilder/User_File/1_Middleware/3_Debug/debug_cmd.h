#ifndef DEBUG_CMD_H
#define DEBUG_CMD_H

enum class EnumCmdList{
    motor = 0,
    error
}enumCmdList;

enum class EnumCmdMotorList{
    get,
    put,
    error
};

enum class EnumCmdMotorStateList{
    target_angle,
    target_omega,
    target_current,
    current_angle,
    current_omega,
    current_current,
    error
};

class CmdClass{
public:
    const char ** cmdStr;
    uint8_t cmdStrLength;
};

static const char* topStrings[] = {"motor"};
static const char* motoStrings[] = {"get", "put"};
static const char* motorStateString[] = { "target_angle",   
                                            "target_omega",   
                                            "target_current", 
                                            "current_angle",  
                                            "current_omega",  
                                            "current_current",};



static const CmdClass CmdTopList = {topStrings, 1};
static const CmdClass CmdMotoList = {motoStrings, 2};
static const CmdClass CmdMotorStateList = {motorStateString, 6};



#endif
