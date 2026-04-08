#include "debug_cmd.h"
#include <string>

inline const std::string* const spiltCmd(const std::string src){
    std::string subSrcList[5];
    char tempStr[50];
    uint8_t count = 0; // 存子串的数量
    uint8_t j = 0;     // tempStr 的写入位置

    for (auto i = src.cbegin(); i != src.cend(); ++i) {
        // 如果不是空格，就往临时缓冲区塞
        if (*i != ' ') {
            if (j < 49) { // 防止溢出
                tempStr[j++] = *i;
            }
        } 
        // 如果遇到空格或者到了字符串末尾，说明一个词结束了
        if (*i == ' ' || (i + 1) == src.cend()) {
            if (j > 0) { // 确保不是空词（比如连续空格）
                tempStr[j] = '\0';
                if (count < 5) {
                    subSrcList[count++] = tempStr;
                }
                j = 0; // 重置临时缓冲区指针
            }
        }
    }

    return subSrcList;
}

EnumCmdList anysisTopCmd(const std::string subSrc){
    uint8_t cnd = 0;
    auto& str = CmdTopList.cmdStr;
    while(cnd < CmdTopList.cmdStrLength){
        if(subSrc.compare(str[cnd])){
            return static_cast<EnumCmdList>(cnd);
        }
    }
    return EnumCmdList::error;
}

void anysisCmd(const std::string src){
    const std::string* const subSrcList = spiltCmd(src);
    switch(anysisTopCmd(subSrcList[0])){
        case(CmdList::motor):{
            
        }
    }
}