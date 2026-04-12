#include "debug_cmd_interface.h"
#include "debug_fifo.h"
#include "debug_cmd.h"

extern "C" {
    /**
     * @brief 由 USB 或串口中断服务程序 (ISR) 调用的 C 接口函数
     * @details 由于硬件中断回调通常是 C 语言实现的，这里使用 extern "C" 
     * 将接收到的原始字节压入 C++ 的 Debug_FIFO 对象中。
     * @param data 硬件层接收到的单字节数据
     */
    void Debug_FIFO_Push(uint8_t data) {
        Debug_FIFO.Push(data);
    }

    /**
     * @brief 调试指令轮询回调函数
     * @details 建议在系统主循环 (Main Loop) 或 1ms 硬件定时器中断中调用此函数。
     * 它会持续检查 FIFO 缓冲区，直到处理完所有完整的指令行。
     */
    void Debug_Cmd_Poll_Callback(void) {
        std::string cmdLine;
        
        // 使用 while 循环而不是 if，确保在单次轮询中能处理完缓冲区内的多条指令
        // 这样可以避免在指令密集发送时产生处理延迟
        while (Debug_FIFO.GetLine(cmdLine)) {
            // 将提取出的指令字符串传递给解析器
            anysisCmd(cmdLine);
        }
    }
}