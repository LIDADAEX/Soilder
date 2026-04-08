#include "debug_cmd_interface.h"

#include "debug_fifo.h"
#include "debug_cmd.h"

extern "C" {
    // 供 USB 中断调用的 C 函数
    void Debug_FIFO_Push(uint8_t data) {
        Debug_FIFO.Push(data);
    }

    // 这就是你要求的“回调函数”，供你 1ms 轮询调用
    void Debug_Cmd_Poll_Callback(void) {
		std::string cmdLine;
		
		// 用 while 而不是 if，可以在 1ms 内处理多条排队指令
		while (Debug_FIFO.GetLine(cmdLine)) {
			anysisCmd(cmdLine);
		}
    }
}