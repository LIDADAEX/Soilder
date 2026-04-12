#ifndef DEBUG_CMD_INTERFACE_H
#define DEBUG_CMD_INTERFACE_H

/* * 这里的包含建议根据实际使用的 HAL 库或固件库调整。
 * 如果只用到 uint8_t，包含 <stdint.h> 会比包含具体的 MCU 型号头文件更通用。
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 将接收到的字节推入调试 FIFO
 * @note  通常在串口接收中断 (UART_IRQHandler) 或 USB 接收回调中调用
 * @param data 接收到的原始 8-bit 数据
 */
void Debug_FIFO_Push(uint8_t data);

/**
 * @brief 调试命令轮询处理接口
 * @note  需在 main 循环或系统定时器任务中周期性调用，用于解析并执行 FIFO 中的指令
 */
void Debug_Cmd_Poll_Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_CMD_INTERFACE_H */