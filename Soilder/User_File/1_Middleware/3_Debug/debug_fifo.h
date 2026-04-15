#ifndef DEBUG_FIFO_H
#define DEBUG_FIFO_H

#include <cstdint>
#include <string>

#define DEBUG_FIFO_SIZE 256

/**
 * @brief 调试串口 FIFO 缓冲区类
 * 用于异步接收串口数据，并从中解析出以特定分隔符开始、换行符结束的指令
 */
class Class_Debug_FIFO {
   private:
    uint8_t buffer[DEBUG_FIFO_SIZE];
    uint16_t head = 0;  // 写指针
    uint16_t tail = 0;  // 读指针

   public:
    /**
     * @brief 写入单个字节到 FIFO
     * @param data 接收到的原始字节
     */
    void Push(uint8_t data) {
        uint16_t next = (head + 1) % DEBUG_FIFO_SIZE;
        // 如果缓冲区未满，则存入数据并移动 head
        if (next != tail) {
            buffer[head] = data;
            head = next;
        }
    }

    /**
     * @brief 从 FIFO 中提取一行指令
     * @param line 用于存储提取出的字符串结果
     * @return bool 是否成功提取到有效指令
     */
    bool GetLine(std::string& line);
};

// 全局单例声明
extern Class_Debug_FIFO Debug_FIFO;

#endif