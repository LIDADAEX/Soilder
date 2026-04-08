#ifndef DEBUG_FIFO_H
#define DEBUG_FIFO_H

#include <string>

#define DEBUG_FIFO_SIZE 256

class Class_Debug_FIFO {
private:
    uint8_t buffer[DEBUG_FIFO_SIZE];
    uint16_t head = 0;
    uint16_t tail = 0;

public:
    // 写入单个字节
    void Push(uint8_t data) {
        uint16_t next = (head + 1) % DEBUG_FIFO_SIZE;
        if (next != tail) {
            buffer[head] = data;
            head = next;
        }
    }

    // 尝试读取一行指令
    bool GetLine(std::string& line);
};

extern Class_Debug_FIFO Debug_FIFO;

#endif