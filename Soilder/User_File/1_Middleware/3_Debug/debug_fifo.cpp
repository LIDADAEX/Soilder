#include "debug_fifo.h"

Class_Debug_FIFO Debug_FIFO;

/**
 * @brief 解析 FIFO 缓冲区，查找以 '/' 开始并以 \r 或 \n 结束的指令
 * * @param line 引用传参，返回解析到的指令内容（不含开始符和结束符）
 * @return true 找到完整指令 | false 未找到完整指令
 */
bool Class_Debug_FIFO::GetLine(std::string& line) {
    // 缓冲区为空，直接返回
    if (head == tail)
        return false;

    uint16_t curr = tail;
    int16_t startPos = -1;  // 用于标记 '/' 的位置
    bool foundEnd = false;
    uint16_t endPos = 0;  // 用于标记换行符的位置

    // 1. 遍历当前缓冲区，寻找起始符 '/' 和结束换行符
    while (curr != head) {
        uint8_t c = buffer[curr];

        if (c == '/') {
            startPos = curr;  // 记录指令起始点
        } else if (startPos != -1 && (c == '\r' || c == '\n')) {
            // 如果已经找到起始符，且当前字符是换行符，说明捕获到了完整行
            endPos = curr;
            foundEnd = true;
            break;
        }
        curr = (curr + 1) % DEBUG_FIFO_SIZE;
    }

    // 2. 边界处理：如果没有找到完整的结束符
    if (!foundEnd) {
        // 如果找到了起始符但没有结束符，且起始符之前有垃圾数据
        // 将 tail 移到起始符位置，丢弃之前的无用数据
        if (startPos != -1 && startPos != tail) {
            tail = startPos;
        }
        // 如果连起始符都没有，且缓冲区快满了，则清空缓冲区防止溢出
        else if (startPos == -1 && ((head + 1) % DEBUG_FIFO_SIZE == tail)) {
            tail = head;
        }
        return false;
    }

    // 3. 提取指令内容 (提取 '/' 之后到 换行符 之前的内容)
    line.clear();
    uint16_t extractPtr = (startPos + 1) % DEBUG_FIFO_SIZE;
    while (extractPtr != endPos) {
        line += (char)buffer[extractPtr];
        extractPtr = (extractPtr + 1) % DEBUG_FIFO_SIZE;
    }

    // 4. 更新 tail 指针，跳过已处理的指令和结束符
    tail = (endPos + 1) % DEBUG_FIFO_SIZE;

    // 5. 自动兼容 \r\n 换行格式
    // 如果后面紧跟另一个换行符（比如 \r 后面跟着 \n），将其一并跳过
    if (tail != head && (buffer[tail] == '\r' || buffer[tail] == '\n')) {
        tail = (tail + 1) % DEBUG_FIFO_SIZE;
    }

    return !line.empty();
}