#include "debug_fifo.h"

Class_Debug_FIFO Debug_FIFO;

/**
 * @brief 从 FIFO 中提取以 '.' 开头，以 \r\n, \n, 或 \r 结尾的指令
 */
bool Class_Debug_FIFO::GetLine(std::string& line) {
    if (head == tail) return false;

    uint16_t curr = tail;
    int16_t startPos = -1;
    bool foundEnd = false;
    uint16_t endPos = 0;

    // 1. 扫描缓冲区寻找起始符 '.' 和 任意结束符
    while (curr != head) {
        uint8_t c = buffer[curr];

        if (c == '.') {
            startPos = curr;
        } 
        else if (startPos != -1 && (c == '\r' || c == '\n')) {
            // 只要在起点之后发现了任一换行符，即认为帧结束
            endPos = curr;
            foundEnd = true;
            break;
        }
        curr = (curr + 1) % DEBUG_FIFO_SIZE;
    }

    // 2. 情况分析
    if (!foundEnd) {
        // 如果缓冲区满了还没找到起始符，或者找到了起始符但没找到结束符
        // 我们可以选择清理掉起始符之前的垃圾数据，保护缓冲区空间
        if (startPos != -1 && startPos != tail) {
            tail = startPos; 
        } else if (startPos == -1 && ((head + 1) % DEBUG_FIFO_SIZE == tail)) {
            tail = head; // 缓冲区满且无起始符，全丢弃
        }
        return false;
    }

    // 3. 提取有效数据 (不包含 '.' 和 结束符)
    line.clear();
    uint16_t extractPtr = (startPos + 1) % DEBUG_FIFO_SIZE;
    while (extractPtr != endPos) {
        line += (char)buffer[extractPtr];
        extractPtr = (extractPtr + 1) % DEBUG_FIFO_SIZE;
    }

    // 4. 更新 tail，跳过当前帧及其结束符
    tail = (endPos + 1) % DEBUG_FIFO_SIZE;

    // 5. 【关键】处理多字节结束符（如 \r\n）
    // 如果下一个字符也是换行符，把它也跳过，防止产生空指令
    if (tail != head && (buffer[tail] == '\r' || buffer[tail] == '\n')) {
        tail = (tail + 1) % DEBUG_FIFO_SIZE;
    }

    return !line.empty();
}