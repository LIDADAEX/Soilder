#include "debug_log.h"

#include <iostream>

void Logger::log(LogLevel level, const std::string &message, const char *file, const int line, const char *function) {
    // 1. 快速过滤
    if (static_cast<int16_t>(level) < static_cast<int16_t>(m_consoleLevel)) {
        return;
    }

    // 2. 极其高效的文件名截取 (零拷贝)
    // 直接操作指针，不创建 std::string 对象
    const char* fileName = file;
    const char* p = file;
    while (*p) {
        if (*p == '/' || *p == '\\') {
            fileName = p + 1;
        }
        p++;
    }

    // 3. 格式化消息
    // 在栈上分配缓冲区，避免堆碎片的产生
    // 假设整条日志长度不超过 256 字节，根据 F407 的内存情况可自行调整
    char buffer[256]; 
    
    // snprintf 是线程安全的且不会溢出
    int len = snprintf(buffer, sizeof(buffer), "[%s:%d][%s][%s]", 
                       fileName, 
                       line, 
                       function, 
                       message.c_str());

    // 4. 发送到控制台
    // 如果 len >= sizeof(buffer)，说明日志被截断了，可以根据需要处理
    if (len > 0) {
        // 直接传递 buffer 指针或再次封装为 string（如果 writeToConsole 必须用 string）
        writeToConsole(level, std::string(buffer, len));
    }
}

void Logger::writeToConsole(LogLevel level, const std::string &formattedMsg) {
	const char* color_head = "";
    const char* level_tag = "";
    const char* color_tail = "\033[0m";

    // 1. 预设颜色和标签 (直接用指针，避免字符串拷贝)
    switch (level) {
        case LogLevel::Debug:    color_head = "\033[37m"; level_tag = "[DEBUG]"; break;
        case LogLevel::Info:     color_head = "";        level_tag = "[INFO]";  color_tail = ""; break;
        case LogLevel::Warning:  color_head = "\033[33m"; level_tag = "[WARNING]"; break;
        case LogLevel::Error:    color_head = "\033[31m"; level_tag = "[ERROR]"; break;
        case LogLevel::Critical: color_head = "\033[31m"; level_tag = "[CRITICAL]"; break;
        case LogLevel::Fatal:    color_head = "\033[41;37m"; level_tag = "[FATAL]"; break;
    }

    // 2. 只有一件事：把所有内容拼成一个完整的字符串
    // 这样底层 _write 只需要调用一次，直接触发你的 128 字节逻辑
	printf("%s%s%s%s\r\n", color_head, level_tag, formattedMsg.c_str(), color_tail);
}

