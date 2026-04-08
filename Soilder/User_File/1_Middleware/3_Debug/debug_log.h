#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <string>

/**
 * @brief 日志等级
 *
 */
enum class LogLevel : uint16_t { Debug, Info, Warning, Error, Critical, Fatal };

/**
 * @brief 日志类，单例，使用需要初始化，初始化流程:
 * auto& logger = Logger::instance();
 * logger.initialize();
 */
class Logger {
public:
    static Logger& instance();

    /**
     * @brief 初始化。
     *
     * @param consoleLevel 控制台最低打印等级
     * @param fileLevel 文件最低打印等级
     */
    void initialize(LogLevel consoleLevel = LogLevel::Info);

    /**
     * @brief 日志输出主函数
     * 不推荐直接使用这个函数，请使用便利宏。
     * 如果开启了文件打印日志则会打印到日志中。
     *
     * @param level 日志等级
     * @param message 日志内容
     * @param file 文件名
     * @param line 文件行数
     * @param function 函数名
     */
    void log(LogLevel level,
             const std::string& message,
             const char* file = "",
             int line = 0,
             const char* function = "");

/*便利宏定义，对应等级，__FILE__,__LINE__,__FUNCTION__
    分别是QT的预设宏可以自动填入日志发送的文件，行数和函数*/
#define LOG_DEBUG(msg) \
    Logger::instance().log(LogLevel::Debug, msg, __FILE__, __LINE__, __FUNCTION__)
#define LOG_INFO(msg) Logger::instance().log(LogLevel::Info, msg, __FILE__, __LINE__, __FUNCTION__)
#define LOG_WARNING(msg) \
    Logger::instance().log(LogLevel::Warning, msg, __FILE__, __LINE__, __FUNCTION__)
#define LOG_ERROR(msg) \
    Logger::instance().log(LogLevel::Error, msg, __FILE__, __LINE__, __FUNCTION__)
#define LOG_CRITICAL(msg) \
    Logger::instance().log(LogLevel::Critical, msg, __FILE__, __LINE__, __FUNCTION__)
#define LOG_FATAL(msg) \
    Logger::instance().log(LogLevel::Fatal, msg, __FILE__, __LINE__, __FUNCTION__)

    /**
     * @brief 设置控制台和文件日志等级
     *
     * @param level 日志等级
     */
    void setLogLevel(LogLevel level);

private:

    LogLevel m_consoleLevel; /**< 控制台最低打印等级 */
    LogLevel m_fileLevel;    /**< 文件最低打印等级  */

    Logger() = default;
    ~Logger() = default;
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    /**
     * @brief 打印到控制台中
     *
     * @param level 日志等级
     * @param formattedMsg 格式化后的日志信息
     */
    void writeToConsole(LogLevel level, const std::string& formattedMsg);
};

inline Logger &Logger::instance() {
    static Logger instance;
    return instance;
}

inline void Logger::initialize(LogLevel consoleLevel) {
    m_consoleLevel = consoleLevel;
}

inline void Logger::setLogLevel(LogLevel level) {
    m_consoleLevel = level;
}



#endif // DEBUG_LOG_H