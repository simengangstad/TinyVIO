#ifndef LOGGER_H
#define LOGGER_H

namespace logger {

    enum class Level { LOG_INFO, LOG_DEBUG };

    /**
     * Initialises the LPUART interface for the log module.
     */
    void initialise();

    /**
     * @brief Sets the @p log_level for the logging module.
     */
    void set_level(const Level log_level);

    /**
     * @return The log level.
     */
    Level get_level(void);

    /**
     * @brief Sets a prefix for each log. For example to differentiate between
     * CORE0 and CORE1.
     */
    void set_prefix(const char* prefix);

    /**
     * @brief Logs an info message.
     */
    void infof(const char* format, ...);

    /**
     * @brief Logs a debug message.
     */
    void debugf(const char* format, ...);

    /**
     * @brief Logs a warning message.
     */
    void warnf(const char* format, ...);

    /**
     * @brief Logs an error message.
     */
    void errorf(const char* format, ...);

    /**
     * @brief Logs a raw message, without any prefix.
     */
    void rawf(const char* format, ...);

    /**
     * @brief Logs a raw message, without any prefix, if the log level is debug.
     */
    void debugrawf(const char* format, ...);
}

#endif
