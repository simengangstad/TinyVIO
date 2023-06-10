#include "logger.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "fsl_common.h"
#include "fsl_lpuart.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "fsl_iomuxc.h"
#pragma GCC diagnostic pop

#define INFO_LEVEL_FMT  "[ INFO  ] "
#define DEBUG_LEVEL_FMT "[ DEBUG ] "
#define WARN_LEVEL_FMT  "[ WARN  ] "
#define ERROR_LEVEL_FMT "[ ERROR ] "

namespace logger {

    /**
     * @brief The current log level.
     */
    static Level log_level = Level::LOG_INFO;

    /**
     * @brief Contains the message to be printed, excluding any log level and
     * custom prefix.
     */
    static char message_buffer[256] = "";

    /**
     * @brief Length of custom prefix.
     */
    static size_t prefix_buffer_length = 0;

    /**
     * @brief Buffer for the prefix which will be printed with every message
     * except raw printing.
     */
    static char prefix_buffer[16] = "";

    /**
     * @brief Whether the logger module has been initialized. Will not print if
     * this flag is false.
     */
    static bool initialised = false;

    /**
     * @brief Convenience function for printing. Gets called by every log
     * message except raw printing.
     *
     * @param log_level_prefix The log level prefix.
     * @param format The format message.
     * @param args The arguments the @p format will be formatted with.
     */
    static void
    print(const char* log_level_prefix, const char* format, va_list args) {

        int formatted_bytes =
            vsnprintf(message_buffer, sizeof(message_buffer), format, args);

        // -1 here as the returned number of formatted bytes don't count the
        // null termination
        if (formatted_bytes > (int)sizeof(message_buffer)) {
            warnf("String too long for format buffer. Consider decreasing the "
                  "string length.\r\n");
            return;
        }

        LPUART_WriteBlocking(LPUART1,
                             (uint8_t*)log_level_prefix,
                             strlen(log_level_prefix));
        LPUART_WriteBlocking(LPUART1,
                             (uint8_t*)prefix_buffer,
                             prefix_buffer_length);
        LPUART_WriteBlocking(LPUART1,
                             (uint8_t*)message_buffer,
                             strlen(message_buffer));
    }

    void initialise() {
        CLOCK_EnableClock(kCLOCK_Iomuxc);

        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_24_LPUART1_TXD, 0U);
        IOMUXC_SetPinMux(IOMUXC_GPIO_AD_25_LPUART1_RXD, 0U);

        // Slew Rate Field: Slow Slew Rate
        // Drive Strength Field: high drive strength
        // Pull / Keep Select Field: Pull Disable, high impedance
        // Pull Up / Down Config. Field: Weak pull down
        // Open Drain Field: Disabled
        // Domain write protection: Both cores are allowed
        // Domain write protection lock: Neither of DWP bits is locked
        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_24_LPUART1_TXD, 0x02U);

        // Slew Rate Field: Slow Slew Rate
        // Drive Strength Field: high drive strength
        // Pull / Keep Select Field: Pull Disable, high impedance
        // Pull Up / Down Config. Field: Weak pull down
        // Open Drain Field: Disabled
        // Domain write protection: Both cores are allowed
        // Domain write protection lock: Neither of DWP bits is locked
        IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_25_LPUART1_RXD, 0x02U);

        lpuart_config_t lpuart_config = {};
        lpuart_config.baudRate_Bps    = 115200U;
        lpuart_config.enableTx        = true;

        LPUART_Init(LPUART1,
                    &lpuart_config,
                    CLOCK_GetRootClockFreq(kCLOCK_Root_Lpuart1));

        initialised = true;
    }

    void set_level(const Level level) { log_level = level; }

    Level get_level() { return log_level; }

    void set_prefix(const char* prefix) {

        const size_t prefix_buffer_character_length = sizeof(prefix_buffer) - 1;
        const size_t prefix_character_length        = strlen(prefix);

        if (prefix_character_length > prefix_buffer_character_length) {
            logger::errorf("Prefix cannot be greater than %d\r\n",
                           prefix_buffer_character_length);
            return;
        }

        strncpy(prefix_buffer, prefix, prefix_buffer_character_length);

        prefix_buffer_length = prefix_character_length;
    }

    void infof(const char* format, ...) {
        if (!initialised) {
            return;
        }

        va_list args;
        va_start(args, format);
        print(INFO_LEVEL_FMT, format, args);
        va_end(args);
    }

    void debugf(const char* format, ...) {
        if (!initialised) {
            return;
        }

        if (log_level == Level::LOG_INFO) {
            return;
        }

        va_list args;
        va_start(args, format);
        print(DEBUG_LEVEL_FMT, format, args);
        va_end(args);
    }

    void warnf(const char* format, ...) {
        if (!initialised) {
            return;
        }

        va_list args;
        va_start(args, format);
        print(WARN_LEVEL_FMT, format, args);
        va_end(args);
    }

    void errorf(const char* format, ...) {
        if (!initialised) {
            return;
        }

        va_list args;
        va_start(args, format);
        print(ERROR_LEVEL_FMT, format, args);
        va_end(args);
    }

    void rawf(const char* format, ...) {
        if (!initialised) {
            return;
        }

        va_list args;
        va_start(args, format);
        vsnprintf(message_buffer, sizeof(message_buffer), format, args);
        va_end(args);

        LPUART_WriteBlocking(LPUART1,
                             (uint8_t*)message_buffer,
                             strlen(message_buffer));
    }

    void debugrawf(const char* format, ...) {
        if (!initialised) {
            return;
        }

        if (log_level == Level::LOG_INFO) {
            return;
        }

        va_list args;
        va_start(args, format);
        vsnprintf(message_buffer, sizeof(message_buffer), format, args);
        va_end(args);

        LPUART_WriteBlocking(LPUART1,
                             (uint8_t*)message_buffer,
                             strlen(message_buffer));
    }
}
