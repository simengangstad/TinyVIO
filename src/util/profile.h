#ifndef PROFILE_H_
#define PROFILE_H_

#include <stdint.h>

#ifndef CPU_MIMXRT1176DVMAA_cm4

namespace profile {
    /**
     * @brief Starts a (nested) run-time profile section.
     *
     * @param identifier [in] The identifier of the section.
     */
    void start(const char* identifier);

    /**
     * @brief Ends the (nested) run-time profile section.
     *
     * @return The time between the call to #start and #end.
     */
    uint32_t end();

    /**
     * @brief Reports all the sections which have been profiled.
     */
    void report();
}

#endif

#if defined(CPU_MIMXRT1176DVMAA)

#include "board.h"
#include "logger.h"

#define HIGH_PRECISION_PROFILE_START()                                  \
    {                                                                   \
        logger::infof("%s:%d - Profile start\r\n", __FILE__, __LINE__); \
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;                 \
        DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;                           \
        DWT->CYCCNT = 0UL;                                              \
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                            \
    }

#define HIGH_PRECISION_PROFILE_END()                                               \
    {                                                                              \
        DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;                                      \
        uint32_t cycles = DWT->CYCCNT;                                             \
        float ms        = 1000.0f * (float)((float)cycles /                        \
                                     (float)BOARD_BOOTCLOCKRUN_CORE_CLOCK); \
                                                                                   \
        logger::infof("%s:%d - Profile end, took %f ms\r\n",                       \
                      __FILE__,                                                    \
                      __LINE__,                                                    \
                      ms);                                                         \
    }

#endif

#endif
