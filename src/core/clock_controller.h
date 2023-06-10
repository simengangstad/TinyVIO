#ifndef _CLOCK_CONTROLLER_H_
#define _CLOCK_CONTROLLER_H_

#include <stdint.h>

namespace clock_controller {

    /**
     * @brief Initialises the main oscillators and clocks.
     */
    void initialise();

    /**
     * @return Milliseconds since boot.
     */
    uint32_t ms();

}

#endif
