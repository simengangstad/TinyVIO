#include "delay.h"

#include "clock_controller.h"

namespace delay {
    void ms(const uint32_t duration) {
        const uint32_t start = clock_controller::ms();

        while (clock_controller::ms() - start < duration) {}
    }
}
