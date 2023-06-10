#ifndef IP_H
#define IP_H

#include <stdint.h>

namespace ip {
    /**
     * @brief IP4 address.
     */
    struct IP4Address {
        uint8_t a, b, c, d;
    };
}

#endif
