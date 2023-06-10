#ifndef MULTICORE_H
#define MULTICORE_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "mcmgr.h"

namespace multicore {

    struct Message {
        uint16_t data[64];
    };

    typedef void (*MessageCallback)(const Message message);

    /**
     * @brief Initialises the MCMGR module.
     *
     * @return True if initialisation was successful. For the secondary core,
     * that means that it booted successfully and received necessary startup
     * data.
     */
    bool initialise();

    /**
     * @brief Destroys the message link (if configured).
     */
    void deinitialise();

    /**
     * @brief Register a message callback function for when messsages are sent
     * between cores.
     */
    void register_message_callback(MessageCallback message_callback);

    /**
     * @brief Sends a @p message to the other core.
     */
    void send_message(const Message message);
}

#endif
