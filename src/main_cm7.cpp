#include "board.h"
#include "clock_controller.h"
#include "delay.h"
#include "ethernet.h"
#include "logger.h"
#include "memory.h"
#include "multicore.h"

#include "pipeline.h"

/**
 * @brief Set when the secondary core has sent a message to this core.
 */
volatile bool has_received = false;

/**
 * @brief The message received from the secondary core.
 */
volatile multicore::Message received_message = {};

/**
 * @brief Called when the secondary core sends a message to this core.
 */
void message_callback(const multicore::Message msg) {

    memcpy((void*)&received_message, (void*)&msg, sizeof(multicore::Message));
    has_received = true;
}

#ifdef DEBUG

#include <etl/error_handler.h>

struct error_log {
    void member_function(const etl::exception& e) {
        logger::errorf("The error was %s in %s at %d \r\n",
                       e.what(),
                       e.file_name(),
                       e.line_number());
    }
};

#endif

int main(void) {

    memory::configure_access_policy();

    clock_controller::initialise();

    logger::initialise();
    logger::set_level(logger::Level::LOG_INFO);
    logger::set_prefix("(CORE0) ");

#ifdef DEBUG
    logger::rawf("\r\n");
    logger::infof("=== Starting up (Build type: DEBUG) ===\r\n");
#else
    logger::rawf("\r\n");
    logger::infof("=== Starting up (Build type: RELEASE) ===\r\n");
#endif

    logger::infof("Core clock: %u MHz\r\n", CLOCK_GetM7Freq() / (uint32_t)1e6);

#ifdef DEBUG

    static error_log log;

    etl::error_handler::set_callback<error_log, &error_log::member_function>(
        log);
#endif

    multicore::initialise();
    multicore::register_message_callback(message_callback);

    multicore::Message message;
    multicore::send_message(message);

    // Wait for the message from the secondary core that it has started up
    while (!has_received) { __asm volatile("nop"); }

    logger::infof("Received message from second core: ");

    for (int i = 0; i < received_message.data[0]; i++) {
        logger::rawf("%c", (char)received_message.data[i + 1]);
    }

    logger::rawf("\r\n");

    vio::run_pipeline_with_proxy_link();

    while (1) {}

    return 0;
}
