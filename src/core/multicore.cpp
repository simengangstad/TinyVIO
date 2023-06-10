#include "multicore.h"

#include "board.h"
#include "clock_controller.h"
#include "logger.h"

#include "rpmsg_lite.h"
#include "rpmsg_platform.h"

#include <string.h>

/**
 * @brief Time to wait whilst CORE1 is checking if it has booted successfully.
 */
#define CORE1_BOOT_TIMEOUT (2000U)

#ifdef MULTICORE_MASTER

#define CORE1_BOOT_ADDRESS ((uint32_t*)0x20200000)

/**
 * @brief RPMSG buffer size.
 */
#define SH_MEM_TOTAL_SIZE (0x2000)

/**
 * @brief Defined by the include binary assembly file (inc_core1_bin.S). The
 * binary for CORE1 is loaded within CORE0's binary. This specifies the buffer
 * containing CORE1's binary. Such that it can be loaded into CORE1's ITCM
 * before boot of CORE1.
 */
extern const char core1_image_start[];

/**
 * @brief Size of CORE1's binary.
 */
extern uint32_t core1_image_size;

#endif

/**
 * @brief Called by SystemInit during the later parts of the ResetISR.
 */
void SystemInitHook(void) { MCMGR_EarlyInit(); }

namespace multicore {

    /**
     * @brief Used for storing event data when a MCMGR event is triggered.
     */
    volatile uint16_t application_event_data;

    struct rpmsg_lite_instance* rpmsg;
    struct rpmsg_lite_instance rpmsg_context;

    struct rpmsg_lite_endpoint* endpoint;
    struct rpmsg_lite_ept_static_context endpoint_context;

    /**
     * @brief RPMSG message callback.
     */
    MessageCallback message_callback = NULL;

#if MULTICORE_MASTER

    /**
     * @brief Memory for RPMSG.
     */
    AT_RPMSG_SECTION_NOINIT(static char rpmsg_base[SH_MEM_TOTAL_SIZE]);

    /**
     * @brief Called when a core triggers an MCMGR event.
     */
    static void application_event_callback(uint16_t event_data, void* context) {
        *((uint16_t*)context) = event_data;
    }

#elif MULTICORE_SLAVE

    /**
     * @brief The RPMSG base address passed to the secondary core during
     * startup. Is the address of #rpmsg_base
     */
    static uint32_t rpmsg_base_address;

#endif

    /**
     * @brief Called when a RPMSG is received.
     */
    [[maybe_unused]] static int32_t
    endpoint_callback(void* payload,
                      uint32_t payload_length,
                      __attribute__((unused)) uint32_t source,
                      __attribute__((unused)) void* context) {

        if (payload_length <= sizeof(Message)) {
            if (message_callback != NULL) {
                Message message;
                memcpy((void*)&message, payload, payload_length);
                message_callback(message);
            }
        }

        return RL_RELEASE;
    }

    bool initialise() {
        MCMGR_Init();

#if MULTICORE_MASTER

        MCMGR_RegisterEvent(kMCMGR_RemoteApplicationEvent,
                            application_event_callback,
                            ((void*)&application_event_data));

        logger::debugf(
            "Copying secondary core image to address: 0x%x, size: %d\r\n",
            (void*)(char*)CORE1_BOOT_ADDRESS,
            (void*)core1_image_size);

        memcpy((void*)(char*)CORE1_BOOT_ADDRESS,
               (void*)core1_image_start,
               core1_image_size);

        logger::debugf("Booting secondary core...\r\n");

        // Here we send the address of the RPMSG base so that the slave core can
        // use that for setting up the message link
        MCMGR_StartCore(kMCMGR_Core1,
                        (void*)(char*)CORE1_BOOT_ADDRESS,
                        (uint32_t)rpmsg_base,
                        kMCMGR_Start_Synchronous);

        // Wait for the secondary core to have notified that it has set up its
        // RPMSG remote link.
        while (application_event_data != RPMSG_LINK_READY) {}

        rpmsg = rpmsg_lite_master_init(rpmsg_base,
                                       SH_MEM_TOTAL_SIZE,
                                       RPMSG_LITE_LINK_ID,
                                       RL_NO_FLAGS,
                                       &rpmsg_context);

        endpoint = rpmsg_lite_create_ept(rpmsg,
                                         RPMSG_MASTER_ENDPOINT_ADDRESS,
                                         endpoint_callback,
                                         NULL,
                                         &endpoint_context);

        // And then wait until the endpoint is created on the remote side
        while (application_event_data != RPMSG_ENDPOINT_READY) {}

        return true;

#elif MULTICORE_SLAVE

        // Retrieve the startup data, which is the RPMSG base address passed
        // from the master core to the slave core
        mcmgr_status_t status;
        uint32_t start = clock_controller::ms();

        do {
            status = MCMGR_GetStartupData(&rpmsg_base_address);
        } while (status != kStatus_MCMGR_Success &&
                 (clock_controller::ms() - start < CORE1_BOOT_TIMEOUT));

        if (clock_controller::ms() - start > CORE1_BOOT_TIMEOUT) {
            return false;
        }

        logger::debugf("Secondary core booted, rpmsg_base_address: %p\r\n",
                       rpmsg_base_address);

        rpmsg = rpmsg_lite_remote_init((void*)(char*)rpmsg_base_address,
                                       RPMSG_LITE_LINK_ID,
                                       RL_NO_FLAGS,
                                       &rpmsg_context);

        // Trigger that the slave core's RPMSG link is ready
        MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, RPMSG_LINK_READY);

        // Wait for 1000 ms for uplink
        if (!rpmsg_lite_wait_for_link_up(rpmsg, 1000)) {
            return false;
        }

        endpoint = rpmsg_lite_create_ept(rpmsg,
                                         RPMSG_SLAVE_ENDPOINT_ADDRESS,
                                         endpoint_callback,
                                         NULL,
                                         &endpoint_context);

        // Notify that the endpoint is ready
        MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, RPMSG_ENDPOINT_READY);

        return true;

#endif

        return false;
    }

    void deinitialise() {
        if (rpmsg != NULL && endpoint != NULL) {
            rpmsg_lite_destroy_ept(rpmsg, endpoint);
            rpmsg_lite_deinit(rpmsg);
        }
    }

    void register_message_callback(MessageCallback callback) {
        message_callback = callback;
    }

    void send_message(const Message message) {
#ifdef MULTICORE_MASTER

        rpmsg_lite_send(rpmsg,
                        endpoint,
                        RPMSG_SLAVE_ENDPOINT_ADDRESS,
                        (char*)&message,
                        sizeof(Message),
                        RL_DONT_BLOCK);

#elif MULTICORE_SLAVE

        rpmsg_lite_send(rpmsg,
                        endpoint,
                        RPMSG_MASTER_ENDPOINT_ADDRESS,
                        (char*)&message,
                        sizeof(Message),
                        RL_DONT_BLOCK);
#endif
    }
}
