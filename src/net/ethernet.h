#ifndef ETHERNET_H
#define ETHERNET_H

#include <stdint.h>

#include "lwip/ip4_addr.h"

#include "ip.h"

namespace ethernet {

    /**
     * @brief Enum for the two available ethernet ports.
     */
    enum class Port { PORT_1G, PORT_100M };

    /**
     * @brief Initialises the ethernet driver.
     *
     * @param ethernet_port Which ethernet port to use.
     * @param address The local IP address.
     * @param net_mask Net mask of the configuration.
     * @param gateway_address Address to the host gateway.
     */
    bool initialise(const ethernet::Port ethernet_port,
                    const ip::IP4Address address,
                    const ip::IP4Address net_mask,
                    const ip::IP4Address gateway_address);

    /**
     * @return Whether the module is initialised.
     */
    bool is_initialised();

    /**
     * @brief Polls the underlying driver, needs to be called regularly.
     */
    void poll();

    /**
     * @param Returns a pointer to the local IP address configured with the net
     * interface.
     */
    ip4_addr_t* getAddress();

    /**
     * @param Returns a pointer to the net mask configured with the net
     * interface.
     */
    ip4_addr_t* getNetMask();

    /**
     * @param Returns a pointer to the gateway IP address configured with the
     * net interface.
     */
    ip4_addr_t* getGatewayAddress();

}

#endif
