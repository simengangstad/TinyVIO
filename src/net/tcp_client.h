#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include "ip.h"

#include <stddef.h>
#include <stdint.h>

namespace tcp {

    namespace client {

        enum class ConnectionStatus {
            CONNECTED = 0,
            REMOTE_NOT_AVAILABLE,
            REMOTE_DISCONNECT
        };

        typedef void (*connection_status_callback)(
            const ConnectionStatus connection_status);

        typedef void (*transmission_status_callback)(void);

        typedef void (*receive_callback)(void* data, const uint16_t data_size);

        /**
         * @brief Starts connecting to a TCP server at @p server_address and @p
         * server_port. When the client has connected, #is_connected() will
         * return true and, if the callback is configured, it will be called.
         *
         * @param server_address IP4 address of the server.
         * @param server_port Which port the server is hosted at.
         * @param connection_status_cb Called when the TCP client is connected
         * to the server (or possibly if the connection failed) or when a
         * disconnect happens.
         *
         * @return true if the connection configuration was successful.
         */
        bool connect(const ip::IP4Address server_address,
                     const uint16_t server_port,
                     connection_status_callback connection_status_cb = NULL);

        /**
         * @brief Disconnects the TCP client from the server.
         *
         * @return true if the disconnect was successful.
         */
        bool disconnect();

        /**
         * @return true if the client is connected to the server.
         */
        bool is_connected();

        /**
         * @brief Register a callback which is called when the application
         * receives data.
         *
         * @param receive_cb The receive callback handle.
         */
        void register_receive_callback(receive_callback receive_cb);

        /**
         * @brief Transmits a @p buffer of size @p buffer_size.
         *
         * @param buffer The buffer to send.
         * @param buffer_size Size of the buffer.
         * @param transmission_status_cb Called when the transmission finished.
         */
        void transmit(uint8_t* buffer,
                      const size_t buffer_size,
                      transmission_status_callback transmission_status_cb);

        /**
         * @brief Transmits a @p buffer of size @p buffer_size and blocks until
         * the transmission has finished.
         *
         * @param buffer The buffer to send.
         * @param buffer_size Size of the buffer.
         */
        void transmit_blocking(uint8_t* buffer, const size_t buffer_size);
    }
}

#endif
