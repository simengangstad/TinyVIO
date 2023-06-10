#include "tcp_client.h"

#include <stdio.h>

#include "ethernet.h"
#include "logger.h"

#include "lwip/tcp.h"

namespace tcp {

    namespace client {

        /**
         * @brief Struct which keeps track of current TCP transmission data
         * buffer.
         */
        struct DataBuffer {
            /**
             * @brief Pointer to the data to send.
             */
            uint8_t* buffer;

            /**
             * @brief Size of the buffer.
             */
            size_t size;

            /**
             * @brief Index to where in the buffer the next call to tcp_write
             * should occur from.
             */
            size_t buffer_index;

            /**
             * @brief The actual amount of bytes sent.
             */
            size_t bytes_sent;

            /**
             * @brief Callback for when the transmission has finished.
             */
            transmission_status_callback transmission_status_cb;
        };

        /**
         * @brief Struct used when TCP callbacks are called.
         */
        struct TcpClientState {

            /**
             * @brief Holds current transmission data buffer.
             */
            DataBuffer data_buffer;

            /**
             * @brief LWIP protocol control block handle.
             */
            struct tcp_pcb* control_block;

            /**
             * @brief Keeps track of whether the client is connected to the
             * server.
             */
            bool connected = false;

            /**
             * @brief Handle for connection status callback.
             */
            connection_status_callback connection_status_cb;

            /**
             * @brief Handle for callback called when data has been received.
             */
            receive_callback receive_cb;
        };

        /**
         * @brief Holds the current TCP client state.
         */
        static TcpClientState tcp_client_state;

        /**
         * @brief LWIP connection callback.
         */
        static err_t tcp_connection_callback(void* arg,
                                             struct tcp_pcb* control_block,
                                             err_t error) {

            TcpClientState* client_state = (TcpClientState*)arg;

            LWIP_ERROR("Invalid TCP control block",
                       client_state->control_block == control_block,
                       return ERR_ARG);

            if (error != ERR_OK) {
                client_state->connection_status_cb(
                    ConnectionStatus::REMOTE_NOT_AVAILABLE);

                return disconnect();
            }

            client_state->connected = true;

            if (client_state->connection_status_cb != NULL) {
                client_state->connection_status_cb(ConnectionStatus::CONNECTED);
            }

            return ERR_OK;
        }

        /**
         * @brief LWIP poll callback.
         */
        static err_t tcp_poll_callback(void* arg,
                                       struct tcp_pcb* control_block) {

            TcpClientState* client_state = (TcpClientState*)arg;

            LWIP_ERROR("Invalid TCP control block",
                       client_state->control_block == control_block,
                       return ERR_ARG);

            if (client_state->connected &&
                control_block->state != ESTABLISHED) {

                client_state->connected = false;

                if (client_state->connection_status_cb != NULL) {
                    client_state->connection_status_cb(
                        ConnectionStatus::REMOTE_DISCONNECT);
                }

                return disconnect();
            }

            return ERR_OK;
        }

        /**
         * @brief LWIP sent callback. This function will attemt to send the
         * remaining bytes in the data buffer until the whole buffer has been
         * sent.
         */
        static err_t tcp_sent_callback(void* arg,
                                       struct tcp_pcb* control_block,
                                       u16_t length) {

            TcpClientState* client_state = (TcpClientState*)arg;

            LWIP_ERROR("Invalid TCP control block",
                       client_state->control_block == control_block,
                       return ERR_ARG);

            DataBuffer* data_buffer = &client_state->data_buffer;

            const size_t data_length = data_buffer->size;

            data_buffer->bytes_sent += length;

            // If there is any remaining dat to send.
            if (data_buffer->buffer_index < data_length) {

                err_t error;

                bool should_send_more = true;

                do {
                    size_t bytes_to_send = data_length -
                                           data_buffer->buffer_index;

                    if (bytes_to_send == 0) {
                        break;
                    }

                    if (bytes_to_send > TCP_MSS) {
                        bytes_to_send = TCP_MSS;
                    }

                    // Attempt to write the TCP output buffer until it can't
                    // hold any more data
                    do {
                        error = tcp_write(
                            control_block,
                            client_state->data_buffer.buffer +
                                client_state->data_buffer.buffer_index,
                            bytes_to_send,
                            data_buffer->buffer_index + bytes_to_send <
                                    data_length
                                ? TCP_WRITE_FLAG_MORE
                                : 0);

                        if (error == ERR_MEM) {
                            bytes_to_send /= 2;
                        }

                    } while (error == ERR_MEM &&
                             (bytes_to_send >= (TCP_MSS / 2)));

                    if (error == ERR_OK) {
                        data_buffer->buffer_index += bytes_to_send;
                    } else {
                        should_send_more = false;
                    }

                } while (should_send_more);
            }

            if (data_buffer->bytes_sent >= data_buffer->size) {

                if (data_buffer->transmission_status_cb != NULL) {
                    data_buffer->transmission_status_cb();
                }
            }

            tcp_output(control_block);

            return ERR_OK;
        }

        /**
         * @brief LWIP receive callback.
         */
        err_t tcp_receive_callback(void* arg,
                                   struct tcp_pcb* control_block,
                                   struct pbuf* p,
                                   err_t error) {

            TcpClientState* client_state = (TcpClientState*)arg;

            LWIP_ERROR("Invalid TCP control block",
                       client_state->control_block == control_block,
                       return ERR_ARG);

            if (p != NULL) {

                if (client_state->receive_cb != NULL) {
                    client_state->receive_cb(p->payload, p->tot_len);
                }

                tcp_recved(control_block, p->tot_len);
                pbuf_free(p);

            } else if (error == ERR_OK) {

                if (client_state->connection_status_cb != NULL &&
                    control_block->state == CLOSE_WAIT) {

                    client_state->connected = false;
                    client_state->connection_status_cb(
                        ConnectionStatus::REMOTE_DISCONNECT);
                }

                return disconnect();
            }

            return ERR_OK;
        }

        /**
         * @brief LWIP error callback.
         */
        static void tcp_error_callback(void* arg, err_t error) {
            TcpClientState* client_state = (TcpClientState*)arg;
            LWIP_UNUSED_ARG(error);

            // Control block is already deallocated at this point, prevent
            // double-free
            client_state->control_block = NULL;

            logger::errorf("TCP error, status code: %d\r\n", error);

            client_state->connected = false;

            if (client_state->connection_status_cb != NULL) {
                client_state->connection_status_cb(
                    ConnectionStatus::REMOTE_NOT_AVAILABLE);
            }

            disconnect();
        }

        bool connect(const ip::IP4Address server_address,
                     const uint16_t server_port,
                     connection_status_callback connection_status_cb) {

            if (!ethernet::is_initialised()) {
                logger::errorf("Ethernet driver is not initialised!\r\n");
                return false;
            }

            tcp_client_state.data_buffer.buffer       = NULL;
            tcp_client_state.data_buffer.size         = 0;
            tcp_client_state.data_buffer.buffer_index = 0;

            tcp_client_state.control_block = tcp_new_ip_type(IP_GET_TYPE());
            tcp_client_state.control_block->callback_arg = &tcp_client_state;
            tcp_client_state.control_block->poll         = tcp_poll_callback;
            tcp_client_state.control_block->sent         = tcp_sent_callback;
            tcp_client_state.control_block->recv         = tcp_receive_callback;
            tcp_client_state.control_block->errf         = tcp_error_callback;
            tcp_client_state.control_block->pollinterval = 2;

            tcp_client_state.connected            = false;
            tcp_client_state.connection_status_cb = connection_status_cb;

            ip4_addr_t netif_server_address;

            IP4_ADDR(&netif_server_address,
                     server_address.a,
                     server_address.b,
                     server_address.c,
                     server_address.d);

            const err_t error = tcp_connect(tcp_client_state.control_block,
                                            &netif_server_address,
                                            server_port,
                                            tcp_connection_callback);

            if (error != ERR_OK) {
                logger::errorf("Failed to connect TCP, error code: %d\r\n",
                               error);
                disconnect();
                return false;
            } else {
                logger::debugf("Awaiting TCP connection...\r\n");
            }

            return true;
        }

        bool disconnect() {

            tcp_client_state.connected = false;

            tcp_client_state.connection_status_cb = NULL;

            if (tcp_client_state.control_block != NULL) {
                tcp_client_state.control_block->callback_arg = NULL;
                tcp_client_state.control_block->poll         = NULL;
                tcp_client_state.control_block->sent         = NULL;
                tcp_client_state.control_block->recv         = NULL;
                tcp_client_state.control_block->errf         = NULL;

                const err_t error = tcp_close(tcp_client_state.control_block);

                if (error != ERR_OK) {
                    logger::errorf(
                        "Failed to close TCP client, error code: %d\r\n",
                        error);

                    return false;
                }

                mem_free(tcp_client_state.control_block);
            }

            return true;
        }

        bool is_connected() { return tcp_client_state.connected; }

        void register_receive_callback(receive_callback receive_cb) {
            tcp_client_state.receive_cb = receive_cb;
        }

        void transmit(uint8_t* buffer,
                      const size_t buffer_size,
                      transmission_status_callback transmission_status_cb) {

            if (!tcp_client_state.connected) {
                logger::errorf("Attempt to send buffer with TCP without being "
                               "connected to the server\r\n");
                return;
            }

            if (tcp_client_state.data_buffer.buffer_index !=
                tcp_client_state.data_buffer.size) {
                logger::errorf("Attempted to send new data buffer before the "
                               "previous one completed.\r\n");
                return;
            }

            tcp_client_state.data_buffer.buffer       = buffer;
            tcp_client_state.data_buffer.size         = buffer_size;
            tcp_client_state.data_buffer.buffer_index = 0;
            tcp_client_state.data_buffer.bytes_sent   = 0;
            tcp_client_state.data_buffer.transmission_status_cb =
                transmission_status_cb;

            size_t bytes_to_send = tcp_client_state.data_buffer.size;

            if (bytes_to_send > TCP_MSS) {
                bytes_to_send = TCP_MSS;
            }

            u8_t flags = bytes_to_send < buffer_size ? TCP_WRITE_FLAG_MORE : 0;

            err_t error = tcp_write(tcp_client_state.control_block,
                                    tcp_client_state.data_buffer.buffer,
                                    bytes_to_send,
                                    flags);

            if (error != ERR_OK) {
                logger::errorf("Error sending data buffer, error code: %d\r\n",
                               error);
                return;
            }

            error = tcp_output(tcp_client_state.control_block);

            if (error != ERR_OK) {
                logger::errorf(
                    "Error outputting data buffer, error code: %d\r\n",
                    error);
                return;
            }

            tcp_client_state.data_buffer.buffer_index += bytes_to_send;
        }

        /**
         * @brief For blocking transmit, this keeps track of when the
         * transmission has completed
         */
        static volatile bool blocking_transmission_complete = false;

        /**
         * @brief Local callback for blocking transmission.
         */
        static void blocking_transmission_status_callback() {
            blocking_transmission_complete = true;
        }

        void transmit_blocking(uint8_t* buffer, const size_t buffer_size) {
            blocking_transmission_complete = false;

            transmit(buffer,
                     buffer_size,
                     blocking_transmission_status_callback);

            while (!blocking_transmission_complete) { ethernet::poll(); }
        }
    }
}
