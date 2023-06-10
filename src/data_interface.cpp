#include "data_interface.h"

#include "ethernet.h"
#include "logger.h"
#include "tcp_client.h"

#include <etl/circular_buffer.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include <stdlib.h>

#define FEATURES_PER_PACKET (256)

#define HEADER_SIZE               (16)
#define HEADER_TYPE_INDEX         (0)
#define HEADER_PAYLOAD_TYPE_INDEX (1)
#define HEADER_PAYLOAD_SIZE_INDEX (2)
#define HEADER_METADATA_INDEX     (8)

namespace data_interface {

    /**
     * @brief The circular buffer has to be large enough for an image, the
     * timestamp (a double) and the with and height (u32). We also include
     * enough space for 50 IMU data points and 50 ground truth data points.
     */
    typedef etl::circular_buffer<
        uint8_t,
        IMAGE_WIDTH * IMAGE_HEIGHT + HEADER_SIZE + sizeof(double) +
            2 * sizeof(uint32_t) +
            (2 * sizeof(double) + 2 * HEADER_SIZE + sizeof(ImuDataPoint) +
             sizeof(GroundTruthDataPoint) * BATCH_SIZE)>
        ReceiveBuffer;

    enum class HeaderType { Transmission = 0, Request = 1 };

    enum class RequestType {
        Initialisation = 0,
        NewDataPoint   = 1,
        NewBatch       = 2,
        None           = 99
    };

    enum class TransmissionType {
        Imu         = 0,
        Image       = 1,
        GroundTruth = 2,
        Features    = 3,
        Pose        = 4,
        Batch       = 5,
        None        = 99
    };

    TransmissionType transmission_type_from_data_point_type(
        const DataPointType data_point_type) {
        switch (data_point_type) {
        case DataPointType::Imu:
            return TransmissionType::Imu;
        case DataPointType::Image:
            return TransmissionType::Image;
        case DataPointType::GroundTruth:
            return TransmissionType::GroundTruth;
        case DataPointType::None:
            return TransmissionType::None;
        }

        return TransmissionType::None;
    }

    DataPointType data_point_type_from_transmission_type(
        const TransmissionType transmission_type) {
        switch (transmission_type) {
        case TransmissionType::Imu:
            return DataPointType::Imu;
        case TransmissionType::Image:
            return DataPointType::Image;
        case TransmissionType::GroundTruth:
            return DataPointType::GroundTruth;
        default:
            return DataPointType::None;
        }
    }

    template <typename T> union TypeUnion {
        T value;
        uint8_t bytes[sizeof(T)];

        TypeUnion() {}

        TypeUnion(T val) : value(val) {}

        TypeUnion(ReceiveBuffer& receive_buffer) {

            for (size_t i = 0; i < sizeof(T); i++) {
                bytes[i] = receive_buffer.front();
                receive_buffer.pop();
            }
        }

        void populate(uint8_t* buffer) {
            for (size_t i = 0; i < sizeof(T); i++) { buffer[i] = bytes[i]; }
        }
    };

    union Vector3Union {
        float vector[3];
        uint8_t bytes[sizeof(vector)];

        Vector3Union() {}

        Vector3Union(const Eigen::Vector3f& v) {
            vector[0] = v.x();
            vector[1] = v.y();
            vector[2] = v.z();
        }

        Vector3Union(ReceiveBuffer& receive_buffer) {

            for (size_t i = 0; i < sizeof(vector); i++) {
                bytes[i] = receive_buffer.front();
                receive_buffer.pop();
            }
        }

        void populate(uint8_t* buffer) {
            for (size_t i = 0; i < sizeof(vector); i++) {
                buffer[i] = bytes[i];
            }
        }

        Eigen::Vector3f to_eigen() const {
            return Eigen::Vector3f(vector[0], vector[1], vector[2]);
        }
    };

    union QuaternionUnion {
        float vector[4];
        uint8_t bytes[sizeof(vector)];

        QuaternionUnion() {}

        QuaternionUnion(const Eigen::Quaternionf& q) {
            vector[0] = q.x();
            vector[1] = q.y();
            vector[2] = q.z();
            vector[3] = q.w();
        }

        QuaternionUnion(ReceiveBuffer& receive_buffer) {

            for (size_t i = 0; i < sizeof(vector); i++) {
                bytes[i] = receive_buffer.front();
                receive_buffer.pop();
            }
        }

        void populate(uint8_t* buffer) {
            for (size_t i = 0; i < sizeof(vector); i++) {
                buffer[i] = bytes[i];
            }
        }

        Eigen::Quaternionf to_eigen() const {
            return Eigen::Quaternionf(vector[0],
                                      vector[1],
                                      vector[2],
                                      vector[3]);
        }
    };

    /**
     * @brief Header for a request or transmission to/from the TCP server.
     *
     * The header consists of the following bytes:
     *
     * 0: Type of header (transmission or request)
     * 1: Transmission or request type
     * 2: Size of the payload
     * 3-7: Not used
     *
     * 8-15: Metadata for the given request or transmission
     */
    struct Header {

        uint8_t data[HEADER_SIZE];

        Header() {}

        Header(const TransmissionType transmission_type,
               const uint32_t payload_size) {
            data[HEADER_TYPE_INDEX] = static_cast<uint8_t>(
                HeaderType::Transmission);
            data[HEADER_PAYLOAD_TYPE_INDEX] = static_cast<uint8_t>(
                transmission_type);

            TypeUnion<uint32_t>(payload_size)
                .populate(&data[HEADER_PAYLOAD_SIZE_INDEX]);
        }

        Header(const RequestType request_type) {

            data[HEADER_TYPE_INDEX] = static_cast<uint8_t>(HeaderType::Request);
            data[HEADER_PAYLOAD_TYPE_INDEX] = static_cast<uint8_t>(
                request_type);
        }

        bool is_transmission() {
            return static_cast<HeaderType>(data[HEADER_TYPE_INDEX]) ==
                   HeaderType::Transmission;
        }

        bool is_request() {
            return static_cast<HeaderType>(data[HEADER_TYPE_INDEX]) ==
                   HeaderType::Request;
        }

        TransmissionType transmission_type() {

            if (!is_transmission()) {
                return TransmissionType::None;
            }

            switch (static_cast<TransmissionType>(
                data[HEADER_PAYLOAD_TYPE_INDEX])) {
            case TransmissionType::Imu:
                return TransmissionType::Imu;
            case TransmissionType::Image:
                return TransmissionType::Image;
            case TransmissionType::GroundTruth:
                return TransmissionType::GroundTruth;
            case TransmissionType::Features:
                return TransmissionType::Features;
            case TransmissionType::Pose:
                return TransmissionType::Pose;
            case TransmissionType::Batch:
                return TransmissionType::Batch;
            default:
                return TransmissionType::None;
            }
        }

        RequestType request_type() {
            if (!is_request()) {
                return RequestType::None;
            }

            switch (static_cast<RequestType>(data[HEADER_PAYLOAD_TYPE_INDEX])) {
            case RequestType::Initialisation:
                return RequestType::Initialisation;
            case RequestType::NewDataPoint:
                return RequestType::NewDataPoint;
            case RequestType::NewBatch:
                return RequestType::NewBatch;
            default:
                return RequestType::None;
            }
        }

        uint32_t payload_size() {

            uint32_t size = 0;

            size |= ((uint32_t)data[HEADER_PAYLOAD_SIZE_INDEX]) << 24;
            size |= ((uint32_t)data[HEADER_PAYLOAD_SIZE_INDEX + 1]) << 16;
            size |= ((uint32_t)data[HEADER_PAYLOAD_SIZE_INDEX + 2]) << 8;
            size |= ((uint32_t)data[HEADER_PAYLOAD_SIZE_INDEX + 3]);

            return size;
        }

        void transmit() const {
            tcp::client::transmit_blocking((uint8_t*)data, sizeof(data));
        }
    };

    /**
     * @brief Header for features.
     *
     * Consists of:
     * - the header identifier (1 byte)
     * - amount of features (4 bytes)
     */
    /*struct FeatureHeader : Header {
        FeatureHeader(const size_t size) {
            data[0] = DATA_PACKET_TYPE_FEATURES;

            data[1] = size >> 24;
            data[2] = (size & 0x00FF0000) >> 16;
            data[3] = (size & 0x0000FF00) >> 8;
            data[4] = (size & 0x000000FF);
        }
    };*/

    struct InitialisationHeader : Header {

        InitialisationHeader(const double start_time)
            : Header(RequestType::Initialisation) {

            TypeUnion<double> timestamp;
            timestamp.value = start_time;

            data[HEADER_METADATA_INDEX]     = timestamp.bytes[0];
            data[HEADER_METADATA_INDEX + 1] = timestamp.bytes[1];
            data[HEADER_METADATA_INDEX + 2] = timestamp.bytes[2];
            data[HEADER_METADATA_INDEX + 3] = timestamp.bytes[3];

            data[HEADER_METADATA_INDEX + 4] = timestamp.bytes[4];
            data[HEADER_METADATA_INDEX + 5] = timestamp.bytes[5];
            data[HEADER_METADATA_INDEX + 6] = timestamp.bytes[6];
            data[HEADER_METADATA_INDEX + 7] = timestamp.bytes[7];
        }
    };

    /**
     * @brief Temporary storage for the features when transmitting. Every
     * feature has two points (x, y) (16-bit numbers).
     */
    // static uint8_t features_buffer[FEATURES_PER_PACKET * (2 * 2)];

    /**
     * @brief Holds the messages received.
     *
     * The circular buffer has to be large enough for an image, the timestamp (a
     * double) and the with and height (u32).
     */
    static ReceiveBuffer receive_buffer;

    static etl::circular_buffer<ImuDataPoint, MAX_NUMBER_OF_IMU_MEASUREMENTS>
        imu_data_points;

    static etl::circular_buffer<ImageDataPoint, 2> image_data_points;

    static etl::circular_buffer<GroundTruthDataPoint, 100>
        ground_truth_data_points;

    static void connection_status_callback(
        tcp::client::ConnectionStatus connection_status) {
        logger::infof("Connection status: %d\r\n", connection_status);
    }

    static void receive_callback(void* data, const uint16_t data_size) {

        for (uint16_t index = 0; index < data_size; index++) {
            receive_buffer.push(((uint8_t*)data)[index]);
        }
    }

    bool initialise(const ip::IP4Address server_address,
                    const uint16_t server_port) {

        if (!tcp::client::connect(server_address,
                                  server_port,
                                  connection_status_callback)) {
            return false;
        }

        tcp::client::register_receive_callback(receive_callback);

        while (!tcp::client::is_connected()) { ethernet::poll(); }

        return true;
    }

    void deinitialise() { tcp::client::disconnect(); }

    void set_start_time(const double start_time) {
        InitialisationHeader(start_time).transmit();
    }

    void transmit_image(const ImageDataPoint& image_data_point) {

        // First notify the server that we're about to transmit an image

        // The payload consists of the image, the width and height (both 32-bit)
        // and the timestamp (double)
        const uint32_t payload_size = image_data_point.image.width *
                                          image_data_point.image.height +
                                      2 * sizeof(uint32_t) + sizeof(double);

        Header(TransmissionType::Image, payload_size).transmit();

        // Now we need to transmit the width, height and timestamp
        uint8_t metadata[sizeof(uint32_t) * 2 + sizeof(double)];

        TypeUnion<double>(image_data_point.timestamp).populate(&metadata[0]);
        TypeUnion<uint32_t>(image_data_point.image.width)
            .populate(&metadata[8]);
        TypeUnion<uint32_t>(image_data_point.image.height)
            .populate(&metadata[12]);

        tcp::client::transmit_blocking(metadata, sizeof(metadata));

        // And finally transmit the image data
        tcp::client::transmit_blocking(image_data_point.image.data,
                                       image_data_point.image.width *
                                           image_data_point.image.height);
    }

    void transmit_pose(const Eigen::Vector3f& position,
                       const Eigen::Quaternionf& orientation) {

        const uint32_t payload_size = (3 + 4) * sizeof(float);
        Header(TransmissionType::Pose, payload_size).transmit();

        uint8_t buffer[payload_size];

        Vector3Union(position).populate(&buffer[0]);
        QuaternionUnion(orientation).populate(&buffer[3 * sizeof(float)]);

        tcp::client::transmit_blocking(buffer, sizeof(buffer));
    }

    void transmit_features(
        __attribute__((unused))
        const etl::vector<Eigen::Vector2f, MAX_NUMBER_OF_FEATURES> features) {

        /*

        if (features.empty()) {
            return;
        }

        const FeatureHeader header(features.size());
        tcp::client::transmit_blocking((uint8_t*)header.data,
                                       sizeof(header.data));

        size_t features_accumulated = 0;

        for (const Eigen::Vector2f& feature : features) {

            const uint16_t x = feature.x();
            const uint16_t y = feature.y();

            features_buffer[features_accumulated * 4]     = x >> 8;
            features_buffer[features_accumulated * 4 + 1] = x & 0xFF;
            features_buffer[features_accumulated * 4 + 2] = y >> 8;
            features_buffer[features_accumulated * 4 + 3] = y & 0xFF;

            features_accumulated++;

            if (features_accumulated == FEATURES_PER_PACKET) {

                tcp::client::transmit_blocking(features_buffer,
                                               features_accumulated * 4);

                features_accumulated = 0;
            }
        }
        */
    }

    bool wait_for_next_batch(
        etl::vector<ImuDataPoint, BATCH_SIZE>& incoming_imu_data_points,
        etl::vector<GroundTruthDataPoint, BATCH_SIZE>&
            incoming_ground_truth_data_points,
        ImageDataPoint& image_data_point) {

        if (!ethernet::is_initialised()) {
            return false;
        }

        if (!tcp::client::is_connected()) {
            return false;
        }

        // Clear the receive buffer first in case there is something there
        receive_buffer.clear();

        logger::debugf("Sending new batch request\r\n");

        // First send signal that we are ready to receive a new data point
        Header(RequestType::NewBatch).transmit();

        // Then wait for the header
        while (receive_buffer.size() < HEADER_SIZE) { ethernet::poll(); }

        Header header;

        for (size_t i = 0; i < HEADER_SIZE; i++) {
            if (receive_buffer.empty()) {
                logger::errorf("Failed to pop value at index %d from receive "
                               "message buffer\r\n",
                               i);
                return false;
            }

            header.data[i] = receive_buffer.front();
            receive_buffer.pop();
        }

        const size_t payload_size = header.payload_size();

        if (payload_size == 0) {
            return false;
        }

        logger::debugf("Payload size: %d\r\n", payload_size);

        // Wait for the whole payload to arrive
        while (receive_buffer.size() < payload_size) { ethernet::poll(); }

        logger::debugf("Got payload size: %d\r\n", payload_size);

        // Decode all the entries

        size_t bytes_decoded = 0;

        while (bytes_decoded < payload_size) {
            Header data_point_header;

            const size_t receive_buffer_size_before_decoding =
                receive_buffer.size();

            for (size_t i = 0; i < HEADER_SIZE; i++) {
                if (receive_buffer.empty()) {
                    logger::errorf(
                        "Failed to pop value at index %d from receive "
                        "message buffer\r\n",
                        i);
                    return false;
                }

                data_point_header.data[i] = receive_buffer.front();
                receive_buffer.pop();
            }

            const DataPointType data_point_type =
                data_point_type_from_transmission_type(
                    data_point_header.transmission_type());
            const uint32_t data_point_payload_size =
                data_point_header.payload_size();

            logger::debugf("Payload size for data point %d: %d\r\n",
                           data_point_header.transmission_type(),
                           data_point_payload_size);

            switch (data_point_type) {
            case DataPointType::Imu: {

                ImuDataPoint imu_data_point;

                const TypeUnion<double> timestamp(receive_buffer);
                const Vector3Union linear_acceleration(receive_buffer);
                const Vector3Union angular_velocity(receive_buffer);

                imu_data_point.timestamp = timestamp.value;
                imu_data_point.linear_acceleration =
                    linear_acceleration.to_eigen();
                imu_data_point.angular_velocity = angular_velocity.to_eigen();

                incoming_imu_data_points.push_back(imu_data_point);

                break;
            }

            case DataPointType::Image: {

                const TypeUnion<double> timestamp(receive_buffer);
                const TypeUnion<uint32_t> width(receive_buffer);
                const TypeUnion<uint32_t> height(receive_buffer);

                image_data_point.timestamp    = timestamp.value;
                image_data_point.image.width  = width.value;
                image_data_point.image.height = height.value;

                for (size_t i = 0; i < image_data_point.image.width *
                                           image_data_point.image.height;
                     i++) {
                    image_data_point.image.data[i] = receive_buffer.front();
                    receive_buffer.pop();
                }

                return true;
            }

            case DataPointType::GroundTruth: {

                GroundTruthDataPoint ground_truth_data_point;

                const TypeUnion<double> timestamp(receive_buffer);

                ground_truth_data_point.timestamp = timestamp.value;

                const Vector3Union position(receive_buffer);
                const QuaternionUnion orientation(receive_buffer);
                const Vector3Union velocity(receive_buffer);
                const Vector3Union acceleration_bias(receive_buffer);
                const Vector3Union angular_velocity_bias(receive_buffer);

                ground_truth_data_point.position    = position.to_eigen();
                ground_truth_data_point.orientation = orientation.to_eigen();
                ground_truth_data_point.velocity    = velocity.to_eigen();
                ground_truth_data_point.acceleration_bias =
                    acceleration_bias.to_eigen();
                ground_truth_data_point.angular_velocity_bias =
                    angular_velocity_bias.to_eigen();

                incoming_ground_truth_data_points.push_back(
                    ground_truth_data_point);

                break;
            }

            case DataPointType::None:

                logger::errorf("Got invalid data point\r\n");
                return false;
            }

            bytes_decoded += (receive_buffer_size_before_decoding -
                              receive_buffer.size());
        }

        return true;
    }

    DataPointType
    wait_for_next_data_point(ImuDataPoint& imu_data_point,
                             ImageDataPoint& image_data_point,
                             GroundTruthDataPoint& ground_truth_data_point) {

        if (!ethernet::is_initialised()) {
            return DataPointType::None;
        }

        if (!tcp::client::is_connected()) {
            return DataPointType::None;
        }

        // Clear the receive buffer first in case there is something there
        receive_buffer.clear();

        logger::debugf("Sending new data point request\r\n");

        // First send signal that we are ready to receive a new data point
        Header(RequestType::NewDataPoint).transmit();

        // Then wait for the header
        while (receive_buffer.size() < HEADER_SIZE) { ethernet::poll(); }

        Header header;

        for (size_t i = 0; i < HEADER_SIZE; i++) {
            if (receive_buffer.empty()) {
                logger::errorf("Failed to pop value at index %d from receive "
                               "message buffer\r\n",
                               i);
                return DataPointType::None;
            }

            header.data[i] = receive_buffer.front();
            receive_buffer.pop();
        }

        // Then decode the header
        const DataPointType data_point_type =
            data_point_type_from_transmission_type(header.transmission_type());
        const uint32_t payload_size = header.payload_size();

        logger::debugf("Payload size: %d\r\n", payload_size);

        // Wait for the whole payload to arrive
        while (receive_buffer.size() < payload_size) { ethernet::poll(); }

        switch (data_point_type) {
        case DataPointType::Imu: {

            const TypeUnion<double> timestamp(receive_buffer);
            const Vector3Union linear_acceleration(receive_buffer);
            const Vector3Union angular_velocity(receive_buffer);

            imu_data_point.timestamp           = timestamp.value;
            imu_data_point.linear_acceleration = linear_acceleration.to_eigen();
            imu_data_point.angular_velocity    = angular_velocity.to_eigen();

            return DataPointType::Imu;
        }

        case DataPointType::Image: {

            const TypeUnion<double> timestamp(receive_buffer);
            const TypeUnion<uint32_t> width(receive_buffer);
            const TypeUnion<uint32_t> height(receive_buffer);

            image_data_point.timestamp    = timestamp.value;
            image_data_point.image.width  = width.value;
            image_data_point.image.height = height.value;

            for (size_t i = 0; i < image_data_point.image.width *
                                       image_data_point.image.height;
                 i++) {
                image_data_point.image.data[i] = receive_buffer.front();
                receive_buffer.pop();
            }

            return DataPointType::Image;
        }

        case DataPointType::GroundTruth: {

            const TypeUnion<double> timestamp(receive_buffer);

            ground_truth_data_point.timestamp = timestamp.value;

            const Vector3Union position(receive_buffer);
            const QuaternionUnion orientation(receive_buffer);
            const Vector3Union velocity(receive_buffer);
            const Vector3Union acceleration_bias(receive_buffer);
            const Vector3Union angular_velocity_bias(receive_buffer);

            ground_truth_data_point.position    = position.to_eigen();
            ground_truth_data_point.orientation = orientation.to_eigen();
            ground_truth_data_point.velocity    = velocity.to_eigen();
            ground_truth_data_point.acceleration_bias =
                acceleration_bias.to_eigen();
            ground_truth_data_point.angular_velocity_bias =
                angular_velocity_bias.to_eigen();

            return DataPointType::GroundTruth;
        }

        case DataPointType::None:

            return DataPointType::None;
        }

        return DataPointType::None;
    }
}
