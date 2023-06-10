#ifndef DATA_INTERFACE_H
#define DATA_INTERFACE_H

#define BATCH_SIZE (50)

#include "ip.h"

#include "data_point.h"
#include "ground_truth_data_point.h"
#include "image_data_point.h"
#include "imu_data_point.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include <etl/vector.h>

#include <stddef.h>

namespace data_interface {
    bool initialise(const ip::IP4Address server_address,
                    const uint16_t server_port);

    void deinitialise();

    void set_start_time(const double start_time);

    void transmit_image(const ImageDataPoint& image_data_point);

    void transmit_features(
        const etl::vector<Eigen::Vector2f, MAX_NUMBER_OF_FEATURES>& features);

    void transmit_pose(const Eigen::Vector3f& position,
                       const Eigen::Quaternionf& orientation);

    bool wait_for_next_image(frontend::Image& image, const uint32_t index);

    bool wait_for_next_batch(
        etl::vector<ImuDataPoint, BATCH_SIZE>& incoming_imu_data_points,
        etl::vector<GroundTruthDataPoint, BATCH_SIZE>&
            incoming_ground_truth_data_points,
        ImageDataPoint& image_data_point);

    DataPointType
    wait_for_next_data_point(ImuDataPoint& imu_data_point,
                             ImageDataPoint& image_data_point,
                             GroundTruthDataPoint& ground_truth_data_point);
}

#endif
