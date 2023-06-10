#ifndef IMU_STATE_H
#define IMU_STATE_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include <etl/map.h>
#include <etl/vector.h>

#include "vio_typedefs.h"

/**
 * @brief Full IMU state structure, used as the current estimate in the filter.
 */
struct IMUState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief The unique identifier for the IMU state.
     */
    StateIdentifier identifier = 0;

    /**
     * @brief Timestamp when state was recorded.
     */
    double timestamp = 0.0;

    /**
     * @brief Time interval to the nearest image.
     */
    double time_interval_to_nearest_image = 0.0;

    /**
     * @brief The rotation from the IMU frame to the world frame.
     */
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();

    /**
     * @brief The position of the IMU frame expressed in world frame.
     */
    Eigen::Vector3f position = Eigen::Vector3f::Zero();

    /**
     * @brief Velocity of the IMU frame expressed in the world frame.
     */
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

    /**
     * @brief Bias for measured angular_velocity.
     */
    Eigen::Vector3f angular_velocity_bias = Eigen::Vector3f::Zero();

    /**
     * @brief Bias for measured acceleration.
     */
    Eigen::Vector3f acceleration_bias = Eigen::Vector3f::Zero();

    /**
     * @brief IMU-camera extrinsic. Transformation from IMU frame to the
     * camera frame (read right to left).
     */
    Eigen::Isometry3f T_camera_imu = Eigen::Isometry3f::Identity();

    IMUState() = default;

    explicit IMUState(const StateIdentifier& new_identifier)
        : identifier(new_identifier) {}

    void reset() {
        identifier                     = 0;
        timestamp                      = 0.0;
        time_interval_to_nearest_image = 0.0;
        orientation.setIdentity();
        position.setZero();
        velocity.setZero();
        angular_velocity_bias.setZero();
        acceleration_bias.setZero();
        T_camera_imu.setIdentity();
    }
};

#endif
