#ifndef AUGMENTED_IMU_STATE_H
#define AUGMENTED_IMU_STATE_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include <etl/map.h>

#include "vio_typedefs.h"

/*
 * @brief Snapshot of the IMU state for a particular time instance, used for the
 * augmented IMU states in the filter.
 */
struct AugmentedIMUState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Timestamp for the snapshot.
     */
    double timestamp = 0.0;

    /**
     * @brief Time interval to the nearest image.
     */
    double time_interval_to_nearest_image = 0.0;

    /**
     * @brief Orientation from the IMU frame to the world frame at the
     * particular time when this IMU state was augmented.
     */
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();

    /**
     * @brief Position of the IMU frame given in world frame at the
     * particular time when this IMU state was augmented.
     */
    Eigen::Vector3f position = Eigen::Vector3f::Zero();

    /**
     * @brief First estimated jacobian of the position.
     */
    Eigen::Vector3f position_FEJ = Eigen::Vector3f::Zero();

    /**
     * @brief IMU-camera extrinsic. Transformation from IMU frame to the
     * camera frame (read right to left).
     */
    Eigen::Isometry3f T_camera_imu = Eigen::Isometry3f::Identity();

    /**
     * @brief The corresponding camera pose transformation. Transforming from
     * the camera frame to the world frame (read right to left).
     */
    Eigen::Isometry3f T_world_camera = Eigen::Isometry3f::Identity();

    AugmentedIMUState() = default;
};

/**
 * @brief Convenience typedef for a set of augmented IMU states.
 */
typedef etl::
    map<StateIdentifier, AugmentedIMUState, MAX_NUMBER_OF_AUGMENTED_STATES>
        AugmentedIMUStatesMap;

#endif
