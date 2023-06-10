#ifndef STATIC_INITIALIZER_H
#define STATIC_INITIALIZER_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include "feature_observation.h"
#include "imu_data_point.h"
#include "imu_state.h"

namespace static_initialiser {

    /**
     * @brief Resets the static initialiser.
     *
     * @param max_feature_distance The max feature distance threshold between
     * two corresponding features. Used to determine if the features are
     * stationary, and thus the agent is stationary.
     * @param static_frames_threshold Number of frames the agent has to remain
     * stationary before we declare that the static initialisation can be
     * performed.
     * @param time_offset_between_imu_and_image The offset between IMU
     * measurements and camera/image frames. Used to relate IMU measurements to
     * the image/camera frames.
     */
    void reset(const double& max_feature_distance,
               const int& static_frames_threshold,
               const double& time_offset_between_imu_and_image);

    /**
     * @brief Attempts static initialisation of the agent.
     *
     * @param imu_measurements The IMU measurements.
     * @param feature_observations The feature observations used to
     * determine if the agent is stationary by checking consecutive features'
     * movement.
     * @param out_initialisation_timestamp The initialisation timestamp will be
     * placed in this variable if the initialisation succeeds.
     * @param out_initial_angular_velocity_bias The initial angular velocity
     * bias will be placed in this variables if the initialisation succeeds.
     * @param out_initial_orientation The initial orientation will be placed in
     * this variable if the initialisation succeeds.
     *
     * @return True if the initialisation succeeds.
     */
    bool
    attempt_initialisation(const IMUBuffer& imu_measurements,
                           const FeatureObservations& feature_observations,
                           double& out_initialisation_timestamp,
                           Eigen::Vector3f& out_initial_angular_velocity_bias,
                           Eigen::Quaternionf& out_initial_orientation);
}

#endif
