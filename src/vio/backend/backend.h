#ifndef BACKEND_H
#define BACKEND_H

#include <etl/vector.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include "feature_observation.h"
#include "feature_track.h"
#include "imu_data_point.h"
#include "imu_state.h"
#include "vio_config.h"

namespace backend {

    /**
     * @brief Size of the IMU error-state, includes:
     * - Orientation error (3)
     * - Velocity error (3)
     * - Position error (3)
     * - Angular velocity bias (3)
     * - Accelerometer bias (3)
     * - IMU-Camera extrinsic orientation (3)
     * - IMU-Camera extrinsic position (3)
     * - Timestamp compensation (1)
     */
    constexpr long IMU_ERROR_STATE_SIZE = (22L);

    /*
     * @brief Initialize the backend.
     *
     * @param configuration[in] The configuration for the backend.
     */
    void configure(const Configuration& configuration);

    /**
     * @return True when the filter has initialised.
     */
    bool has_initialised();

    /**
     * @brief Processes the feature observations from the frontend and the IMU
     * measurements, will propagate the state in the filter and potentially do a
     * filter update if the feature observations have gone out of frame or if
     * the maximum number of augment IMU states (the sliding window) is
     * surpassed.
     *
     * @param feature_observations [in] The feature observations.
     * @param imu_measurements [in, out] The IMU measurements. Those entries
     * which are used in the backend for propagation are removed from this
     * buffer.
     */
    bool process_feature_observations(
        const FeatureObservations& feature_observations,
        IMUBuffer& imu_measurements);

    /**
     * @return The IMU state estimate, the state vector without the augmented
     * IMU states.
     */
    IMUState get_imu_state();

    /**
     * @brief Returns the IMU state covariance. This is not the full state
     * covariance with the augmented IMU states, only the covariance related to
     * the filter's estimates of the IMU state.
     */
    Eigen::Matrix<float, IMU_ERROR_STATE_SIZE, IMU_ERROR_STATE_SIZE>
    get_imu_state_covariance();

    /**
     * @brief Returns the feature tracks.
     */
    etl::vector<FeatureTrack, MAX_NUMBER_OF_FEATURE_TRACKS>
    get_feature_tracks();

    /**
     * @brief Debugging structure, used for observing the state of the backend.
     */
    struct BackendInformation {
        bool initialised;
        bool is_stationary;
        bool measurement_update_features;
        bool measurement_update_pruning_states;
    };

    /**
     * @brief Returns the current backend information.
     */
    BackendInformation get_backend_information();
}

#endif
