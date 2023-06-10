#include "backend.h"

#include <cfloat>
#include <cmath>
#include <stdio.h>

#include <etl/list.h>
#include <etl/map.h>
#include <etl/stack.h>

#include "Eigen/src/Core/Matrix.h"
#include "allocator.h"
#include "augmented_imu_state.h"
#include "board.h"
#include "feature_track.h"
#include "frontend.h"
#include "imu_state.h"
#include "math_utils.h"
#include "profile.h"
#include "static_initialiser.h"

#ifndef CPU_MIMXRT1176DVMAA
#include <opencv2/opencv.hpp>
#define PRINTF printf
#else
#define PRINTF logger::infof
#endif

/**
 * @brief Primary buffer for the eigen allocator.
 */
AT_QUICKACCESS_SECTION_DATA(static uint8_t primary_buffer[0x60000]);

/**
 * @brief Secondary buffer for the eigen allocator.
 */
AT_OCRAM_SECTION_DATA(static uint8_t secondary_buffer[0xB1800]);

/**
 * @brief Custom allocator for Eigen. Makes Eigen use this allocator for all
 * heap usage.
 */
AT_QUICKACCESS_SECTION_DATA(
    Allocator eigen_allocator(primary_buffer,
                              sizeof(primary_buffer),
                              secondary_buffer,
                              sizeof(secondary_buffer)));

/**
 * @brief Allocates @p size bytes
 */
void* eigen_custom_malloc(const size_t size) {
    return eigen_allocator.allocate(size);
}

/**
 * @brief Frees @p ptr.
 */
void eigen_custom_free(void* ptr) { eigen_allocator.free(ptr); }

/**
 * @brief Reallocates @p ptr with the new size @p size
 */
void* eigen_custom_realloc(void* ptr, const size_t size) {
    return eigen_allocator.reallocate(ptr, size);
}

/**
 * @brief Chi-squared 95th percentile confidence interval. We have a 0 at the
 * start here to make the degrees-of-freedom correspond with the 0-indexing.
 */
static double chi_squared_test_table[] = {
    0.0,
    0.0039321400000195232127,
    0.10258658877510107299,
    0.35184631774927144221,
    0.7107230213973241284,
    1.1454762260617692426,
    1.6353828943279067332,
    2.1673499092980570424,
    2.7326367934996618203,
    3.3251128430668148717,
    3.9402991361190600195,
    4.5748130793222241408,
    5.2260294883926405873,
    5.8918643377098476321,
    6.5706313837893439711,
    7.2609439276700298294,
    7.9616455723785506393,
    8.6717602046700772433,
    9.390455080688981937,
    10.117013063859044308,
    10.850811394182585445,
    11.591305208820736894,
    12.338014578790645004,
    13.090514188172798882,
    13.848425027170213397,
    14.611407639483305232,
    15.37915658326173407,
    16.151395849664105242,
    16.927875044422496131,
    17.708366182824583746,
    18.492660981953466859,
    19.280568559129289241,
    20.071913464548288175,
    20.866533990714788871,
    21.664280712551974517,
    22.46501522088268743,
    23.268609018893769758,
    24.074942556679907568,
    24.883904383335622157,
    25.695390399574776552,
    26.509303196693110749,
    27.325551469994191933,
    28.144049496682630007,
    28.96471666977569015,
    29.787477080861954448,
    30.612259145595476895,
    31.438995266697048692,
    32.267621529973396832,
    33.098077429486295387,
    33.930305618527832223,
    34.764251683501747436,
    35.599863938188292423,
    36.437093236191635981,
    37.27589279964429636,
    38.116218062479397588,
    38.958026526785090482,
    39.801277630931259921,
    40.64593262831063214,
    41.491954475668954672,
    42.339307730113461048,
    43.187958453989764962,
    44.037874126904725358,
    44.88902356425022333,
    45.741376841650335905,
    46.594905224813963684,
    47.44958110432793319,
    48.305377934971758691,
    49.162270179176807972,
    50.020233254289266256,
    50.879243483328636444,
    51.73927804896290894,
    52.600314950447234708,
    53.462332963296205435,
    54.325311601480684942,
    55.189231081958702418,
    56.054072291366608738,
    56.919816754711987983,
    57.786446605923181608,
    58.653944560122617702,
    59.522293887502257803,
    60.391478388689463941,
    61.261482371500676436,
    62.132290628988528169,
    63.003888418695503049,
    63.876261443034167087,
    64.749395830719990386,
    65.623278119188640289,
    66.49789523793464241,
    67.373234492713166333,
    68.249283550550828181,
    69.126030425515523348,
    70.00346346519876306,
    70.881571337867427474,
    71.760343020245002776,
    72.639767785884686191,
    73.519835194100096487,
    74.400535079420933471,
    75.281857541543672596,
    76.163792935749071944,
    77.046331863760286751,
};

/**
 * @brief The optimisation configuration for the feature tracks.
 */
FeatureTrack::OptimisationConfig FeatureTrack::optimisation_config;

namespace backend {

    using namespace Eigen;

    /**
     * @brief The process covariance consists of the following:
     * - Angular velocity variance (3)
     * - Accelerometer variance (3)
     * - Angular velocity random walk variance (3)
     * - Accelerometer random walk variance (3)
     */
    constexpr long PROCESS_COVARIANCE_SIZE = (12);

    /**
     * @brief This is effectively the maximum number of cols/rows in the
     * state covariance. Declared as a convenience variable here as the
     * state covariance matrix is stored in a separate buffer (which can be
     * stored in e.g. DTCM). In order for the Eigen matrix map to be defined
     * properly, we define the stride for the whole matrix such that when
     * resizing the matrix when new states are augmented or marginalised,
     * they don't affect the values which are already in the matrix.
     */
    constexpr int STATE_COVARIANCE_BUFFER_STRIDE =
        (IMU_ERROR_STATE_SIZE + MAX_NUMBER_OF_AUGMENTED_STATES * 6);

    /**
     * @brief Convenience typedef for the feature tracks used in the filter
     * and the corresponding feature identifier.
     */
    typedef etl::
        map<FeatureIdentifier, FeatureTrack, MAX_NUMBER_OF_FEATURE_TRACKS>
            FeatureTrackMap;

    /**
     * @brief Convenience typedef for a vector of feature identifiers.
     */
    typedef etl::vector<FeatureIdentifier, MAX_NUMBER_OF_FEATURE_TRACKS>
        FeatureIdentifierBuffer;

    /**
     * @brief Convenience typedef for a vector of state identifiers.
     */
    typedef etl::vector<StateIdentifier, MAX_NUMBER_OF_AUGMENTED_STATES>
        StateIdentifierBuffer;

    /**
     * @brief The full state used in the filter. Stores the current IMU
     * state, the sliding window of the augmented IMU states as well as
     * estimates for the a priori state.
     */
    struct State {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief Current IMU state.
         */
        IMUState imu_state;

        /**
         * @brief A priori estimate of the current IMU state. The a priori
         * estimate is found during the prediction of the filter, and is
         * also used for the next time step of the prediction when finding
         * the transition matrix of the error-state dynamics.
         */
        IMUState imu_state_a_priori;

        /**
         * @brief Previous IMU state.
         */
        IMUState previous_imu_state;

        /**
         * @brief A priori estimate of the previous IMU state.
         */
        IMUState previous_imu_state_a_priori;

        /**
         * @brief Sliding window of added snapshots of the IMU state.
         */
        AugmentedIMUStatesMap augmented_imu_states;

        /**
         * @brief Timestamp compensation, effectively t_imu = t_camera +
         * timestamp_compensation. Timestamp of camera has to add this value
         * to synchronise with IMU timestamp.
         */
        double timestamp_compensation = 0.0;

        /**
         * @brief The process covariance for the model. In the conventional
         * Extended Kalman Filter notation this is the matrix Q.
         */
        Matrix<float, PROCESS_COVARIANCE_SIZE, PROCESS_COVARIANCE_SIZE>
            process_model_covariance = Matrix<float,
                                              PROCESS_COVARIANCE_SIZE,
                                              PROCESS_COVARIANCE_SIZE>::Zero();

        void reset() {
            imu_state.reset();
            imu_state_a_priori.reset();
            previous_imu_state.reset();
            previous_imu_state_a_priori.reset();
            augmented_imu_states.clear();
            timestamp_compensation = 0.0;
            process_model_covariance.setZero();
        }
    };

    /**
     * @brief Gravity vector.
     */
    AT_QUICKACCESS_SECTION_DATA(
        static const Vector3f gravity = Vector3f(0, 0, -9.81f));

    // ----
    // Pure functions
    //
    // These functions do not modify the state and are kept here to make
    // sure that they don't refer to any of the static variables.
    // ----

    /**
     * @brief Calculates the error-state transition matrix between a given
     * interval.
     *
     * @param state [in] The current state.
     * @param use_a_priori_estimates [in] Whether to use the a priori state
     * estimates, when calculating the transition matrix.
     * @param angular_velocity [in] The angular velocity measured in the
     * interval.
     * @param previous_angular_velocity [in] The acceleration measured in
     * the interval.
     * @param delta_time [in] The length of the interval.
     * @param out_transition_matrix [out] The transition matrix will be
     * placed in this matrix.
     */
    static void calculate_error_state_transition_matrix(
        const State& state,
        const bool use_a_priori_estimates,
        const Vector3f& angular_velocity,
        const Vector3f& previous_angular_velocity,
        const double& delta_time,
        MatrixXf& out_transition_matrix) {

        out_transition_matrix.setIdentity();

        const Vector3f theta =
            delta_time * (previous_angular_velocity + angular_velocity) / 2 +
            delta_time * delta_time *
                (previous_angular_velocity.cross(angular_velocity)) / 12;

        const Matrix3f theta_skew_symmetric = math::skew_symmetric(theta);

        const Matrix3f R_world_bk =
            state.previous_imu_state.orientation.toRotationMatrix();

        // This is the velocity for the previous state k
        const Vector3f v_k = (use_a_priori_estimates
                                  ? state.previous_imu_state_a_priori.velocity
                                  : state.previous_imu_state.velocity);

        // This is the position for the previous state k
        const Vector3f p_k = (use_a_priori_estimates
                                  ? state.previous_imu_state_a_priori.position
                                  : state.previous_imu_state.position);

        // This is the a priori velocity estimate
        const Vector3f v_a_priori = (use_a_priori_estimates
                                         ? state.imu_state_a_priori.velocity
                                         : state.imu_state.velocity);

        // This is the a priori position estimate
        const Vector3f p_a_priori = (use_a_priori_estimates
                                         ? state.imu_state_a_priori.position
                                         : state.imu_state.position);

        // --- Orientation ---

        // Term of orientation-gyro bias relationship
        out_transition_matrix.block<3, 3>(0, 9) =
            -R_world_bk * (Matrix3f::Identity() + 0.5f * theta_skew_symmetric) *
            delta_time;

        // --- Velocity ---

        // Term of velocity-orientation relationship
        out_transition_matrix.block<3, 3>(3, 0) = -math::skew_symmetric(
            v_a_priori - v_k - gravity * delta_time);

        // Term of velocity-gyro bias relationship
        out_transition_matrix.block<3, 3>(3, 9) =
            math::skew_symmetric(-p_a_priori + p_k + v_a_priori * delta_time -
                                 0.5 * gravity * delta_time * delta_time) *
                R_world_bk +
            math::skew_symmetric(-0.5 * p_a_priori + 0.5 * p_k +
                                 0.5 * v_a_priori * delta_time -
                                 gravity * delta_time * delta_time / 6.0f) *
                R_world_bk * theta_skew_symmetric;

        // Term of velocity-accelerometer bias relationship
        out_transition_matrix.block<3, 3>(3, 12) =
            -R_world_bk * (Matrix3f::Identity() + 0.5f * theta_skew_symmetric) *
            delta_time;

        // --- Position ---

        // Term of position-orientation relationship
        out_transition_matrix.block<3, 3>(6, 0) = -math::skew_symmetric(
            p_a_priori - p_k - v_k * delta_time -
            0.5 * gravity * delta_time * delta_time);

        // Term of position-velocity relationship
        out_transition_matrix.block<3, 3>(6, 3) = Matrix3f::Identity() *
                                                  delta_time;

        // Term of position-gyro bias relationship
        out_transition_matrix.block<3, 3>(6, 9) =
            -math::skew_symmetric(gravity * delta_time * delta_time *
                                  delta_time / 6.0f) *
                R_world_bk +
            delta_time *
                math::skew_symmetric(p_a_priori - p_k -
                                     gravity * delta_time * delta_time / 6.0f) *
                R_world_bk * theta_skew_symmetric / 4.0f;

        // Term of position-accelerometer bias relationship
        out_transition_matrix.block<3, 3>(6, 12) =
            -R_world_bk * (3 * Matrix3f::Identity() + theta_skew_symmetric) *
            delta_time * delta_time / 6.0f;
    }

    /**
     * @brief Finds the augment IMU states which have no associated
     * features.
     *
     * @param state[in] The state of the filter, has the augmented IMU
     * states.
     * @param feature_track_map[in] The feature track map, which keeps all
     * the features which we test against.
     * @param out_unused_state_identifiers[out] The unused states will be
     * placed in this vector.
     */
    static void
    find_unused_states(const State& state,
                       const FeatureTrackMap& feature_track_map,
                       StateIdentifierBuffer& out_unused_state_identifiers) {

        for (auto iterator = state.augmented_imu_states.begin();
             iterator != state.augmented_imu_states.end();
             iterator++) {
            out_unused_state_identifiers.push_back(iterator->first);
        }

        for (auto iterator = feature_track_map.begin();
             iterator != feature_track_map.end();
             iterator++) {
            const FeatureTrack& feature = iterator->second;

            for (auto state_iterator = feature.observations.begin();
                 state_iterator != feature.observations.end();
                 state_iterator++) {
                const auto& unused_states_iterator = etl::find(
                    out_unused_state_identifiers.begin(),
                    out_unused_state_identifiers.end(),
                    state_iterator->first);

                if (unused_states_iterator !=
                    out_unused_state_identifiers.end()) {
                    out_unused_state_identifiers.erase(unused_states_iterator);
                }
            }
        }

        etl::sort(out_unused_state_identifiers.begin(),
                  out_unused_state_identifiers.end());
    }

    // ----
    // End of pure functions
    // ----

    /**
     * @brief Configuration parameters, set on initialisation.
     */
    static Configuration configuration;

    /**
     * @brief The full state of the filter.
     */
    AT_QUICKACCESS_SECTION_DATA(static State state);

    /**
     * @brief Holds the identifier of the next state.
     */
    AT_QUICKACCESS_SECTION_DATA(static StateIdentifier next_identifier = 0);

    /**
     * @brief The buffer for the state covariance.
     */
    AT_QUICKACCESS_SECTION_DATA(
        static float state_covariance_buffer[STATE_COVARIANCE_BUFFER_STRIDE *
                                             STATE_COVARIANCE_BUFFER_STRIDE]);

    /**
     * @brief Rows in the state covariance.
     */
    AT_QUICKACCESS_SECTION_DATA(
        static int state_covariance_rows = IMU_ERROR_STATE_SIZE);

    /**
     * @brief Columns in the state covariance.
     */
    AT_QUICKACCESS_SECTION_DATA(
        static int state_covariance_cols = IMU_ERROR_STATE_SIZE);

    /**
     * @brief The currently tracked features in the pipeline.
     */
    static FeatureTrackMap feature_track_map;

    /**
     * @brief Is set when the initialisation of the filter succeeds.
     */
    AT_QUICKACCESS_SECTION_DATA(static bool initialised = false);

    /**
     * @brief The number of tracked features in the filter.
     */
    AT_QUICKACCESS_SECTION_DATA(static double tracking_rate = 0.0);

    /**
     * @brief Threshold for deciding which IMU measurement is associated
     * with which image frame.
     */
    AT_QUICKACCESS_SECTION_DATA(static double frame_threshold = 0.0);

    /**
     * @brief The previous measured angular velocity, used when calculating
     * the error state transition matrix.
     */
    AT_QUICKACCESS_SECTION_DATA(
        static Vector3f previous_measured_angular_velocity);

    /**
     * @brief Set when the agent is stationary and we should swap the filter
     * dynamics.
     */
    AT_QUICKACCESS_SECTION_DATA(static bool currently_stationary = false);

    /**
     * @brief Set to true when the first features are passed to the filter.
     */
    AT_QUICKACCESS_SECTION_DATA(static bool got_features = false);

    /**
     * @brief This is solely for debugging purposes.
     */
    static BackendInformation backend_information;

    /**
     * @brief Attempts initialisation of the system. Uses the @p
     * imu_measurements and @p feature_observations to do static initialisation.
     *
     * @param imu_measurements [in] The IMU measurements.
     * @param feature_observations [in] The feature observations made in the
     * current frame.
     * @param out_initial_orientation [out] The initial orientation is placed in
     * this reference if the initialisation succeeds.
     * @param out_initial_angular_velocity_bias [out] The initial angular
     * velocity bias is placed in this reference if the initialisation succeeds.
     * @param out_initialisation_timestamp [out] The timestamp when the
     * initialisation occured.
     * @param out_initialisation_index [out] Represents the index of the last
     * measurement used for the initialisation. Is set if the initialisation
     * succeeds.
     *
     * @return True if the initialisation succeeded.
     */
    static bool
    initialisation_succeeds(const IMUBuffer& imu_measurements,
                            const FeatureObservations& feature_observations,
                            Quaternionf& out_initial_orientation,
                            Vector3f& out_initial_angular_velocity_bias,
                            double& out_initialisation_timestamp,
                            size_t& out_initialisation_index);

    /**
     * @brief Processes the IMU measurements up to @p time_bound,
     * propagating the nominal state and the state covariance.
     *
     * @param[in, out] imu_measurements The IMU measurements.
     * @param[in] time_bound Any IMU measurement past this value will not be
     * processed.
     */
    static void propagate_state(IMUBuffer& imu_measurements,
                                const double& time_bound);

    /**
     * @brief Augments the state with the current IMU state. Stores a
     * "snapshot" of the IMU state which is later used to perform the
     * measurement update.
     */
    static void augment_state();

    /**
     * @brief Adds the feature observations from the frontend to the current
     * feature tracks
     *
     * @param feature_observations[in] The feature observations to add.
     */
    static void
    add_feature_observations(const FeatureObservations& feature_observations);

    /**
     * @brief Removes augmented IMU states from the state.
     *
     * @param state_identifiers_to_remove[in] The states to remove.
     */
    static void remove_entries_from_state(
        const StateIdentifierBuffer& state_identifiers_to_remove);

    /**
     * @brief Uses the feature tracks to check if the agent is
     * stationary. Evaluated if the distances are lower than a given
     * threshold.
     *
     * @return True if the agent is stationary.
     */
    static bool is_stationary();

    /**
     * @brief Finds the features which are ready for an update step in the
     * filter.
     *
     * @param out_invalid_features [out] Feature identifiers which we've lost
     * track of an either does not have enough observations for triangulation or
     * failed triangulation.
     * @param out_features_ready_for_update [out] Feature identifiers which are
     * ready for an update in the filter.
     */
    static void find_feature_tracks_ready_for_update(
        FeatureIdentifierBuffer& out_invalid_features,
        FeatureIdentifierBuffer& out_features_ready_for_update);

    /**
     * @brief Performs an update of the filter with a set of @p features.
     *
     * @param features [in] The features to perform the update with.
     */
    static void update_with_features(const FeatureIdentifierBuffer& features);

    /**
     * @brief Forces an update of the filter with the given @p states and their
     * associated features
     *
     * @param states [in] The states which are used in the update.
     */
    static void force_update_with_states(const StateIdentifierBuffer& states);

    /**
     * @brief Calculates a single observation jacobian and residual for a
     * particular @p state_identifier and a given @p feature.
     *
     * @param feature [in] The feature to calculate the
     * jacobian for.
     * @param state_identifier [in] The state identifier to calculate the
     * jacobian for.
     * @param out_H_x [out] The jacobian with respect to the state (except
     * the extrinsics will be placed in this matrix).
     * @param out_H_extrinsics [out] The jacobian with respect to the
     * IMU-camera extrinsics will be placed in this matrix.
     * @param out_H_feature [out] The jacobian with respect to the feature
     * position will be placed in this matrix.
     * @param out_residual [out] the residual from the estimated feature
     * position and the observed feature position will be placed in this
     * vector.
     */
    static void
    calculate_observation_jacobian(const FeatureTrack& feature,
                                   const StateIdentifier& state_identifier,
                                   Matrix<float, 2, 6>& out_H_x,
                                   Matrix<float, 2, 6>& out_H_extrinsics,
                                   Matrix<float, 2, 3>& out_H_feature,
                                   Vector2f& out_residual);

    /**
     * @brief Calculates the jacobian and residual for a feature with the
     * observations seen in @p state_identifiers.
     *
     * @param feature [in] The feature.
     * @param state_identifiers [in] The augmented IMU states where the
     * feature was observed.
     * @param out_H [out] The jacobian will be placed in this matrix.
     * @param out_r [out] The residual will be placed in this matrix.
     */
    static void
    calculate_feature_jacobian(const FeatureTrack& feature,
                               const StateIdentifierBuffer& state_identifiers,
                               MatrixXf& out_H,
                               VectorXf& out_r);

    /**
     * @brief Performs the filter update.
     *
     * @param H The jacobian relating the error-state to the residuals.
     * @param r The residuals from the measurement and the predicted
     * measurements.
     */
    static void measurement_update(MatrixXf& H, VectorXf& r);

    /**
     * @brief Measurement update for the filter when the agent is
     * stationary.
     */
    static void stationary_update();

    /**
     * @brief Performs a Mahalanobis test based on the residuals @p r for
     * the filter update and the corresponding jacobian @p H. Effectively
     * calculates the squared Mahalanobis distance, which should be
     * Chi-Squared distributed, and tests the observations and the estimate
     * observations (which are encoded in the residual) against a 95%
     * percentile for the Chi-Squared distribution with the same
     * degrees-of-freedom @p dof.
     *
     * @param H The jacobian used in the filter update.
     * @param r The residuals used in the filter update.
     * @param dof The degrees of freedom for the Chi-Squared 95% percentile
     * to compare the squared mahalanobis distance against.
     *
     * @return True if the test was passed.
     */
    static bool succeeds_mahalanobis_test(const MatrixXf& H,
                                          const VectorXf& r,
                                          const int& dof);

    /**
     * @brief Finds two redundant IMU states (states without much movement).
     *
     * @param out_redundant_augmented_imu_states The state identifiers which
     * are redundant.
     */
    static void find_redundant_augmented_imu_states(
        StateIdentifierBuffer& out_redundant_augmented_imu_states);

    /**
     * @brief Attempts to find a new anchor state for @p feature.
     *
     * @param excluded_states [in] List of states to not use when searching for
     * the new anchor state.
     */
    static StateIdentifier
    get_new_anchor_identifier(FeatureTrack& feature,
                              const StateIdentifierBuffer& excluded_states);

    void configure(const Configuration& configuration_) {

        configuration = configuration_;

        // --- State ---

        state.reset();

        state.process_model_covariance.block<3, 3>(0, 0) =
            Matrix3f::Identity() * configuration.noise_gyro;
        state.process_model_covariance.block<3, 3>(3, 3) =
            Matrix3f::Identity() * configuration.noise_accelerometer;
        state.process_model_covariance.block<3, 3>(6, 6) =
            Matrix3f::Identity() * configuration.noise_gyro_bias;
        state.process_model_covariance.block<3, 3>(9, 9) =
            Matrix3f::Identity() * configuration.noise_accelerometer_bias;

        state.imu_state.T_camera_imu.linear() =
            configuration.T_camera_imu.linear();

        // Need to do the opposite here as the configuration file is the
        // other way around when defining the translation
        state.imu_state.T_camera_imu.translation() =
            configuration.T_imu_camera.translation();

        state.timestamp_compensation =
            configuration.initial_timestamp_compensation;

        next_identifier = 0;

        // --- State covariance ---

        // Fill the state covariance buffer with zero before configuring
        memset(state_covariance_buffer, 0, sizeof(state_covariance_buffer));

        state_covariance_rows = IMU_ERROR_STATE_SIZE;
        state_covariance_cols = IMU_ERROR_STATE_SIZE;

        Map<MatrixXf, 0, OuterStride<>> state_covariance(
            state_covariance_buffer,
            state_covariance_rows,
            state_covariance_cols,
            OuterStride<>(STATE_COVARIANCE_BUFFER_STRIDE));

        for (int i = 0; i < 3; ++i) {
            state_covariance(i,
                             i) = configuration.initial_covariance_orientation;
        }

        for (int i = 3; i < 6; ++i) {
            state_covariance(i, i) = configuration.initial_covariance_velocity;
        }

        for (int i = 6; i < 9; ++i) {
            state_covariance(i, i) = configuration.initial_covariance_position;
        }

        for (int i = 9; i < 12; ++i) {
            state_covariance(i, i) = configuration.initial_covariance_gyro_bias;
        }

        for (int i = 12; i < 15; ++i) {
            state_covariance(i, i) =
                configuration.initial_covariance_accelerometer_bias;
        }

        if (configuration.estimate_extrinsics) {
            for (int i = 15; i < 18; ++i) {
                state_covariance(i, i) =
                    configuration.initial_covariance_extrinsics_rotation;
            }
            for (int i = 18; i < 21; ++i) {
                state_covariance(i, i) =
                    configuration.initial_covariance_extrinsics_translation;
            }
        }

        if (configuration.estimate_time_compensation) {
            state_covariance(21, 21) = 4e-6;
        }

        // --- Various other variables ---
        feature_track_map.clear();

        initialised = false;

        tracking_rate = 0.0;

        frame_threshold = 1.0 / (2.0 * configuration.imu_rate);

        previous_measured_angular_velocity.setZero();

        currently_stationary = false;

        got_features = false;

        FeatureTrack::optimisation_config.translation_threshold =
            configuration.feature_translation_threshold;

        static_initialiser::reset(
            configuration.zero_update_max_feature_distance,
            (int)(configuration.static_duration *
                  (float)configuration.update_frequency),
            state.timestamp_compensation);
    }

    bool has_initialised() { return initialised; }

    bool process_feature_observations(
        const FeatureObservations& feature_observations,
        IMUBuffer& imu_measurements) {

        // Features are not utilised until receiving IMU measurements which are
        // ahead of the feature timestamps
        if (!got_features) {
            if (!imu_measurements.empty() &&
                (imu_measurements.begin()->timestamp -
                     feature_observations.timestamp -
                     state.timestamp_compensation <=
                 0.0)) {
                got_features = true;
            } else {
                return false;
            }
        }

        if (!has_initialised()) {

            Quaternionf initial_orientation;
            Vector3f initial_angular_velocity_bias;
            size_t initialisation_index     = 0;
            double initialisation_timestamp = 0.0;

            if (!initialisation_succeeds(imu_measurements,
                                         feature_observations,
                                         initial_orientation,
                                         initial_angular_velocity_bias,
                                         initialisation_timestamp,
                                         initialisation_index)) {
                return false;
            }

            initialised = true;

            // Make sure to keep track of the last measured angular velocity
            // before we erase the measurements associate with the
            // initialisation
            previous_measured_angular_velocity =
                imu_measurements[initialisation_index].angular_velocity;

            // Erase the measurements used for initialisation
            imu_measurements.erase(imu_measurements.begin(),
                                   imu_measurements.begin() +
                                       initialisation_index);

            // Update the state with the initialisation
            state.imu_state.timestamp = initialisation_timestamp;
            state.imu_state.angular_velocity_bias =
                initial_angular_velocity_bias;
            state.imu_state.orientation = initial_orientation;
            state.imu_state_a_priori    = state.imu_state;
        }

        profile::start("backend:propagate_state");

        propagate_state(imu_measurements,
                        feature_observations.timestamp +
                            state.timestamp_compensation);

        profile::end();

        profile::start("backend:augment_state");
        augment_state();
        profile::end();

        profile::start("backend:add_feature_observations");
        //  Add new observations for existing features or new
        //  features in the map server.
        add_feature_observations(feature_observations);
        profile::end();

        profile::start("backend:remove_unused_states");
        StateIdentifierBuffer unused_augmented_imu_states;

        find_unused_states(state,
                           feature_track_map,
                           unused_augmented_imu_states);
        remove_entries_from_state(unused_augmented_imu_states);
        profile::end();

        profile::start("backend:stationary_detection");

        currently_stationary = is_stationary();

        if (currently_stationary) {
            stationary_update();
        }

        profile::end();

        // --- Perform update with features tracks ---

        profile::start("backend:feature_update");

        static FeatureIdentifierBuffer invalid_features,
            features_ready_for_update;

        find_feature_tracks_ready_for_update(invalid_features,
                                             features_ready_for_update);

        // Simply erase the features which have lost track and either:
        //
        // 1. Does not have enough observations to be triangulated
        // 2. Failed triangulation
        for (const FeatureIdentifier& identifier : invalid_features) {
            feature_track_map.erase(identifier);
        }

        backend_information.measurement_update_features =
            !features_ready_for_update.empty();

        if (!features_ready_for_update.empty()) {

            if (!currently_stationary) {
                update_with_features(features_ready_for_update);
            }

            // Erase the features used with the update
            for (const FeatureIdentifier& identifier :
                 features_ready_for_update) {
                feature_track_map.erase(identifier);
            }
        }

        profile::end();

        // ---
        // Remove augmented IMU states if the state vector is at maximum
        // capacity
        // ---

        profile::start("backend:prune_imu_states");

        static StateIdentifierBuffer states_to_remove;
        states_to_remove.clear();

        // Remove the last state if we are stationary, since during the
        // stationary update, we're oly concerned with the current and the
        // previous before the stationary detection either way
        if (currently_stationary) {
            states_to_remove.push_back(state.imu_state.identifier - 1);
        } else {

            if (state.augmented_imu_states.size() ==
                MAX_NUMBER_OF_AUGMENTED_STATES) {
                find_redundant_augmented_imu_states(states_to_remove);
            }
        }

        backend_information.measurement_update_pruning_states =
            !states_to_remove.empty();

        if (!states_to_remove.empty()) {
            force_update_with_states(states_to_remove);
            remove_entries_from_state(states_to_remove);
        }

        profile::end();

        backend_information.initialised   = has_initialised();
        backend_information.is_stationary = currently_stationary;

        return true;
    }

    IMUState get_imu_state() { return state.imu_state; }

    Matrix<float, IMU_ERROR_STATE_SIZE, IMU_ERROR_STATE_SIZE>
    get_imu_state_covariance() {

        Map<MatrixXf, 0, OuterStride<>> state_covariance(
            state_covariance_buffer,
            state_covariance_rows,
            state_covariance_cols,
            OuterStride<>(STATE_COVARIANCE_BUFFER_STRIDE));

        Matrix<float, IMU_ERROR_STATE_SIZE, IMU_ERROR_STATE_SIZE> P =
            state_covariance.block(0,
                                   0,
                                   IMU_ERROR_STATE_SIZE,
                                   IMU_ERROR_STATE_SIZE);

        return P;
    }

    etl::vector<FeatureTrack, MAX_NUMBER_OF_FEATURE_TRACKS>
    get_feature_tracks() {

        etl::vector<FeatureTrack, MAX_NUMBER_OF_FEATURE_TRACKS> feature_tracks;

        for (const auto& feature_track : feature_track_map) {
            feature_tracks.push_back(feature_track.second);
        }

        return feature_tracks;
    }

    BackendInformation get_backend_information() { return backend_information; }

    static bool
    initialisation_succeeds(const IMUBuffer& imu_measurements,
                            const FeatureObservations& feature_observations,
                            Quaternionf& out_initial_orientation,
                            Vector3f& out_initial_angular_velocity_bias,
                            double& out_initialisation_timestamp,
                            size_t& out_initialisation_index) {

        if (!static_initialiser::attempt_initialisation(
                imu_measurements,
                feature_observations,
                out_initialisation_timestamp,
                out_initial_angular_velocity_bias,
                out_initial_orientation)) {

            return false;
        }

        // Move the initialisation index to the last IMU measurements used for
        // initialisation
        for (const ImuDataPoint& measurement : imu_measurements) {
            if (measurement.timestamp > out_initialisation_timestamp) {
                break;
            }

            out_initialisation_index++;
        }

        // If we use all the measurements, we correct for 0 indexing
        if (out_initialisation_index >= imu_measurements.size()) {
            out_initialisation_index = imu_measurements.size() - 1;
        }

        return true;
    }

    static void propagate_state(IMUBuffer& imu_measurements,
                                const double& time_bound) {

        size_t processed_imu_measurements = 0;

        double interval_to_nearest_image = 0.0;

        MatrixXf total_transition_matrix =
            MatrixXf::Identity(IMU_ERROR_STATE_SIZE, IMU_ERROR_STATE_SIZE);

        MatrixXf initial_G = MatrixXf::Zero(IMU_ERROR_STATE_SIZE, 12);

        double total_delta_time = 0.0;

        // Go through every measurement and propagate the nominal state. We
        // keep track of the transition matrices for the error-state between
        // every propagation, as well as the associated process covariance.
        // This is then applied at the end, so that we omit a lot of matrix
        // multiplication
        //
        // We can do this since the transition matrix has this behaviour:
        //
        // clang-format off
        //
        // Phi(t_{k+n}, t_k) = Phi(t_{k+n}, t_{k+(n-1)}) Phi(t_k{k+(n-1)}, t_{k+(n-2)}) .. Phi(t_{k+(n-(n-1), t_k)})
        //
        // For the process noise covariance we have the following, we:
        //
        //
        // Q = int_{t_k}^{t_{k+1}} Phi(tau, t_k) G(tau) w G(tau)^T Phi(tau, t_k)^T dtau
        //   ~= Phi(t_{k+1}, t_k) G(t_k) w G(t_k)^T Phi(t_{k+1}, t_k)^T delta t
        //   ~= Phi(t_{k+n}, t_k) G(t_k) w G(t_k)^T Phi(t_{k+n}, t_k)^T (delta t) * n
        //
        // clang-format on

        bool is_first_propagated_measurement = true;

        for (const auto& measurement : imu_measurements) {

            const double timestamp = measurement.timestamp;

            // If the timestamp is behind the current IMU state, we
            // disregard doing any more processing with it since we are
            // further ahead in time
            if (timestamp <= state.imu_state.timestamp) {
                processed_imu_measurements++;
                continue;
            }

            // If the measurement is beyond what we want to associate with
            // this camera frame, we finish up
            if (timestamp - time_bound > frame_threshold) {
                break;
            }

            interval_to_nearest_image = timestamp - time_bound;

            const Vector3f measured_angular_velocity =
                measurement.angular_velocity;
            const Vector3f measured_acceleration =
                measurement.linear_acceleration;

            const IMUState& imu_state = state.imu_state;

            const Vector3f acceleration_unbiased = measured_acceleration -
                                                   imu_state.acceleration_bias;

            const Vector3f angular_velocity_unbiased =
                measured_angular_velocity - imu_state.angular_velocity_bias;

            const Vector3f previous_angular_velocity_unbiased =
                previous_measured_angular_velocity -
                imu_state.angular_velocity_bias;

            const double delta_time = timestamp - imu_state.timestamp;

            // ------------------------- Nominal state -------------------------

            const float angular_velocity_unbiased_norm =
                angular_velocity_unbiased.norm();

            const Vector3f angular_velocity_unbiased_normalized =
                angular_velocity_unbiased / angular_velocity_unbiased_norm;

            // Set the old IMU state before it's updated
            state.previous_imu_state = state.imu_state;

            Quaternionf& q = state.imu_state.orientation;
            Vector3f& v    = state.imu_state.velocity;
            Vector3f& p    = state.imu_state.position;

            // clang-format off
            //
            // For Runga-Kutta we need the orientation at the next state and halfway to the next step.
            //
            // This follows the work by Solà in Quaternion kinematics for the error-state Kalman filter. In particular, 
            // this is from chapter 4.6.
            //
            // We have that q_dot = 0.5 * q * w, where w is the angular velocity
            //
            // The discrete Taylor series is given by:
            //
            // q_n+1 = q_n + q_n_dot * dt + (1/2!) * q_n_ddot * dt^2 + (1/3!) * q_n_dddot * dt^3 + ...
            //
            // The quaternion derivatives are given by (utilising the product rule):
            //
            // q_n_dot      = 0.5 * q_n * w_n
            // q_n_ddot     = (0.5^2) * q_n * w_n^2 + 0.5 * q_n * w_dot
            // q_n_dddot    = (0.5^3) * q_n * w_n^3 + 0.25 * q_n * w_dot * w_n + 0.5 * q_n * w_n * w_dot
            //
            // When then we assume that the angular rate is kept constant between time point n and n+1, such that w_dot = 0, we have that:
            //
            // q_n_dot      = 0.5 * q_n * w_n
            // q_n_ddot     = (0.5^2) * q_n * w_n^2 
            // q_n_dddot    = (0.5^3) * q_n * w_n^3 
            //
            // Yielding:
            //
            // q_n+1 = q_n * (1 + (0.5 * w_n) * dt + (1/2!) * ((0.5^2) * w_n^2) * dt^2 + (1/3!) * ((0.5^3) * w_n^3) * dt^3 + ...) 
            //
            // Where we have that the series is the exponential of w_n * dt / 2 (equation 40 in Solà's paper):
            //
            // Exp(w_n * dt / 2) = (1 + (0.5 * w_n * dt) + (1/2!) * (0.5 * w_n * dt)^2 + (1/3!) * (0.5 * w_n * dt)^3 + ...) 
            //
            // This again, following equation 42, is equivalent to the quaternion given by:
            //
            //      [          cos(|w_n| * dt / 2) ]
            // q* = [   w_n  * sin(|w_n| * dt / 2) ]
            //      [ -------                      ]
            //      [ | w_n |                      ] 
            //      
            //
            // Thus, q* — when pre multiplied with q_n — propagates the orientation. 
            //
            // clang-format on

            Quaternionf delta_q_whole_step, delta_q_half_step;

            // --- Whole step ---

            delta_q_whole_step.w() = cos(angular_velocity_unbiased_norm *
                                         delta_time * 0.5f);

            delta_q_whole_step.x() = angular_velocity_unbiased_normalized.x() *
                                     sin(angular_velocity_unbiased_norm *
                                         delta_time * 0.5f);

            delta_q_whole_step.y() = angular_velocity_unbiased_normalized.y() *
                                     sin(angular_velocity_unbiased_norm *
                                         delta_time * 0.5f);

            delta_q_whole_step.z() = angular_velocity_unbiased_normalized.z() *
                                     sin(angular_velocity_unbiased_norm *
                                         delta_time * 0.5f);

            // --- Half step ---
            //
            // This is simply the above, only with the step in time equal to dt
            // / 2

            delta_q_half_step.w() = cos(angular_velocity_unbiased_norm *
                                        delta_time * 0.25f);

            delta_q_half_step.x() = angular_velocity_unbiased_normalized.x() *
                                    sin(angular_velocity_unbiased_norm *
                                        delta_time * 0.25f);

            delta_q_half_step.y() = angular_velocity_unbiased_normalized.y() *
                                    sin(angular_velocity_unbiased_norm *
                                        delta_time * 0.25f);

            delta_q_half_step.z() = angular_velocity_unbiased_normalized.z() *
                                    sin(angular_velocity_unbiased_norm *
                                        delta_time * 0.25f);

            const Quaternionf q_whole_step = q * delta_q_whole_step;
            const Quaternionf q_half_step  = q * delta_q_half_step;

            // Finally create the rotation matrices taking us from the current
            // frame whole/half the way to the next frame
            Matrix3f R_whole_step = q_whole_step.toRotationMatrix();
            Matrix3f R_half_step  = q_half_step.toRotationMatrix();

            // --- Runga-Kutta integration ---
            //
            // We want to do numerical integration to find the state estimate at
            // time t_{k+1}. We have the ODE x_dot = f(t, x).
            //
            // Runga-Kutta yields that:
            //
            // x(t_{k+1}) = x(t_k) + (dt/6) * (k_1 + 2 * k_2 + 2 * k_3 + k_4),
            // where:
            //
            // k_1 = f(t_k, x(t_k))
            // k_2 = f(t_k + dt / 2, x(t_k) + dt * k_1 / 2)
            // k_3 = f(t_k + dt / 2, x(t_k) + dt * k_2 / 2)
            // k_4 = f(t_k + dt, x(t_k) + dt * k_3)

            // --- K1 ---
            //
            // k_1 = f(t_k, x_k)
            //
            // The derivative at the current point in time given the state at
            // the current time
            const Vector3f k1_v_dot =
                q.toRotationMatrix() * acceleration_unbiased + gravity;

            const Vector3f k1_p_dot = v;

            // --- K2 ---
            //
            // k_2 = f(t_k + dt / 2, x(t_k) + dt * k_1 / 2)
            //
            // The derivative half a step forward, utilising K1

            const Vector3f k2_v_dot = R_half_step * acceleration_unbiased +
                                      gravity;
            const Vector3f k2_p_dot = v + k1_v_dot * delta_time / 2;

            // --- K3 ---
            //
            // k_3 = f(t_k + dt / 2, x(t_k) + dt * k_2 / 2)
            //
            // The derivative half a step forward, utilising K2

            const Vector3f k3_v_dot = R_half_step * acceleration_unbiased +
                                      gravity;

            const Vector3f k3_p_dot = v + k2_v_dot * delta_time / 2;

            // --- K4 ---
            //
            // k_4 = f(t_k + dt, x(t_k) + dt * k_3)
            //
            // The derivative a whole step forward, utilising K3

            const Vector3f k4_v_dot = R_whole_step * acceleration_unbiased +
                                      gravity;

            const Vector3f k4_p_dot = v + k3_v_dot * delta_time;

            // Do the numerical integration
            //
            // x(t_{k+1}) = x(t_k) + (dt/6) * (k_1 + 2 * k_2 + 2 * k_3 + k_4),
            v = v + delta_time / 6 *
                        (k1_v_dot + 2 * k2_v_dot + 2 * k3_v_dot + k4_v_dot);
            p = p + delta_time / 6 *
                        (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot);

            q = q_whole_step;

            // Update first estimate jacobians for previous and current state
            state.previous_imu_state_a_priori = state.imu_state_a_priori;
            state.imu_state_a_priori          = state.imu_state;

            // ------------ Transition matrix & process covariance -------------

            MatrixXf transition_matrix(IMU_ERROR_STATE_SIZE,
                                       IMU_ERROR_STATE_SIZE);

            calculate_error_state_transition_matrix(
                state,
                initialised,
                angular_velocity_unbiased,
                previous_angular_velocity_unbiased,
                delta_time,
                transition_matrix);

            // Need to find the first matrix G in the differential equation
            // for the error-state given by: delta x = F delta x + G w as
            // the noise term will consist of G.

            // Note here that we do not include the entries related to the
            // camera-IMU extrinsics or the time offset between the IMU
            // measurements and the camera frames, as all those entries are
            // 0 and kept static anyway

            if (is_first_propagated_measurement) {

                const Matrix3f R_world_bk =
                    state.previous_imu_state.orientation.toRotationMatrix();

                initial_G.block<3, 3>(0, 0)  = -R_world_bk;
                initial_G.block<3, 3>(3, 3)  = -R_world_bk;
                initial_G.block<3, 3>(9, 6)  = Matrix3f::Identity();
                initial_G.block<3, 3>(12, 9) = Matrix3f::Identity();

                is_first_propagated_measurement = false;
            }

            total_transition_matrix *= transition_matrix;

            total_delta_time += delta_time;

            state.imu_state.timestamp          = timestamp;
            state.imu_state_a_priori.timestamp = timestamp;

            processed_imu_measurements++;

            previous_measured_angular_velocity = measured_angular_velocity;
        }

        // Now we can predict the a priori covariance by means of:
        //
        //             [ Phi P_II_k Phi^T + Q      Phi * P_IA_k ]
        // P_{k+1}^- = [                                        ]
        //             [ P_AI_k  Phi^T             P_AA_k       ]
        //
        // Do note that the a priori estimate for the augmented IMU states
        // are kept static (we don't do anything with P_AA_k)
        Map<MatrixXf, 0, OuterStride<>> state_covariance(
            state_covariance_buffer,
            state_covariance_rows,
            state_covariance_cols,
            OuterStride<>(STATE_COVARIANCE_BUFFER_STRIDE));

        // Approximated process model covariance matrix
        MatrixXf Q = total_transition_matrix * initial_G *
                     state.process_model_covariance * initial_G.transpose() *
                     total_transition_matrix.transpose() * total_delta_time;

        state_covariance.block(0,
                               0,
                               IMU_ERROR_STATE_SIZE,
                               IMU_ERROR_STATE_SIZE) =
            total_transition_matrix *
                state_covariance.block(0,
                                       0,
                                       IMU_ERROR_STATE_SIZE,
                                       IMU_ERROR_STATE_SIZE) *
                total_transition_matrix.transpose() +
            Q;

        if (state_covariance.cols() > IMU_ERROR_STATE_SIZE) {
            state_covariance.block(0,
                                   IMU_ERROR_STATE_SIZE,
                                   IMU_ERROR_STATE_SIZE,
                                   state_covariance.cols() -
                                       IMU_ERROR_STATE_SIZE) =
                total_transition_matrix *
                state_covariance.block(0,
                                       IMU_ERROR_STATE_SIZE,
                                       IMU_ERROR_STATE_SIZE,
                                       state_covariance.cols() -
                                           IMU_ERROR_STATE_SIZE);

            state_covariance.block(IMU_ERROR_STATE_SIZE,
                                   0,
                                   state_covariance.rows() -
                                       IMU_ERROR_STATE_SIZE,
                                   IMU_ERROR_STATE_SIZE) =
                state_covariance.block(IMU_ERROR_STATE_SIZE,
                                       0,
                                       state_covariance.rows() -
                                           IMU_ERROR_STATE_SIZE,
                                       IMU_ERROR_STATE_SIZE) *
                total_transition_matrix.transpose();
        }

        // Make covariance matrix symmetric
        state_covariance =
            ((state_covariance + state_covariance.transpose()) / 2.0).eval();

        state.imu_state.identifier = next_identifier++;
        state.imu_state.time_interval_to_nearest_image =
            interval_to_nearest_image;

        imu_measurements.erase(imu_measurements.begin(),
                               imu_measurements.begin() +
                                   processed_imu_measurements);
    }

    static void augment_state() {

        const Matrix3f& R_camera_imu = state.imu_state.T_camera_imu.linear();
        const Vector3f& t_camera_imu =
            state.imu_state.T_camera_imu.translation();

        const Matrix3f R_world_imu =
            state.imu_state.orientation.toRotationMatrix();

        const Vector3f t_camera_world = state.imu_state.position +
                                        R_world_imu * t_camera_imu;

        // Add a snapshot of the IMU state at the current identifier/frame
        AugmentedIMUState imu_state_snapshot;

        imu_state_snapshot.timestamp = state.imu_state.timestamp;
        imu_state_snapshot.time_interval_to_nearest_image =
            state.imu_state.time_interval_to_nearest_image;
        imu_state_snapshot.orientation  = state.imu_state.orientation;
        imu_state_snapshot.position     = state.imu_state.position;
        imu_state_snapshot.position_FEJ = state.imu_state_a_priori.position;

        imu_state_snapshot.T_camera_imu.linear()      = R_camera_imu;
        imu_state_snapshot.T_camera_imu.translation() = t_camera_imu;

        imu_state_snapshot.T_world_camera.linear() = R_world_imu *
                                                     R_camera_imu.transpose();
        imu_state_snapshot.T_world_camera.translation() = t_camera_world;

        state.augmented_imu_states.insert(
            etl::make_pair(state.imu_state.identifier, imu_state_snapshot));

        Map<MatrixXf, 0, OuterStride<>> state_covariance(
            state_covariance_buffer,
            state_covariance_rows,
            state_covariance_cols,
            OuterStride<>(STATE_COVARIANCE_BUFFER_STRIDE));

        // Update the covariance matrix of the state with the augmented
        // state
        MatrixXf J          = MatrixXf::Zero(6, state_covariance.rows());
        J.block(0, 0, 3, 3) = Matrix3f::Identity();
        J.block(3, 6, 3, 3) = Matrix3f::Identity();

        // We have that the final covariance matrix should be:
        //
        //           [ I ]                 [ P_k      P_k J^T  ]
        // P_{k+1} = [   ] P_k [ I J^T ] = [                   ]
        //           [ J ]                 [ J P_k   J P_k J^T ]
        //
        // Where P is symmetric, so (P_k J^T)^T = J P_k

        const MatrixXf JP   = J * state_covariance;
        const MatrixXf JPJT = JP * J.transpose();

        const long augmented_state_row_index    = state_covariance.rows();
        const long augmented_state_column_index = state_covariance.cols();

        // Resize the state covariance matrix for the augmented state
        state_covariance_rows += 6;
        state_covariance_cols += 6;

        Map<MatrixXf, 0, OuterStride<>> augmented_state_covariance(
            state_covariance_buffer,
            state_covariance_rows,
            state_covariance_cols,
            OuterStride<>(STATE_COVARIANCE_BUFFER_STRIDE));

        augmented_state_covariance.block(augmented_state_row_index,
                                         augmented_state_column_index,
                                         6,
                                         6) = JPJT;

        augmented_state_covariance.block(augmented_state_row_index,
                                         0,
                                         6,
                                         augmented_state_column_index) = JP;

        augmented_state_covariance.block(0,
                                         augmented_state_column_index,
                                         augmented_state_row_index,
                                         6) = JP.transpose();

        const MatrixXf augmented_state_covariance_symmetric =
            (augmented_state_covariance +
             augmented_state_covariance.transpose()) /
            2.0;

        augmented_state_covariance = augmented_state_covariance_symmetric;
    }

    static void
    add_feature_observations(const FeatureObservations& feature_observations) {

        const StateIdentifier state_identifier = state.imu_state.identifier;
        const StateIdentifier previous_state_identifier = state_identifier - 1;

        size_t tracked_features = 0;

        const double dt = state.imu_state.time_interval_to_nearest_image;

        for (const auto& feature_observation :
             feature_observations.observations) {

            // This is a new feature observation
            if (!feature_track_map.contains(feature_observation.identifier)) {

                // Don't add a new feature_observation if the map server is
                // full
                if (feature_track_map.full()) {
                    printf("Warning: Feature tracks full\r\n");
                    continue;
                }

                feature_track_map.insert(etl::make_pair(
                    feature_observation.identifier,
                    FeatureTrack(feature_observation.identifier)));

                // Extrapolate where the feature_observation will be at this
                // particular IMU state
                feature_track_map.at(feature_observation.identifier)
                    .observations.insert(etl::make_pair(
                        state_identifier,
                        Vector2f(feature_observation.u +
                                     feature_observation.u_velocity * dt,
                                 feature_observation.v +
                                     feature_observation.v_velocity * dt)));

                feature_track_map.at(feature_observation.identifier)
                    .observations_velocity.insert(etl::make_pair(
                        state_identifier,
                        Vector2f(feature_observation.u_velocity,
                                 feature_observation.v_velocity)));

                const bool was_extracted_in_previous_frame = !(
                    feature_observation.u_init == -1 &&
                    feature_observation.v_init == -1);

                // If the feature_observation was extracted in the previous
                // camera frame, we want to add an extra observation for the
                // previous IMU frame
                if (was_extracted_in_previous_frame &&
                    state.augmented_imu_states.contains(
                        previous_state_identifier)) {

                    const double time_interval_to_nearest_image_frame =
                        state.augmented_imu_states[previous_state_identifier]
                            .time_interval_to_nearest_image;

                    // Extrapolate where the feature observation was at the
                    // previous IMU state
                    feature_track_map.at(feature_observation.identifier)
                        .observations.insert(etl::make_pair(
                            previous_state_identifier,
                            Vector2f(
                                feature_observation.u_init +
                                    feature_observation.u_init_velocity *
                                        time_interval_to_nearest_image_frame,
                                feature_observation.v_init +
                                    feature_observation.v_init_velocity *
                                        time_interval_to_nearest_image_frame)));

                    feature_track_map.at(feature_observation.identifier)
                        .observations_velocity.insert(etl::make_pair(
                            previous_state_identifier,
                            Vector2f(feature_observation.u_init_velocity,
                                     feature_observation.v_init_velocity)));
                }

            } else {

                if (!feature_track_map.at(feature_observation.identifier)
                         .observations.full()) {

                    // Extrapolate feature observation position to current
                    // IMU frame
                    feature_track_map.at(feature_observation.identifier)
                        .observations.insert(etl::make_pair(
                            state_identifier,
                            Vector2f(feature_observation.u +
                                         feature_observation.u_velocity * dt,
                                     feature_observation.v +
                                         feature_observation.v_velocity * dt)));

                    feature_track_map.at(feature_observation.identifier)
                        .observations_velocity.insert(etl::make_pair(
                            state_identifier,
                            Vector2f(feature_observation.u_velocity,
                                     feature_observation.v_velocity)));

                    tracked_features++;
                }
            }
        }

        tracking_rate = static_cast<double>(tracked_features) /
                        static_cast<double>(feature_track_map.size());
    }

    static void remove_entries_from_state(
        const StateIdentifierBuffer& state_identifiers_to_remove) {

        for (const StateIdentifier& state_identifier :
             state_identifiers_to_remove) {

            const long state_sequence = etl::distance(
                state.augmented_imu_states.begin(),
                state.augmented_imu_states.find(state_identifier));

            const long state_start = IMU_ERROR_STATE_SIZE + 6 * state_sequence;
            const long state_end   = state_start + 6;

            Map<MatrixXf, 0, OuterStride<>> state_covariance(
                state_covariance_buffer,
                state_covariance_rows,
                state_covariance_cols,
                OuterStride<>(STATE_COVARIANCE_BUFFER_STRIDE));

            // Update the state covariance matrix
            if (state_end < state_covariance.rows()) {
                state_covariance.block(state_start,
                                       0,
                                       state_covariance.rows() - state_end,
                                       state_covariance.cols()) =
                    state_covariance.block(state_end,
                                           0,
                                           state_covariance.rows() - state_end,
                                           state_covariance.cols());

                state_covariance.block(0,
                                       state_start,
                                       state_covariance.rows(),
                                       state_covariance.cols() - state_end) =
                    state_covariance.block(0,
                                           state_end,
                                           state_covariance.rows(),
                                           state_covariance.cols() - state_end);
            }

            state_covariance_cols -= 6;
            state_covariance_rows -= 6;

            state.augmented_imu_states.erase(state_identifier);
        }
    }

    static bool is_stationary() {

        if (feature_track_map.empty()) {
            return false;
        }

        etl::list<float, MAX_NUMBER_OF_FEATURE_TRACKS> feature_track_distances;

        for (const auto& feature_track_iterator : feature_track_map) {
            const FeatureTrack& feature_track = feature_track_iterator.second;

            if (feature_track.observations.size() < 2) {
                continue;
            }

            // Grab all the state identifiers
            StateIdentifierBuffer state_identifiers;

            for (const auto& observation_iterator :
                 feature_track.observations) {
                state_identifiers.push_back(observation_iterator.first);
            }

            // Sort them so that we can access the observations sequentially
            etl::sort(state_identifiers.begin(), state_identifiers.end());

            float feature_total_distance = 0.0f;

            for (size_t i = 1; i < state_identifiers.size(); i++) {

                const Eigen::Vector2f& previous = feature_track.observations.at(
                    state_identifiers[i - 1]);

                const Eigen::Vector2f& next = feature_track.observations.at(
                    state_identifiers[i]);

                feature_total_distance += (next - previous).norm();
            }

            feature_track_distances.push_back(feature_total_distance);
        }

        if (feature_track_distances.empty()) {
            return false;
        }

        bool has_small_feature_movement = false;

        feature_track_distances.sort();

        if (feature_track_distances.size() >= 20) {

            // Remove outliers roughly
            auto iterator = feature_track_distances.end();
            for (size_t i = 0; i < 9; i++) { iterator--; };

            const float max_distance = *iterator;

            has_small_feature_movement =
                max_distance < configuration.zero_update_max_feature_distance;
        } else {

            // If we have quite small amount of features, we just grab the
            // greatest distance and check against that
            has_small_feature_movement =
                feature_track_distances.back() <
                configuration.zero_update_max_feature_distance;
        }

        return has_small_feature_movement;
    }

    static void find_feature_tracks_ready_for_update(
        FeatureIdentifierBuffer& out_invalid_features,
        FeatureIdentifierBuffer& out_features_ready_for_update) {

        out_invalid_features.clear();
        out_features_ready_for_update.clear();

        for (auto iterator = feature_track_map.begin();
             iterator != feature_track_map.end();
             iterator++) {

            FeatureTrack& feature = iterator->second;

            const bool is_currently_tracked = feature.observations.contains(
                state.imu_state.identifier);

            if (!is_currently_tracked) {

                // If we have lost track of this feature and do not have enough
                // observations to triangulate it, it should just be removed
                if (feature.observations.size() <
                    (size_t)configuration.least_amount_of_observations) {
                    out_invalid_features.push_back(feature.identifier);
                    continue;
                }

                // For the feature to be used in an update, we have to be able
                // to triangulate it with its current observations, since there
                // will be no more with this feature since we have lost track of
                // it
                const bool could_not_be_triangulated =
                    !feature.has_been_triangulated &&
                    (!feature.has_enough_movement_to_triangulate(
                         state.augmented_imu_states,
                         is_currently_tracked) ||
                     !feature.attempt_triangulation(
                         state.augmented_imu_states,
                         state.imu_state.identifier));

                if (could_not_be_triangulated) {
                    out_invalid_features.push_back(feature.identifier);
                    continue;
                }

                out_features_ready_for_update.push_back(feature.identifier);

            } else {

                const bool has_been_tracked_over_max_track_length =
                    feature.observations.size() >=
                    (size_t)configuration.max_track_length;

                // Just continue if we are currently tracking this feature and
                // it has not been tracked long enough
                if (!has_been_tracked_over_max_track_length) {
                    continue;
                }

                const bool could_not_be_triangulated =
                    !feature.has_been_triangulated &&
                    (!feature.has_enough_movement_to_triangulate(
                         state.augmented_imu_states,
                         is_currently_tracked) ||
                     !feature.attempt_triangulation(
                         state.augmented_imu_states,
                         state.imu_state.identifier));

                // In the case where the triangulation fails for a feature which
                // is still tracked, we don't have to discard it just yet, since
                // it might be able to be triangulated in the next frames
                if (could_not_be_triangulated) {
                    continue;
                }

                out_features_ready_for_update.push_back(feature.identifier);
            }
        }
    }

    static void update_with_features(const FeatureIdentifierBuffer& features) {

        // Calculate the amount of rows in the jacobian used in the measurement
        // update
        int jacobian_row_size = 0;

        for (const FeatureIdentifier& identifier : features) {
            const FeatureTrack& feature = feature_track_map[identifier];

            // Do - 3 here as after null space projection, the matrix
            // will be of size (2M - 3). This is because the jacobian is
            // of size 2Mx3, with full column rank, yielding the nullity
            // to be: nullity = dim(jacobian) - rank(jacobian) = 2*M - 3
            jacobian_row_size += 2 * feature.observations.size() - 3;
        }

        MatrixXf H = MatrixXf::Zero(jacobian_row_size, state_covariance_cols);

        VectorXf residuals = VectorXf::Zero(jacobian_row_size);

        int row_index = 0;

        for (const FeatureIdentifier& identifier : features) {

            const FeatureTrack& feature = feature_track_map.at(identifier);

            StateIdentifierBuffer state_identifiers;

            for (const auto& measurement : feature.observations) {
                state_identifiers.push_back(measurement.first);
            }

            // Calculate the jacobian and the residual for all the
            // observations of the feature
            //
            // The -3 here comes from the null space projection
            MatrixXf H_j = MatrixXf(2 * state_identifiers.size() - 3,
                                    state_covariance_cols);

            VectorXf r_j = VectorXf(2 * state_identifiers.size() - 3);

            calculate_feature_jacobian(feature, state_identifiers, H_j, r_j);

            if (succeeds_mahalanobis_test(H_j,
                                          r_j,
                                          2 * state_identifiers.size() - 3)) {

                H.block(row_index, 0, H_j.rows(), H.cols()) = H_j.leftCols(
                    H.cols());
                residuals.block(row_index, 0, r_j.rows(), 1) = r_j;

                row_index += H_j.rows();
            }
        }

        // Resize for the features passing the gating test
        if (row_index != jacobian_row_size) {
            H.conservativeResize(row_index, H.cols());
            residuals.conservativeResize(row_index);
        }

        measurement_update(H, residuals);
    }

    static void force_update_with_states(const StateIdentifierBuffer& states) {

        int jacobian_row_size = 0;

        static FeatureIdentifierBuffer associated_features;
        associated_features.clear();

        // First we find the associated features to these states
        for (auto& feature_entry : feature_track_map) {

            FeatureTrack& feature = feature_entry.second;

            // Then we need to find all the states which are also associated
            // with this feature
            static StateIdentifierBuffer involved_state_identifiers;
            involved_state_identifiers.clear();

            for (const StateIdentifier& identifier : states) {

                if (feature.observations.find(identifier) !=
                    feature.observations.end()) {
                    involved_state_identifiers.push_back(identifier);
                }
            }

            if (involved_state_identifiers.empty()) {
                continue;
            }

            if (feature.has_been_triangulated) {

                // If the anchor state for this feature is in the states to use
                // for the feature update, we need to find a new one as these
                // states will be remove from the state vector
                const bool needs_new_anchor_state =
                    etl::find(involved_state_identifiers.begin(),
                              involved_state_identifiers.end(),
                              feature.identifier_anchor) !=
                    involved_state_identifiers.end();

                if (needs_new_anchor_state) {

                    StateIdentifier new_anchor_identifier =
                        get_new_anchor_identifier(feature,
                                                  involved_state_identifiers);

                    const AugmentedIMUState& new_anchor_state =
                        state.augmented_imu_states.at(new_anchor_identifier);

                    const Matrix3f& R_wc =
                        new_anchor_state.T_world_camera.linear();
                    const Vector3f& t_cw =
                        new_anchor_state.T_world_camera.translation();

                    const Vector3f feature_position_camera_frame =
                        R_wc.inverse() * (feature.position - t_cw);

                    feature.inverse_depth = 1 /
                                            feature_position_camera_frame.z();

                    feature.observation_anchor_homogeneous.x() =
                        feature.observations.at(new_anchor_identifier).x();

                    feature.observation_anchor_homogeneous.y() =
                        feature.observations.at(new_anchor_identifier).y();

                    feature.identifier_anchor = new_anchor_identifier;
                }
            }

            // Remember, we have the list of augmented states to perform the
            // update for. These states have feature observations. These
            // features are observed in other augmented states, which are the
            // involved states.
            //
            // In order to perform the forced update with this feature we need
            // to check if it has been triangulated first. If not we need to
            // perform the triangulation, but the triangulation is only possible
            // if the number of associated states with this feature is greater
            // than 1, as we need at least two feature observations to do the
            // triangulation

            const bool triangulation_is_possible =
                involved_state_identifiers.size() > 1;

            if (!currently_stationary && triangulation_is_possible) {

                const bool is_currently_tracked =
                    feature.observations.find(state.imu_state.identifier) !=
                    feature.observations.end();

                if (!feature.has_been_triangulated) {

                    if (!feature.has_enough_movement_to_triangulate(
                            state.augmented_imu_states,
                            is_currently_tracked)) {

                        continue;

                    } else {

                        // Here we pass -1 as the current state identifier to
                        // signify that we can indeed use first state which has
                        // observed this feature in the triangulation
                        if (!feature.attempt_triangulation(
                                state.augmented_imu_states,
                                -1)) {
                            continue;
                        }
                    }
                }

                associated_features.push_back(feature.identifier);

                jacobian_row_size += 2 * involved_state_identifiers.size() - 3;
            }
        }

        if (!currently_stationary && !associated_features.empty()) {

            MatrixXf H = MatrixXf::Zero(jacobian_row_size,
                                        state_covariance_cols);

            VectorXf residuals = VectorXf::Zero(jacobian_row_size);

            int row_index = 0;

            for (auto& feature_iterator : feature_track_map) {

                FeatureTrack& feature = feature_iterator.second;

                static StateIdentifierBuffer involved_state_identifiers;
                involved_state_identifiers.clear();

                for (const StateIdentifier& identifier : states) {
                    if (feature.observations.find(identifier) !=
                        feature.observations.end()) {
                        involved_state_identifiers.push_back(identifier);
                    }
                }

                const bool feature_should_be_used_in_update =
                    etl::find(associated_features.begin(),
                              associated_features.end(),
                              feature.identifier) != associated_features.end();

                if (feature_should_be_used_in_update) {

                    // Calculate the jacobian and the residual for all the
                    // observations of the feature
                    //
                    // The -3 here comes from the null space projection
                    MatrixXf H_j(2 * involved_state_identifiers.size() - 3,
                                 state_covariance_cols);

                    VectorXf r_j(2 * involved_state_identifiers.size() - 3);

                    calculate_feature_jacobian(feature,
                                               involved_state_identifiers,
                                               H_j,
                                               r_j);

                    if (succeeds_mahalanobis_test(
                            H_j,
                            r_j,
                            2 * involved_state_identifiers.size() - 3)) {

                        H.block(row_index, 0, H_j.rows(), H.cols())  = H_j;
                        residuals.block(row_index, 0, r_j.rows(), 1) = r_j;

                        row_index += H_j.rows();
                    }
                }

                // Erase the states which are used in the update from this
                // feature
                for (const StateIdentifier& identifier :
                     involved_state_identifiers) {
                    feature.observations.erase(identifier);
                    feature.observations_velocity.erase(identifier);
                }
            }

            // Resize for the features passing the gating test
            if (row_index != jacobian_row_size) {
                H.conservativeResize(row_index, H.cols());
                residuals.conservativeResize(row_index);
            }

            measurement_update(H, residuals);

        }
        // Simply delete observations if the system is stationary or
        // there are no features
        else {
            for (auto& feature_iterator : feature_track_map) {
                FeatureTrack& feature = feature_iterator.second;

                static StateIdentifierBuffer involved_state_identifiers;
                involved_state_identifiers.clear();

                for (const auto& identifier : states) {
                    if (feature.observations.find(identifier) !=
                        feature.observations.end())
                        involved_state_identifiers.push_back(identifier);
                }

                if (involved_state_identifiers.empty()) {
                    continue;
                }

                for (const auto& identifier : involved_state_identifiers) {
                    feature.observations.erase(identifier);
                    feature.observations_velocity.erase(identifier);
                }
            }
        }
    }

    static void
    calculate_observation_jacobian(const FeatureTrack& feature,
                                   const StateIdentifier& state_identifier,
                                   Matrix<float, 2, 6>& out_H_x,
                                   Matrix<float, 2, 6>& out_H_extrinsics,
                                   Matrix<float, 2, 3>& out_H_feature,
                                   Vector2f& out_residual) {

        const AugmentedIMUState& imu_state_snapshot =
            state.augmented_imu_states.at(state_identifier);

        // Camera extrinsics. The translation is read as the translation
        // from the IMU frame to the camera frame in the IMU frame
        const Matrix3f& R_camera_imu = imu_state_snapshot.T_camera_imu.linear();
        const Vector3f& t_camera_imu =
            imu_state_snapshot.T_camera_imu.translation();

        // Pose of the augmented IMU snapshot state
        const Matrix3f R_world_imu =
            imu_state_snapshot.orientation.toRotationMatrix();
        const Matrix3f R_imu_world = R_world_imu.transpose();

        const Vector3f& p_world = imu_state_snapshot.position;

        // Camera pose, which includes the camera-IMU extrinsics. The
        // translation is read as the translation from world to camera, in
        // world frame
        const Matrix3f R_camera_world       = R_camera_imu * R_imu_world;
        const Vector3f t_camera_world_world = p_world +
                                              R_world_imu * t_camera_imu;

        // Feature position in world and camera frame, respectively
        const Vector3f& feature_world = feature.position;

        const Vector3f feature_camera = R_camera_world *
                                        (feature_world - t_camera_world_world);

        // Translation vector to feature position from the IMU position,
        // given in world
        const Vector3f t_feature_imu_world =
            (initialised ? feature_world - imu_state_snapshot.position_FEJ
                         : feature_world - p_world);

        // Measurement
        const Vector2f& z = feature.observations.find(state_identifier)->second;

        // ---
        //      Jacobian of the residual (z) with respect to the 3D position
        //      of the feature in the camera frame
        // ---
        Matrix<float, 2, 3> dz_dfeature_camera = Matrix<float, 2, 3>::Zero();

        dz_dfeature_camera(0, 0) = 1.0f / feature_camera(2);
        dz_dfeature_camera(1, 1) = 1.0f / feature_camera(2);

        dz_dfeature_camera(0, 2) = -feature_camera(0) /
                                   (feature_camera(2) * feature_camera(2));
        dz_dfeature_camera(1, 2) = -feature_camera(1) /
                                   (feature_camera(2) * feature_camera(2));

        // ---
        //      Jacobian of the feature position with respect to the IMU
        //      orientation and position
        // ---
        Matrix<float, 3, 6> dfeature_camera_dpose = Matrix<float, 3, 6>::Zero();

        // Jacobian of the feature position in camera frame with respect to
        // the IMU orientation
        dfeature_camera_dpose.leftCols(
            3) = R_camera_world * math::skew_symmetric(t_feature_imu_world);

        // Jacobian of the feature position in camera frame with respect to
        // the IMU position
        dfeature_camera_dpose.rightCols(3) = -R_camera_world;

        // ---
        //      Jacobian of the feature position with respect to the
        //      extrinsics
        // ---
        Matrix<float, 3, 6> dfeature_camera_dextrinsics =
            Matrix<float, 3, 6>::Zero();

        // Jacobian with respect to the camera-IMU orientation
        dfeature_camera_dextrinsics.leftCols(
            3) = -math::skew_symmetric(R_camera_world * t_feature_imu_world) +
                 math::skew_symmetric(R_camera_imu * t_camera_imu);

        // Jacobian with respect to the camera-IMU translation
        dfeature_camera_dextrinsics.rightCols(3) = -R_camera_imu;

        // Build the final jacobians
        out_H_x          = dz_dfeature_camera * dfeature_camera_dpose;
        out_H_extrinsics = dz_dfeature_camera * dfeature_camera_dextrinsics;
        out_H_feature    = dz_dfeature_camera * R_camera_world;

        out_residual = z - Vector2f(feature_camera(0) / feature_camera(2),
                                    feature_camera(1) / feature_camera(2));
    }

    static void
    calculate_feature_jacobian(const FeatureTrack& feature,
                               const StateIdentifierBuffer& state_identifiers,
                               MatrixXf& out_H,
                               VectorXf& out_r) {

        const int jacobian_row_size = 2 * state_identifiers.size();

        // Jacobian with respect to the IMU state
        MatrixXf H_xj = MatrixXf::Zero(jacobian_row_size,
                                       state_covariance_cols);

        // Jacobian with respect to the feature position
        MatrixXf H_fj = MatrixXf::Zero(jacobian_row_size, 3);

        // Residual
        VectorXf r_j = VectorXf::Zero(jacobian_row_size);

        int row_index = 0;

        for (const auto& state_identifier : state_identifiers) {
            Matrix<float, 2, 6> H_xi = Matrix<float, 2, 6>::Zero();
            Matrix<float, 2, 6> H_ei = Matrix<float, 2, 6>::Zero();
            Matrix<float, 2, 3> H_fi = Matrix<float, 2, 3>::Zero();
            Vector2f r_i             = Vector2f::Zero();
            calculate_observation_jacobian(feature,
                                           state_identifier,
                                           H_xi,
                                           H_ei,
                                           H_fi,
                                           r_i);

            const int augmented_state_index = etl::distance(
                state.augmented_imu_states.begin(),
                state.augmented_imu_states.find(state_identifier));

            H_xj.block<2, 6>(row_index,
                             IMU_ERROR_STATE_SIZE +
                                 6 * augmented_state_index) = H_xi;

            H_xj.block<2, 6>(row_index, 15) = H_ei;

            if (configuration.estimate_time_compensation) {
                H_xj.block<2, 1>(row_index, 21) =
                    feature.observations_velocity.at(state_identifier);
            }

            H_fj.block<2, 3>(row_index, 0) = H_fi;

            r_j.segment<2>(row_index) = r_i;

            row_index += 2;
        }

        // Project the residual and Jacobians onto the nullspace of H_fj so
        // that we can effectively cancel the terms related to the feature
        // position
        JacobiSVD<MatrixXf> svd_helper(H_fj, ComputeFullU);
        MatrixXf A = svd_helper.matrixU().rightCols(jacobian_row_size - 3);

        out_H = A.transpose() * H_xj;
        out_r = A.transpose() * r_j;
    }

    static void measurement_update(MatrixXf& H, VectorXf& r) {

        if (H.rows() == 0 || r.rows() == 0) {
            return;
        }

        // Perform QR decomposition to make a reduced H if the number of rows is
        // greater then the number of cols
        if (H.rows() > H.cols()) {

            // Do QR decomposition with Givens rotations, as expressed in
            // Matrix Computations, 4th Edition by Golub and Van Loan,
            // algorithm 5.2.4
            //
            // The inspiration for this is from OpenVINS, which uses the
            // same method for the QR decomposition
            JacobiRotation<float> G;

            for (int j = 0; j < H.cols(); j++) {
                for (int i = (int)H.rows() - 1; i > j; i--) {

                    G.makeGivens(H(i - 1, j), H(i, j));

                    (H.block(i - 1, j, 2, H.cols() - j))
                        .applyOnTheLeft(0, 1, G.adjoint());

                    (r.block(i - 1, 0, 2, 1)).applyOnTheLeft(0, 1, G.adjoint());
                }
            }

            const int rows = MIN(H.rows(), H.cols());

            H.conservativeResize(rows, H.cols());
            r.conservativeResize(rows);
        }

        Map<MatrixXf, 0, OuterStride<>> state_covariance(
            state_covariance_buffer,
            state_covariance_rows,
            state_covariance_cols,
            OuterStride<>(STATE_COVARIANCE_BUFFER_STRIDE));

        // ---
        // Compute the Kalman gain.
        // ---

        MatrixXf Sinv = MatrixXf::Identity(H.rows(), H.rows());

        // Force S to go out of scope to save memory further on
        //
        // In the following C is the number of columns in H, which is equal to
        // the rows and columns in the state covariance matrix. R is the number
        // of rows in H
        {
            MatrixXf S(H.rows(), H.rows());

            // This product is RxC * CxC * CxR = RxR
            S.triangularView<Upper>() = H * state_covariance * H.transpose();
            S.triangularView<Upper>() += configuration.noise_feature *
                                         MatrixXf::Identity(H.rows(), H.rows());

            S.selfadjointView<Upper>().llt().solveInPlace(Sinv);
        }

        // This product is CxC * CxR * RxR = CxR
        const MatrixXf K = state_covariance * H.transpose() *
                           Sinv.selfadjointView<Upper>();

        // This product is CxR * R = C
        const VectorXf delta_x = K * r;

        // ---
        // IMU state update
        // ---
        const VectorXf& delta_x_imu = delta_x.head(IMU_ERROR_STATE_SIZE);

        // Do a rough check for the delta not being to large and singular
        if (isnan(delta_x_imu.sum()) ||
            delta_x_imu.segment<3>(3).norm() > 0.5 ||
            delta_x_imu.segment<3>(6).norm() > 1.0) {

            return;
        }

        const Quaternionf dq_imu = math::small_angle_quaternion(
            delta_x_imu.head<3>());

        state.imu_state.orientation = dq_imu * state.imu_state.orientation;
        state.imu_state.velocity += delta_x_imu.segment<3>(3);
        state.imu_state.position += delta_x_imu.segment<3>(6);
        state.imu_state.angular_velocity_bias += delta_x_imu.segment<3>(9);
        state.imu_state.acceleration_bias += delta_x_imu.segment<3>(12);

        // ---
        // IMU-camera extrinsics
        // ---
        const Quaternionf dq_extrinsic = math::small_angle_quaternion(
            delta_x_imu.segment<3>(15));

        state.imu_state.T_camera_imu.linear() *=
            dq_extrinsic.toRotationMatrix().transpose();

        state.imu_state.T_camera_imu.translation() += delta_x_imu.segment<3>(
            18);

        // ---
        // Time compensation
        // ---
        state.timestamp_compensation += delta_x_imu(21);

        // ---
        // Augmented IMU states
        // ---
        auto augmented_imu_states_iterator = state.augmented_imu_states.begin();

        for (size_t i = 0; i < state.augmented_imu_states.size();
             i++, augmented_imu_states_iterator++) {

            const VectorXf& delta_x_augmented = delta_x.segment<6>(
                IMU_ERROR_STATE_SIZE + i * 6);

            const Quaternionf dq_augmented = math::small_angle_quaternion(
                delta_x_augmented.head<3>());

            augmented_imu_states_iterator->second.orientation =
                dq_augmented *
                augmented_imu_states_iterator->second.orientation;

            augmented_imu_states_iterator->second.position +=
                delta_x_augmented.tail<3>();

            // Update corresponding camera pose
            const Matrix3f R_bc =
                state.imu_state.T_camera_imu.linear().transpose();

            const Vector3f& t_cb = state.imu_state.T_camera_imu.translation();

            const Matrix3f R_wb = augmented_imu_states_iterator->second
                                      .orientation.toRotationMatrix();

            augmented_imu_states_iterator->second.T_world_camera.linear() =
                R_wb * R_bc;

            augmented_imu_states_iterator->second.T_world_camera.translation() =
                augmented_imu_states_iterator->second.position + R_wb * t_cb;
        }

        // ---
        // State covariance
        // ---

        // This product is CxR * RxC * CxC = CxC
        state_covariance.triangularView<Upper>() -= K * H * state_covariance;
        state_covariance = state_covariance.selfadjointView<Upper>();
    }

    static void stationary_update() {

        Map<MatrixXf, 0, OuterStride<>> state_covariance(
            state_covariance_buffer,
            state_covariance_rows,
            state_covariance_cols,
            OuterStride<>(STATE_COVARIANCE_BUFFER_STRIDE));

        const long N = (long)state.augmented_imu_states.size();

        // ---
        // Stationary measurement jacobian
        // ---
        MatrixXf H = MatrixXf::Zero(9, state_covariance.cols());

        // IMU error-state velocity
        H.block<3, 3>(0, 3).setIdentity();

        // Last two augmented IMU error-states' orientations
        H.block<3, 3>(3, IMU_ERROR_STATE_SIZE + 6 * N - 6) =
            -0.5 * Matrix3f::Identity();
        H.block<3, 3>(3, IMU_ERROR_STATE_SIZE + 6 * N - 12) =
            0.5 * Matrix3f::Identity();

        // Last two augmented IMU error-state' positions
        H.block<3, 3>(6, IMU_ERROR_STATE_SIZE + 6 * N - 3).setIdentity();
        H.block<3, 3>(6,
                      IMU_ERROR_STATE_SIZE + 6 * N - 9) = -Matrix3f::Identity();

        // ---
        // Stationary residual
        // ---
        VectorXf r = VectorXf::Zero(9);

        // IMU error-state velocity residual, with the constraint assuming
        // the true velocity is zero
        r.segment<3>(0) = -state.imu_state.velocity;

        // Last two augmented IMU states' orientation residual, where we
        // only use the imaginary part of the quaternion with the assumption
        // that the error will be small and that the real part of the
        // quaternion then will tend to being 1.
        const Quaternionf delta_q =
            state.augmented_imu_states[state.imu_state.identifier].orientation *
            state.augmented_imu_states[state.imu_state.identifier - 1]
                .orientation.conjugate();

        r.segment<3>(3) = Vector3f(delta_q.x(), delta_q.y(), delta_q.z());

        // Last two augmented IMU states' position residual, where we also
        // have here have the assumption that the difference between the
        // true position at k and k-1 is 0 due to the constraint, so we can
        // drop the terms
        r.segment<3>(6) = -(
            state.augmented_imu_states[state.imu_state.identifier].position -
            state.augmented_imu_states[state.imu_state.identifier - 1]
                .position);

        // ---
        // Measurement covariance
        // ---
        MatrixXf R = MatrixXf::Zero(9, 9);

        R.block<3, 3>(0, 0) = configuration.zero_update_noise_v *
                              Matrix3f::Identity();
        R.block<3, 3>(3, 3) = configuration.zero_update_noise_q *
                              Matrix3f::Identity();
        R.block<3, 3>(6, 6) = configuration.zero_update_noise_p *
                              Matrix3f::Identity();

        // ---
        // Update step
        // ---
        const MatrixXf S           = H * state_covariance * H.transpose() + R;
        const MatrixXf K_transpose = S.ldlt().solve(H * state_covariance);
        const MatrixXf K           = K_transpose.transpose();

        const VectorXf delta_x = K * r;

        // ---
        // IMU state update
        // ---
        const VectorXf& delta_x_imu = delta_x.head(IMU_ERROR_STATE_SIZE);

        const Quaternionf dq_imu = math::small_angle_quaternion(
            delta_x_imu.head<3>());

        state.imu_state.orientation = dq_imu * state.imu_state.orientation;
        state.imu_state.velocity += delta_x_imu.segment<3>(3);
        state.imu_state.position += delta_x_imu.segment<3>(6);
        state.imu_state.angular_velocity_bias += delta_x_imu.segment<3>(9);
        state.imu_state.acceleration_bias += delta_x_imu.segment<3>(12);

        // ---
        // IMU-camera extrinsics
        // ---
        const Quaternionf dq_extrinsic = math::small_angle_quaternion(
            delta_x_imu.segment<3>(15));

        state.imu_state.T_camera_imu.linear() *=
            dq_extrinsic.toRotationMatrix().transpose();
        state.imu_state.T_camera_imu.translation() += delta_x_imu.segment<3>(
            18);

        // ---
        // Time compensation
        // ---
        state.timestamp_compensation += delta_x_imu(21);

        // ---
        // Augmented IMU states
        // ---
        auto augmented_imu_states_iterator = state.augmented_imu_states.begin();

        for (size_t i = 0; i < state.augmented_imu_states.size();
             ++i, ++augmented_imu_states_iterator) {

            const VectorXf& delta_x_augmented = delta_x.segment<6>(
                IMU_ERROR_STATE_SIZE + i * 6);

            const Quaternionf dq_augmented = math::small_angle_quaternion(
                delta_x_augmented.head<3>());

            augmented_imu_states_iterator->second.orientation =
                dq_augmented *
                augmented_imu_states_iterator->second.orientation;

            augmented_imu_states_iterator->second.position +=
                delta_x_augmented.tail<3>();

            // Update corresponding camera pose
            const Matrix3f R_bc =
                state.imu_state.T_camera_imu.linear().transpose();

            const Vector3f& t_cb = state.imu_state.T_camera_imu.translation();

            const Matrix3f R_wb = augmented_imu_states_iterator->second
                                      .orientation.toRotationMatrix();

            augmented_imu_states_iterator->second.T_world_camera.linear() =
                R_wb * R_bc;

            augmented_imu_states_iterator->second.T_world_camera.translation() =
                augmented_imu_states_iterator->second.position + R_wb * t_cb;
        }

        // ---
        // State covariance
        // ---
        const MatrixXf I_KH = MatrixXf::Identity(K.rows(), H.cols()) - K * H;
        state_covariance    = I_KH * state_covariance;

        // Make the covariance symmetric
        state_covariance =
            ((state_covariance + state_covariance.transpose()) / 2.0).eval();
    }

    static bool succeeds_mahalanobis_test(const MatrixXf& H,
                                          const VectorXf& r,
                                          const int& dof) {

        const Map<MatrixXf, 0, OuterStride<>> state_covariance(
            state_covariance_buffer,
            state_covariance_rows,
            state_covariance_cols,
            OuterStride<>(STATE_COVARIANCE_BUFFER_STRIDE));

        const MatrixXf P1 = H * state_covariance * H.transpose();
        const MatrixXf P2 = configuration.noise_feature *
                            MatrixXf::Identity(H.rows(), H.rows());

        const double gamma = r.transpose() * (P1 + P2).ldlt().solve(r);

        if (gamma < chi_squared_test_table[dof]) {
            return true;
        } else {
            return false;
        }
    }

    static void find_redundant_augmented_imu_states(
        StateIdentifierBuffer& out_redundant_augmented_imu_states) {

        // In the following, we search for at maximum two augmented IMU states
        // which can be marked as redundant due to little movement from a set
        // reference state, R.
        //
        // If neither R+1 or R+2 have little movement compared with R, we mark
        // the first state in the sliding window as redundant
        //
        //
        // R    R+1     R+2     Current state
        //
        // *   |-----------|
        //      States which
        //      are examined
        //      for little
        //      movement
        auto reference_state_iterator = state.augmented_imu_states.end();

        // Push the reference state four states back
        for (size_t i = 0; i < 4; i++) { reference_state_iterator--; }

        const Vector3f& reference_position =
            reference_state_iterator->second.T_world_camera.translation();
        const Matrix3f& reference_orientation =
            reference_state_iterator->second.T_world_camera.linear();

        auto first_state_iterator = state.augmented_imu_states.begin();

        // This is used to compare against the reference state. Initially this
        // thus points at state R+1
        auto state_iterator = reference_state_iterator;
        state_iterator++;

        // Now we check both R+1 and R+2
        for (size_t i = 0; i < 2; ++i) {

            const Vector3f& position =
                state_iterator->second.T_world_camera.translation();
            const Matrix3f& rotation =
                state_iterator->second.T_world_camera.linear().transpose();

            // Find how different this state is from the reference position
            const float distance = (position - reference_position).norm();
            const float angle =
                AngleAxisf(rotation * reference_orientation).angle();

            if (angle < configuration.rotation_threshold &&
                distance < configuration.translation_threshold &&
                tracking_rate > configuration.tracking_rate_threshold) {

                // Only add if it isn't already in the buffer of redundant
                // states
                if (etl::find(out_redundant_augmented_imu_states.begin(),
                              out_redundant_augmented_imu_states.end(),
                              state_iterator->first) ==
                    out_redundant_augmented_imu_states.end()) {

                    out_redundant_augmented_imu_states.push_back(
                        state_iterator->first);
                }

                state_iterator++;
            } else {

                // If the state has significant movement, we instead mark the
                // first state as redundant so we can keep this for reference
                // later
                if (etl::find(out_redundant_augmented_imu_states.begin(),
                              out_redundant_augmented_imu_states.end(),
                              first_state_iterator->first) ==
                    out_redundant_augmented_imu_states.end()) {
                    out_redundant_augmented_imu_states.push_back(
                        first_state_iterator->first);
                }

                // Move the first state forward since it is now added in the
                // redundant states
                first_state_iterator++;

                // Since the current state iterator has significant movement, we
                // move the iterator back so that we can examine other states
                state_iterator--;
                state_iterator--;
            }
        }

        // Sort the elements in the output vector.
        etl::sort(out_redundant_augmented_imu_states.begin(),
                  out_redundant_augmented_imu_states.end());
    }

    static StateIdentifier
    get_new_anchor_identifier(FeatureTrack& feature,
                              const StateIdentifierBuffer& excluded_states) {

        auto state_iterator = state.augmented_imu_states.begin();

        bool found_valid_anchor_state    = false;
        float minimum_reprojection_error = FLT_MAX;
        StateIdentifier new_anchor_identifier;

        for (size_t i = 0; i < state.augmented_imu_states.size();
             i++, state_iterator++) {

            const bool state_has_observed_feature =
                (feature.observations.find(state_iterator->first) !=
                 feature.observations.end());

            if (!state_has_observed_feature) {
                continue;
            }

            const bool state_is_not_in_excluded_list =
                etl::find(excluded_states.begin(),
                          excluded_states.end(),
                          state_iterator->first) == excluded_states.end();

            if (state_is_not_in_excluded_list) {

                const Matrix3f& R_wc =
                    state_iterator->second.T_world_camera.linear();

                // This is read as the translation from world to the camera
                // frame (right-to-left)
                const Vector3f& t_cw =
                    state_iterator->second.T_world_camera.translation();

                const Vector3f feature_position_in_camera_frame =
                    R_wc.inverse() * (feature.position - t_cw);

                const float reprojection_error =
                    Vector2f(
                        feature_position_in_camera_frame.x() /
                                feature_position_in_camera_frame.z() -
                            feature.observations.at(state_iterator->first).x(),
                        feature_position_in_camera_frame.y() /
                                feature_position_in_camera_frame.z() -
                            feature.observations.at(state_iterator->first).y())
                        .norm();

                if (minimum_reprojection_error > reprojection_error) {
                    minimum_reprojection_error = reprojection_error;
                    new_anchor_identifier      = state_iterator->first;
                    found_valid_anchor_state   = true;
                }
            }
        }

        if (found_valid_anchor_state) {
            return new_anchor_identifier;
        } else {
            state_iterator = state.augmented_imu_states.end();
            --state_iterator;
            return state_iterator->first;
        }
    }
}
