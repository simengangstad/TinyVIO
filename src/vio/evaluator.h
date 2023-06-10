#ifndef EVALUATOR_H
#define EVALUATOR_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#include "Eigen/Dense"
#pragma GCC diagnostic pop

#include <math.h>
#include <stddef.h>

/**
 * @brief Convenience structure for calculating metrics such as RMSE and NEES.
 */
struct Evaluator {

    struct SampleMetric {

        size_t samples             = 0;
        double current             = 0.0f;
        double accumulated         = 0.0f;
        double accumulated_squared = 0.0f;
        double maximum             = 0.0f;

        inline void append(const double sample) {
            samples++;

            accumulated += sample;
            accumulated_squared += sample * sample;

            current = sample;

            if (current > maximum) {
                maximum = current;
            }
        }

        inline double rmse() {

            if (samples == 0) {
                return 0.0;
            }

            return sqrt(accumulated_squared / static_cast<double>(samples));
        }

        inline double average() {
            if (samples == 0) {
                return 0.0;
            }

            return accumulated / static_cast<double>(samples);
        }
    };

    struct NEES {

        size_t samples = 0;

        double current_orientation        = 0.0;
        double current_velocity           = 0.0;
        double current_position           = 0.0;
        double current_gyro_bias          = 0.0;
        double current_accelerometer_bias = 0.0;

        double accumulated_orientation        = 0.0;
        double accumulated_velocity           = 0.0;
        double accumulated_position           = 0.0;
        double accumulated_gyro_bias          = 0.0;
        double accumulated_accelerometer_bias = 0.0;

        /**
         * @brief Adds a sample used for calculating NEES.
         */
        void add_sample(const Eigen::Vector3f& angle_axis_error,
                        const Eigen::Vector3f& velocity_error,
                        const Eigen::Vector3f& position_error,
                        const Eigen::Vector3f& gyro_bias_error,
                        const Eigen::Vector3f accelerometer_bias_error,
                        const Eigen::Matrix<float, 15, 15>& state_covariance);

        /**
         * @brief Prints the ANEES.
         */
        void print_average();

        /**
         * @brief Prints the current NEES.
         */
        void print_current();
    };

    /**
     * @brief The orientation error metric.
     */
    SampleMetric orientation_error_metric;

    /**
     * @brief The velocity error metric.
     */
    SampleMetric velocity_error_metric;

    /**
     * @brief The position error metric.
     */
    SampleMetric position_error_metric;

    /**
     * @brief The gyro bias error metric.
     */
    SampleMetric gyro_bias_error_metric;

    /**
     * @brief The accelerometer bias error metric.
     */
    SampleMetric accelerometer_bias_error_metric;

    /**
     * @brief Metric of the features observed in the feature.
     */
    SampleMetric feature_observations_metric;

    /**
     * @brief Metric of the feature tracks in the backend.
     */
    SampleMetric feature_tracks_metric;

    /**
     * @brief NEES the state.
     */
    NEES nees;

    /**
     * @brief Adds a state sample to the evaluator. The samples are used for
     * calculating NEES and sample metrics.
     */
    void add_state_sample(const Eigen::Quaternionf& true_orientation,
                          const Eigen::Quaternionf& estimated_orientation,
                          const Eigen::Vector3f& true_velocity,
                          const Eigen::Vector3f& estimated_velocity,
                          const Eigen::Vector3f& true_position,
                          const Eigen::Vector3f& estimated_position,
                          const Eigen::Vector3f& true_gyro_bias,
                          const Eigen::Vector3f& estimated_gyro_bias,
                          const Eigen::Vector3f& true_accelerometer_bias,
                          const Eigen::Vector3f& estimated_accelerometer_bias,
                          const Eigen::Matrix<float, 15, 15>& state_covariance);

    /**
     * @brief Adds a feature sample to the evaluator. Used to keep track of
     * number of feature observations and tracks.
     */
    void add_feature_sample(const size_t& number_of_feature_observations,
                            const size_t& number_of_feature_tracks);

    /**
     * @brief Prints the state metrics and feature metrics.
     */
    void print_metrics();
};

#endif
