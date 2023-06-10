#include "static_initialiser.h"

#define MIN_NEEDED_FEATURES_FOR_INITIALISATION (20)

#include <etl/map.h>
#include <etl/vector.h>

namespace static_initialiser {

    using namespace Eigen;

    typedef etl::map<FeatureIdentifier, Vector2f, MAX_NUMBER_OF_FEATURES>
        FeatureObservationMap;

    /**
     * @brief Calculates the orientation based on the measured acceleration and
     * the assumed world gravity vector as well as calculating the angular
     * velocity bias as the sum of the angular velocity measurements.
     *
     * @param imu_measurements The IMU measurements to use.
     * @param first_frame_time The first frame time. Used as the starting point
     * for the IMU measurements.
     * @param last_frame_time The last frame time. Used as the ending point for
     * the IMU measurements.
     * @param out_initial_angular_velocity_bias The angular velocity bias will
     * be placed in this variable.
     * @param out_initial_orientation The orientation will be placed in this
     * variable.
     *
     * @return The time of the last used IMU measurement.
     */
    static double
    calculate_orientation_and_bias(const IMUBuffer& imu_measurements,
                                   const double& first_frame_time,
                                   const double& last_frame_time,
                                   Vector3f& out_angular_velocity_bias,
                                   Quaternionf& out_orientation) {

        Vector3f sum_angular_velocity    = Vector3f::Zero();
        Vector3f sum_linear_acceleration = Vector3f::Zero();

        int processed_measurements = 0;

        double last_imu_timestamp = 0.0;

        for (const ImuDataPoint& measurement : imu_measurements) {

            const double timestamp = measurement.timestamp;

            // Don't include measurements before the first frame time
            if (timestamp < first_frame_time) {
                continue;
            }

            // Don't include measurements after the time bound of the last
            // frame.
            if (timestamp > last_frame_time) {
                break;
            }

            sum_angular_velocity += measurement.angular_velocity;
            sum_linear_acceleration += measurement.linear_acceleration;

            processed_measurements++;

            last_imu_timestamp = timestamp;
        }

        out_angular_velocity_bias = sum_angular_velocity /
                                    processed_measurements;

        const Vector3f gravity_imu = sum_linear_acceleration /
                                     processed_measurements;
        const Vector3f gravity_world(0.0, 0.0, -gravity_imu.norm());

        out_orientation = Quaternionf::FromTwoVectors(gravity_imu,
                                                      -gravity_world);

        return last_imu_timestamp;
    }

    /**
     * @brief Maximum feature distance allowed between static images.
     */
    static double max_feature_distance = 0.0;

    /**
     * @brief Number of consecutive image frames to trigger static
     * initialisation.
     */
    static int static_frames_threshold = 0;

    /**
     * @brief Counter for the number of static camera/image frames that will be
     * used in the static initialization.
     */
    static int static_frames_counter = 0;

    /**
     * @brief Difference between timestamp of the IMU measurement and the image.
     */
    static double time_offset_between_imu_and_image_measurements;

    /**
     * @brief The previous feature observations used in the initialisation
     * phase. Is used to compare the distance to the current feature
     * observations and see if they are below a given threshold.
     */
    FeatureObservationMap previous_feature_observations;

    /**
     * @brief Frame time of the IMU measurement closest associated with the
     * first camera/image frame.
     */
    static double first_frame_time = 0.0;

    void reset(const double& max_feature_distance_,
               const int& static_frames_threshold_,
               const double& time_offset_between_imu_and_image_measurements_) {

        max_feature_distance    = max_feature_distance_;
        static_frames_threshold = static_frames_threshold_;
        time_offset_between_imu_and_image_measurements =
            time_offset_between_imu_and_image_measurements_;

        static_frames_counter = 0;
        first_frame_time      = 0.0;
    }

    bool attempt_initialisation(const IMUBuffer& imu_measurements,
                                const FeatureObservations& feature_observations,
                                double& out_initialisation_timestamp,
                                Vector3f& out_initial_angular_velocity_bias,
                                Quaternionf& out_initial_orientation) {

        if (static_frames_counter == 0) {
            static_frames_counter++;

            previous_feature_observations.clear();

            for (const FeatureObservation& observation :
                 feature_observations.observations) {

                previous_feature_observations.insert(
                    etl::make_pair(observation.identifier,
                                   Vector2f(observation.u, observation.v)));
            }

            first_frame_time = feature_observations.timestamp +
                               time_offset_between_imu_and_image_measurements;
            return false;
        }

        // ----
        // Find distances from the initialisation features to the current
        // features
        // ----

        static FeatureObservationMap current_features;
        current_features.clear();

        static etl::vector<double, MAX_NUMBER_OF_FEATURES> distances;
        distances.clear();

        for (const FeatureObservation& observation :
             feature_observations.observations) {

            current_features.insert(
                etl::make_pair(observation.identifier,
                               Vector2f(observation.u, observation.v)));

            if (previous_feature_observations.contains(
                    observation.identifier)) {

                const Vector2f current_observation(observation.u,
                                                   observation.v);

                const Vector2f& previous_observation =
                    previous_feature_observations.at(observation.identifier);

                distances.push_back(
                    (current_observation - previous_observation).norm());
            }
        }

        // Abort and reset if there are not enough observations to start the
        // initialisation.
        if (distances.empty() ||
            distances.size() < MIN_NEEDED_FEATURES_FOR_INITIALISATION) {
            static_frames_counter = 0;
            return false;
        }

        // ----
        // Do rough outlier rejection
        // ----

        // Sort to find the maximum distances from the previous and current
        // frame.
        etl::sort(distances.begin(), distances.end());

        // Check if there are at least N distances, which are below the
        // threshold.
        auto iterator = distances.end();
        for (int i = 0; i < MINIMUM_REQUIRED_FEATURES_FOR_INITIALISATION - 1;
             i++)
            iterator--;

        const double max_distance = *iterator;

        if (max_distance < max_feature_distance) {
            static_frames_counter++;

            previous_feature_observations = current_features;

            if (static_frames_counter < static_frames_threshold) {
                return false;
            }
        } else {
            static_frames_counter = 0;
            return false;
        }

        // ----
        // Calculate the orientation and the angular velocity bias based on the
        // IMU measurements. This assumes of course that the IMU is stationary.
        // ----

        out_initialisation_timestamp = calculate_orientation_and_bias(
            imu_measurements,
            first_frame_time,
            feature_observations.timestamp +
                time_offset_between_imu_and_image_measurements,
            out_initial_angular_velocity_bias,
            out_initial_orientation);

        return true;
    }
}
