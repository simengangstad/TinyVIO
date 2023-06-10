#include "pipeline.h"

#include "backend.h"
#include "board.h"
#include "clock_controller.h"
#include "data_interface.h"
#include "ethernet.h"
#include "evaluator.h"
#include "frontend.h"
#include "logger.h"
#include "memory.h"
#include "profile.h"
#include "vio_config.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include <etl/vector.h>

#include <stdio.h>
#include <string.h>

namespace vio {

    using namespace Eigen;

    /**
     * @brief Buffer for the image data (placed in DTCM)
     */
    AT_OCRAM_SECTION_DATA(
        static uint8_t data_buffer[IMAGE_WIDTH * IMAGE_HEIGHT]);

    /**
     * @brief Holds the current IMU measurements.
     */
    static IMUBuffer imu_buffer;

    /**
     * @brief This is purely a transformations for visualisation when sending
     * the estimated pose back to the visualisation application.
     */
    static const Quaternionf q_worldaxes_eurocbodyaxes = Quaternionf(
        AngleAxisf(-M_PI / 2.0f, Vector3f(0.0f, 0.0f, 1.0f)));

    /**
     * @brief This is purely a transformations for visualisation when sending
     * the estimated pose back to the visualisation application.
     */
    static const Quaternionf q_worldaxes_eurocreferenceaxes = Quaternionf(
        AngleAxisf(M_PI / 2.0f, Vector3f(1.0f, 0.0f, 0.0f)));

    struct Pose {
        Vector3f translation;
        Quaternionf orientation;

        Pose() {
            translation.setZero();
            orientation = Quaternionf(1.0, 0.0, 0.0, 0.0);
        }

        Pose(const Vector3f t, const Matrix3f& R) {
            translation = t;
            orientation = Quaternionf(R);
        }

        Pose(const Vector3f t, const Quaternionf& q) {
            translation = t;
            orientation = q;
        }
    };

    /**
     * @brief Holds the feature observations from the frontend.
     */
    static FeatureObservations feature_observations;

    /**
     * @brief The IMU data points which arrive over the data link.
     */
    static etl::vector<ImuDataPoint, BATCH_SIZE> incoming_imu_data_points;

    /**
     * @brief The ground-truth data points which arrive over the data link.
     */
    static etl::vector<GroundTruthDataPoint, BATCH_SIZE>
        incoming_ground_truth_data_points;

    /**
     * @brief Used for evaluating different metrics of the pipeline: RMSE, NEES
     * etc.
     */
    static Evaluator evaluator;

    void run_pipeline_with_proxy_link() {

        if (!ethernet::initialise(ethernet::Port::PORT_1G,
                                  {192, 168, 0, 102},
                                  {255, 255, 255, 0},
                                  {192, 168, 0, 100})) {
            return;
        }

        if (!data_interface::initialise({192, 168, 0, 100}, 5001)) {
            logger::errorf("Failed to setup data interface\r\n");
            return;
        }

        const Configuration configuration =
            Configuration::euroc_configuration();

        configuration.print();

        frontend::configure(configuration);
        backend::configure(configuration);

        data_interface::set_start_time(0.0f);

        Isometry3f T_w_b0;

        bool got_ground_truth             = false;
        bool has_alignment_transformation = false;

        IMUState state;
        GroundTruthDataPoint current_ground_truth;

        size_t camera_frame_index = 0;

        double track_length = 0.0;

        while (true) {

            ImageDataPoint image_data_point;
            image_data_point.image.data = data_buffer;

            incoming_imu_data_points.clear();
            incoming_ground_truth_data_points.clear();

            bool success = data_interface::wait_for_next_batch(
                incoming_imu_data_points,
                incoming_ground_truth_data_points,
                image_data_point);

            if (!success) {
                logger::errorf("Failed to get next batch\r\n");
                break;
            }

            if (!incoming_ground_truth_data_points.empty()) {

                // Only start calculating the track length the system has
                // initialised
                if (backend::has_initialised()) {

                    // First, we calculate the distance between the previous
                    // last ground truth position and the first one in this
                    // batch
                    if (got_ground_truth) {
                        track_length += (q_worldaxes_eurocreferenceaxes *
                                             incoming_ground_truth_data_points
                                                 .begin()
                                                 ->position -
                                         current_ground_truth.position)
                                            .norm();
                    }

                    // Then we calculate the distances between the intermediary
                    // ground truths
                    for (auto iterator =
                             incoming_ground_truth_data_points.begin();
                         iterator !=
                         incoming_ground_truth_data_points.end() - 1;
                         iterator++) {

                        const GroundTruthDataPoint& first  = *iterator;
                        const GroundTruthDataPoint& second = *(iterator + 1);

                        track_length +=
                            (q_worldaxes_eurocreferenceaxes * second.position -
                             q_worldaxes_eurocreferenceaxes * first.position)
                                .norm();
                    }
                }

                GroundTruthDataPoint ground_truth_data_point =
                    incoming_ground_truth_data_points.back();

                current_ground_truth.position =
                    q_worldaxes_eurocreferenceaxes *
                    ground_truth_data_point.position;

                current_ground_truth.orientation =
                    q_worldaxes_eurocreferenceaxes *
                    ground_truth_data_point.orientation *
                    q_worldaxes_eurocbodyaxes.conjugate();

                current_ground_truth.velocity =
                    q_worldaxes_eurocreferenceaxes *
                    ground_truth_data_point.velocity;

                current_ground_truth.acceleration_bias =
                    q_worldaxes_eurocreferenceaxes *
                    ground_truth_data_point.acceleration_bias;

                current_ground_truth.angular_velocity_bias =
                    q_worldaxes_eurocreferenceaxes *
                    ground_truth_data_point.angular_velocity_bias;

                if (!got_ground_truth) {
                    got_ground_truth = true;
                }
            }

            for (const ImuDataPoint& imu_data_point :
                 incoming_imu_data_points) {

                if (imu_buffer.full()) {
                    imu_buffer.erase(imu_buffer.begin(),
                                     imu_buffer.begin() +
                                         imu_buffer.size() / 2);
                }

                imu_buffer.push_back(imu_data_point);
            }

            camera_frame_index++;

            feature_observations.observations.clear();

            profile::start("pipeline");
            profile::start("frontend");
            const bool process = frontend::process_image(image_data_point,
                                                         imu_buffer,
                                                         feature_observations);
            profile::end();

            if (process) {

                profile::start("backend");
                backend::process_feature_observations(feature_observations,
                                                      imu_buffer);
                profile::end();

                evaluator.add_feature_sample(
                    feature_observations.observations.size(),
                    backend::get_feature_tracks().size());
            }

            const uint32_t duration = profile::end();

            if (backend::has_initialised()) {

                if (!has_alignment_transformation && got_ground_truth) {

                    // We set a transformation in order to match the
                    // ground truth body frame and the estimated body
                    // frame

                    T_w_b0.translation() = current_ground_truth.position;

                    // Here we find the difference between the ground
                    // truth body frame and the estimated body frame, so
                    // that we can align the frames when this is
                    // multiplied with the estimated orientation further
                    // on
                    T_w_b0.linear() = (current_ground_truth.orientation *
                                       (q_worldaxes_eurocreferenceaxes *
                                        backend::get_imu_state().orientation *
                                        q_worldaxes_eurocbodyaxes.conjugate())
                                           .conjugate())
                                          .toRotationMatrix();

                    has_alignment_transformation = true;

                    logger::infof("Initialised at frame: %d\r\n",
                                  camera_frame_index);
                }

                state = backend::get_imu_state();

                // Align with visualisation frame
                state.position = q_worldaxes_eurocreferenceaxes *
                                 state.position;

                state.orientation = q_worldaxes_eurocreferenceaxes *
                                    state.orientation *
                                    q_worldaxes_eurocbodyaxes.conjugate();

                if (got_ground_truth && has_alignment_transformation) {

                    Eigen::Matrix<float,
                                  backend::IMU_ERROR_STATE_SIZE,
                                  backend::IMU_ERROR_STATE_SIZE>
                        state_covariance = backend::get_imu_state_covariance();

                    evaluator.add_state_sample(
                        current_ground_truth.orientation,
                        Eigen::Quaternionf(T_w_b0.linear() * state.orientation),
                        current_ground_truth.velocity,
                        q_worldaxes_eurocreferenceaxes * state.velocity,
                        current_ground_truth.position,
                        T_w_b0.translation() + T_w_b0.linear() * state.position,
                        current_ground_truth.angular_velocity_bias,
                        q_worldaxes_eurocreferenceaxes *
                            state.angular_velocity_bias,
                        current_ground_truth.acceleration_bias,
                        q_worldaxes_eurocreferenceaxes *
                            state.acceleration_bias,
                        state_covariance.block<15, 15>(0, 0));
                }
            }

            const Pose estimate_aligned(T_w_b0.translation() +
                                            T_w_b0.linear() * state.position,
                                        T_w_b0.linear() * state.orientation);

            if (has_alignment_transformation) {

                data_interface::transmit_pose(estimate_aligned.translation,
                                              estimate_aligned.orientation);

                printf("%lu, %f, %f, %f, %f, %f, %f, %f\r\n",
                       duration,
                       track_length,
                       current_ground_truth.position.x(),
                       current_ground_truth.position.y(),
                       current_ground_truth.position.z(),
                       estimate_aligned.translation.x(),
                       estimate_aligned.translation.y(),
                       estimate_aligned.translation.z());
            }
        }

        profile::report();
        printf("\r\n");

        evaluator.print_metrics();
        evaluator.nees.print_average();

        data_interface::deinitialise();
    }
}
