#include "evaluator.h"

void Evaluator::NEES::add_sample(
    const Eigen::Vector3f& angle_axis_error,
    const Eigen::Vector3f& velocity_error,
    const Eigen::Vector3f& position_error,
    const Eigen::Vector3f& gyro_bias_error,
    const Eigen::Vector3f accelerometer_bias_error,
    const Eigen::Matrix<float, 15, 15>& state_covariance) {

    current_orientation = angle_axis_error.transpose() *
                          state_covariance.block<3, 3>(0, 0).ldlt().solve(
                              angle_axis_error);

    current_velocity = velocity_error.transpose() *
                       state_covariance.block<3, 3>(3, 3).ldlt().solve(
                           velocity_error);

    current_position = position_error.transpose() *
                       state_covariance.block<3, 3>(6, 6).ldlt().solve(
                           position_error);

    current_gyro_bias = gyro_bias_error.transpose() *
                        state_covariance.block<3, 3>(9, 9).ldlt().solve(
                            gyro_bias_error);

    current_accelerometer_bias =
        accelerometer_bias_error.transpose() *
        state_covariance.block<3, 3>(12, 12).ldlt().solve(
            accelerometer_bias_error);

    accumulated_orientation += current_orientation;
    accumulated_velocity += current_velocity;
    accumulated_position += current_position;
    accumulated_gyro_bias += current_gyro_bias;
    accumulated_accelerometer_bias += current_accelerometer_bias;

    samples++;
}

void Evaluator::NEES::print_average() {

    if (samples == 0) {
        return;
    }

    printf("ANEES (%d): %f, %f, %f, %f, %f\r\n",
           samples,
           accumulated_orientation / static_cast<double>(samples),
           accumulated_velocity / static_cast<double>(samples),
           accumulated_position / static_cast<double>(samples),
           accumulated_gyro_bias / static_cast<double>(samples),
           accumulated_accelerometer_bias / static_cast<double>(samples));
}

void Evaluator::NEES::print_current() {
    printf("NEES: %f, %f, %f, %f, %f\r\n",
           current_orientation,
           current_velocity,
           current_position,
           current_gyro_bias,
           current_accelerometer_bias);
}

void Evaluator::add_state_sample(
    const Eigen::Quaternionf& true_orientation,
    const Eigen::Quaternionf& estimated_orientation,
    const Eigen::Vector3f& true_velocity,
    const Eigen::Vector3f& estimated_velocity,
    const Eigen::Vector3f& true_position,
    const Eigen::Vector3f& estimated_position,
    const Eigen::Vector3f& true_gyro_bias,
    const Eigen::Vector3f& estimated_gyro_bias,
    const Eigen::Vector3f& true_accelerometer_bias,
    const Eigen::Vector3f& estimated_accelerometer_bias,
    const Eigen::Matrix<float, 15, 15>& state_covariance) {

    // --- Orientation ---

    const Eigen::Quaternionf orientation_error =
        true_orientation * estimated_orientation.conjugate();

    const Eigen::AngleAxisf axis = Eigen::AngleAxisf(orientation_error);
    const Eigen::Vector3f angle_axis_error = axis.angle() * axis.axis();

    orientation_error_metric.append(angle_axis_error.norm());

    // --- Velocity ---

    const Eigen::Vector3f velocity_error = true_velocity - estimated_velocity;

    velocity_error_metric.append(velocity_error.norm());

    // --- Position ---

    const Eigen::Vector3f position_error = true_position - estimated_position;

    position_error_metric.append(position_error.norm());

    // --- Gyro bias ---

    const Eigen::Vector3f gyro_bias_error = true_gyro_bias -
                                            estimated_gyro_bias;

    gyro_bias_error_metric.append(gyro_bias_error.norm());

    // --- Accelerometer bias ---

    const Eigen::Vector3f accelerometer_bias_error =
        true_accelerometer_bias - estimated_accelerometer_bias;

    accelerometer_bias_error_metric.append(accelerometer_bias_error.norm());

    // --- NEES ---

    nees.add_sample(angle_axis_error,
                    velocity_error,
                    position_error,
                    gyro_bias_error,
                    accelerometer_bias_error,
                    state_covariance);
}

void Evaluator::add_feature_sample(const size_t& number_of_feature_observations,
                                   const size_t& number_of_feature_tracks) {
    feature_observations_metric.append(number_of_feature_observations);
    feature_tracks_metric.append(number_of_feature_tracks);
}

void Evaluator::print_metrics() {

    // --- Position ---
    printf("--- Position ---\r\n");
    printf("RMSE: %f\r\n", position_error_metric.rmse());
    printf("Average error: %f\r\n", position_error_metric.average());
    printf("Max error: %f\r\n", position_error_metric.maximum);
    printf("Final error: %f\r\n", position_error_metric.current);
    printf("\r\n");

    // --- Orientation ---
    printf("--- Orientation ---\r\n");
    printf("RMSE: %f\r\n", orientation_error_metric.rmse());
    printf("Average error: %f\r\n", orientation_error_metric.average());
    printf("Max error: %f\r\n", orientation_error_metric.maximum);
    printf("Final error: %f\r\n", orientation_error_metric.current);
    printf("\r\n");

    // --- Velocity ---
    printf("--- Velocity ---\r\n");
    printf("RMSE: %f\r\n", velocity_error_metric.rmse());
    printf("Average error: %f\r\n", velocity_error_metric.average());
    printf("Max error: %f\r\n", velocity_error_metric.maximum);
    printf("Final error: %f\r\n", velocity_error_metric.current);
    printf("\r\n");

    // --- Gyro bias ---
    printf("--- Gyro bias ---\r\n");
    printf("RMSE: %f\r\n", gyro_bias_error_metric.rmse());
    printf("Average error: %f\r\n", gyro_bias_error_metric.average());
    printf("Max error: %f\r\n", gyro_bias_error_metric.maximum);
    printf("Final error: %f\r\n", gyro_bias_error_metric.current);
    printf("\r\n");

    // --- Accelerometer bias ---
    printf("--- Accelerometer bias ---\r\n");
    printf("RMSE: %f\r\n", accelerometer_bias_error_metric.rmse());
    printf("Average error: %f\r\n", accelerometer_bias_error_metric.average());
    printf("Max error: %f\r\n", accelerometer_bias_error_metric.maximum);
    printf("Final error: %f\r\n", accelerometer_bias_error_metric.current);
    printf("\r\n");

    // --- Features ---
    printf("--- Feature observations ---\r\n");
    printf("Average per frame: %f\r\n", feature_observations_metric.average());
    printf("Maximum: %f\r\n", feature_tracks_metric.maximum);
    printf("\r\n");

    printf("--- Feature tracks ---\r\n");
    printf("Average per frame: %f\r\n", feature_tracks_metric.average());
    printf("Maximum: %f\r\n", feature_tracks_metric.maximum);
}
