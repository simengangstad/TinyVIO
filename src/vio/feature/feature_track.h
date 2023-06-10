#ifndef FEATURE_TRACK_H_
#define FEATURE_TRACK_H_

#include <etl/map.h>
#include <etl/vector.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include "Eigen/Dense"
#pragma GCC diagnostic pop

#include "augmented_imu_state.h"
#include "imu_state.h"
#include "vio_typedefs.h"

class FeatureTrack {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    /**
     * @brief Optimisation configuration for Levenberg-Marquardt, used when
     * triangulating the feature track.
     */
    struct OptimisationConfig {
        double translation_threshold;
        double huber_epsilon;
        double estimation_precision;
        double initial_damping;

        int outer_loop_max_iteration;
        int inner_loop_max_iteration;

        OptimisationConfig()
            : translation_threshold(0.2), huber_epsilon(0.01),
              estimation_precision(5e-7), initial_damping(1e-3),
              outer_loop_max_iteration(10), inner_loop_max_iteration(10) {}
    };

    /**
     * @brief The identifier of the feature track.
     */
    FeatureIdentifier identifier;

    /**
     * @brief The observations of the feature track.
     */
    etl::map<StateIdentifier, Eigen::Vector2f, MAX_NUMBER_OF_OBSERVATIONS>
        observations;

    /**
     * @brief The observation velocities of the feature track.
     */
    etl::map<StateIdentifier, Eigen::Vector2f, MAX_NUMBER_OF_OBSERVATIONS>
        observations_velocity;

    // Optimization configuration for solving the 3d position.
    static OptimisationConfig optimisation_config;

    /**
     * @brief The 3D position in world frame of the feature triangulated from
     * the feature track.
     */
    Eigen::Vector3f position = Eigen::Vector3f::Zero();

    /**
     * @brief Whether the feature track has been triangulated.
     */
    bool has_been_triangulated = false;

    /**
     * @brief The state identifier of the anchor frame used in the
     * triangulation.
     */
    StateIdentifier identifier_anchor = -1;

    /**
     * @brief The inverse depth of the triangulated feature in the anchor frame.
     */
    float inverse_depth = 0.0;

    /**
     * @brief The homogeneous corrected observation in the anchor frame.
     */
    Eigen::Vector3f observation_anchor_homogeneous;

    FeatureTrack() : identifier(0) {}

    explicit FeatureTrack(const FeatureIdentifier& identifier_)
        : identifier(identifier_) {}

    /**
     * @brief Check the camera poses to ensure there is enough translation
     *        to triangulate the feature track into a 3D position.
     *
     * @param augmented_imu_states The sliding window of the augmented IMU
     * states (the states where the camera measurement is taken).
     * @param is_currently_tracked Whether the feature is currently tracked.
     *
     * @return True if the translation between the augmented IMU
     * poses is sufficient.
     */
    [[nodiscard]] inline bool has_enough_movement_to_triangulate(
        const AugmentedIMUStatesMap& augmented_imu_states,
        bool is_currently_tracked) const;

    /**
     * @brief Attempt to triangulate the feature track into a 3D position.
     *
     * @param augmented_imu_states The sliding window of augmented IMU states
     * (the states where the camera measurement is taken).
     * @param current_state_identifier The identifier of the current state in
     * the filter, will not be used in the triangulation.
     *
     * @return True if the triangulation was successful.
     */
    inline bool
    attempt_triangulation(const AugmentedIMUStatesMap& augmented_imu_states,
                          const StateIdentifier& current_state_identifier);

  private:
    /**
     * @brief Compute the residual/cost which is used in the optimisation to
     * triangulate the feature from the feature track.
     *
     * @param T_ci_cn Transformation from the last camera pose in the track to
     * the camera frame associated with the observation @p z.
     * @param parameters The current estimate of the parameters in the
     * optimisation.
     * @param z The observed feature in the camera frame ci.
     *
     * @return The squared norm of the residual.
     */
    inline float cost(const Eigen::Isometry3f& T_ci_cn,
                      const Eigen::Vector3f& parameters,
                      const Eigen::Vector2f& z) const;

    /**
     * @brief Jacobian of the given measurement with respect to the parameters
     * in the optimisation, where these parameters are used to find the
     * triangulated position in the anchor frame.
     *
     * @param T_ci_cn Transformation from the last camera pose in the track to
     * the camera frame associated with the observation @p z.
     * @param parameters The current estimate of the parameters in the
     * optimisation.
     * @param z The observed feature in the given camera frame ci.
     * @param out_J The jacobian will be placed in this matrix.
     * @param out_r The residual will be placed in this vector.
     * @param out_w The weight introduced by a huber kernel will be placed in
     * this variable.
     */
    inline void jacobian(const Eigen::Isometry3f& T_ci_c0,
                         const Eigen::Vector3f& parameters,
                         const Eigen::Vector2f& z,
                         Eigen::Matrix<float, 2, 3>& out_J,
                         Eigen::Vector2f& out_r,
                         float& out_w) const;

    /**
     * @brief Create an initial guess for the feature track's triangulated 3D
     * position using two camera frames and depth estimation.
     *
     * @param T_ci_cn Transformation between the two camera frames, from cn to
     * ci.
     * @param zi The feature observation in the ith frame.
     * @param zn The feature observation in the nth frame.
     * @param out_guess The guess will be placed in this vector.
     */
    inline void generate_initial_guess(const Eigen::Isometry3f& T_ci_cn,
                                       const Eigen::Vector2f& zi,
                                       const Eigen::Vector2f& zn,
                                       Eigen::Vector3f& out_guess) const;
};

bool FeatureTrack::has_enough_movement_to_triangulate(
    const AugmentedIMUStatesMap& augmented_imu_states,
    const bool is_currently_tracked) const {

    StateIdentifier first_pose_identifier = observations.begin()->first;
    StateIdentifier last_pose_identifier;

    // If the feature is currently tracked, we grab the last pose just
    // before it went out of track.
    if (is_currently_tracked) {
        last_pose_identifier = (--observations.end())->first;
    } else {
        last_pose_identifier = (--(--observations.end()))->first;
    }

    const Eigen::Isometry3f& first_pose =
        augmented_imu_states.find(first_pose_identifier)->second.T_world_camera;
    const Eigen::Isometry3f& last_pose =
        augmented_imu_states.find(last_pose_identifier)->second.T_world_camera;

    // Get the direction of the feature when it was first observed
    Eigen::Vector3f feature_direction(observations.begin()->second(0),
                                      observations.begin()->second(1),
                                      1.0);

    feature_direction = feature_direction / feature_direction.norm();

    // Transform to world frame
    feature_direction = first_pose.linear() * feature_direction;

    // Compute the translation between the first frame and the last frame.
    // Here we assume the first frame and the last frame will provide the
    // largest motion to speed up the checking process
    const Eigen::Vector3f translation = (last_pose.translation() -
                                         first_pose.translation());

    const double parallel_translation = translation.transpose() *
                                        feature_direction;

    const Eigen::Vector3f orthogonal_translation =
        translation - parallel_translation * feature_direction;

    if (orthogonal_translation.norm() >
        optimisation_config.translation_threshold) {
        return true;
    } else {
        return false;
    }
}

bool FeatureTrack::attempt_triangulation(
    const AugmentedIMUStatesMap& augmented_imu_states,
    const StateIdentifier& current_state_identifier) {

    // ----
    // First grab every every observation which has an associated state and is
    // not the current state
    // ----

    static etl::vector<Eigen::Isometry3f, MAX_NUMBER_OF_AUGMENTED_STATES>
        camera_transformations(0);
    camera_transformations.clear();

    static etl::vector<Eigen::Vector2f, MAX_NUMBER_OF_FEATURES> measurements(0);
    measurements.clear();

    etl::vector<StateIdentifier, MAX_NUMBER_OF_AUGMENTED_STATES>
        camera_identifiers(0);
    camera_identifiers.clear();

    for (const auto& observation : observations) {

        auto state_iterator = augmented_imu_states.find(observation.first);

        if (!augmented_imu_states.contains(observation.first)) {
            continue;
        }

        // Don't include the current state in the triangulation
        if (current_state_identifier == state_iterator->first) {
            continue;
        }

        measurements.push_back(observation.second);
        camera_transformations.push_back(state_iterator->second.T_world_camera);
        camera_identifiers.push_back(state_iterator->first);
    }

    // ----
    // Initialisation before Levenberg-Marquardt
    // ----

    // Modify every camera pose such that they represent transformations with
    // respect to the last camera frame, which we specify as the anchor frame
    const Eigen::Isometry3f T_w_cn =
        camera_transformations[camera_transformations.size() - 1];

    for (auto& camera_transformation : camera_transformations) {

        // This effectively becomes:
        //
        // T_ci_cn = T_w_ci^-1 * T_w_cn = T_ci_w * T_w_cn
        camera_transformation = camera_transformation.inverse() * T_w_cn;
    }

    Eigen::Vector3f initial_position(0.0, 0.0, 0.0);

    if (!has_been_triangulated) {
        // Use the first and last camera frame for generating the initial guess
        // with the assumption that they have the largest baseline.
        generate_initial_guess(camera_transformations[0],
                               measurements[0],
                               measurements[camera_transformations.size() - 1],
                               initial_position);
    } else {
        // If this feature track has already been triangulated and we want to do
        // a new triangulation to refine the estimate, we transform the world
        // position to the last camera frame
        initial_position = T_w_cn.inverse() * position;
    }

    // ----
    // Levenberg-Marquardt optimisation for triangulation
    // ----
    Eigen::Vector3f solution(initial_position(0) / initial_position(2),
                             initial_position(1) / initial_position(2),
                             1.0f / initial_position(2));

    double lambda          = optimisation_config.initial_damping;
    int inner_loop_counter = 0;
    int outer_loop_counter = 0;
    bool is_cost_reduced;
    double delta_norm;

    // Compute the initial cost from the initial estimate
    float total_cost = 0.0;

    for (size_t i = 0; i < camera_transformations.size(); ++i) {
        total_cost +=
            cost(camera_transformations[i], solution, measurements[i]);
    }

    do {
        Eigen::Matrix3f A = Eigen::Matrix3f::Zero();
        Eigen::Vector3f b = Eigen::Vector3f::Zero();

        for (size_t i = 0; i < camera_transformations.size(); ++i) {
            Eigen::Matrix<float, 2, 3> J;
            Eigen::Vector2f r;
            float w;

            jacobian(camera_transformations[i],
                     solution,
                     measurements[i],
                     J,
                     r,
                     w);

            if (w == 1) {
                A += J.transpose() * J;
                b += J.transpose() * r;
            } else {
                double w_square = w * w;
                A += w_square * J.transpose() * J;
                b += w_square * J.transpose() * r;
            }
        }

        // Solve for the delta that can reduce the total cost
        do {
            Eigen::Matrix3f damper       = lambda * Eigen::Matrix3f::Identity();
            Eigen::Vector3f delta        = (A + damper).ldlt().solve(b);
            Eigen::Vector3f new_solution = solution - delta;
            delta_norm                   = delta.norm();

            float new_cost = 0.0;
            for (size_t i = 0; i < camera_transformations.size(); ++i) {
                new_cost += cost(camera_transformations[i],
                                 new_solution,
                                 measurements[i]);
            }

            if (new_cost < total_cost) {
                is_cost_reduced = true;
                solution        = new_solution;
                total_cost      = new_cost;
                lambda          = lambda / 10 > 1e-10 ? lambda / 10 : 1e-10;
            } else {
                is_cost_reduced = false;
                lambda          = lambda * 10 < 1e12 ? lambda * 10 : 1e12;
            }

        } while (inner_loop_counter++ <
                     optimisation_config.inner_loop_max_iteration &&
                 !is_cost_reduced);

        inner_loop_counter = 0;

    } while (outer_loop_counter++ <
                 optimisation_config.outer_loop_max_iteration &&
             delta_norm > optimisation_config.estimation_precision);

    // Construct the final solution from the parameters in the solution
    Eigen::Vector3f final_position(solution(0) / solution(2),
                                   solution(1) / solution(2),
                                   1.0f / solution(2));

    // ----
    // Check result for being in front of every camera frame
    // ----

    for (const auto& T_ci_cn : camera_transformations) {
        Eigen::Vector3f position_camera_frame =
            T_ci_cn.linear() * final_position + T_ci_cn.translation();
        if (position_camera_frame.z() <= 0) {
            has_been_triangulated = false;
            return false;
        }
    }

    const float normalized_cost = total_cost /
                                  (2.0f * (float)camera_transformations.size() *
                                   (float)camera_transformations.size());

    if (normalized_cost > 4.7673e-04) {
        has_been_triangulated = false;
        return false;
    }

    has_been_triangulated = true;

    position = T_w_cn.linear() * final_position + T_w_cn.translation();

    identifier_anchor = camera_identifiers[camera_identifiers.size() - 1];

    inverse_depth = 1 / final_position(2);

    observation_anchor_homogeneous = Eigen::Vector3f(
        final_position.x() * inverse_depth,
        final_position.y() * inverse_depth,
        1.0f);

    return true;
}

float FeatureTrack::cost(const Eigen::Isometry3f& T_ci_cn,
                         const Eigen::Vector3f& parameters,
                         const Eigen::Vector2f& z) const {

    const float& alpha = parameters(0);
    const float& beta  = parameters(1);
    const float& rho   = parameters(2);

    const Eigen::Vector3f h = T_ci_cn.linear() *
                                  Eigen::Vector3f(alpha, beta, 1.0) +
                              rho * T_ci_cn.translation();

    const float& h1 = h(0);
    const float& h2 = h(1);
    const float& h3 = h(2);

    const Eigen::Vector2f z_hat(h1 / h3, h2 / h3);

    return (z - z_hat).squaredNorm();
}

void FeatureTrack::jacobian(const Eigen::Isometry3f& T_ci_cn,
                            const Eigen::Vector3f& parameters,
                            const Eigen::Vector2f& z,
                            Eigen::Matrix<float, 2, 3>& out_J,
                            Eigen::Vector2f& out_r,
                            float& out_w) const {

    const float& alpha = parameters(0);
    const float& beta  = parameters(1);
    const float& rho   = parameters(2);

    const Eigen::Vector3f h = T_ci_cn.linear() *
                                  Eigen::Vector3f(alpha, beta, 1.0) +
                              rho * T_ci_cn.translation();

    const float& h1 = h(0);
    const float& h2 = h(1);
    const float& h3 = h(2);

    Eigen::Matrix3f W;

    W.leftCols<2>()  = T_ci_cn.linear().leftCols<2>();
    W.rightCols<1>() = T_ci_cn.translation();

    out_J.row(0) = -((1.0f / h3) * W.row(0) - (h1 / (h3 * h3)) * W.row(2));
    out_J.row(1) = -((1.0f / h3) * W.row(1) - (h2 / (h3 * h3)) * W.row(2));

    const Eigen::Vector2f z_hat(h1 / h3, h2 / h3);

    out_r = z - z_hat;

    float error = out_r.norm();

    if (error <= optimisation_config.huber_epsilon) {
        out_w = 1.0;
    } else {
        out_w = sqrt(2.0 * optimisation_config.huber_epsilon / error);
    }
}

void FeatureTrack::generate_initial_guess(const Eigen::Isometry3f& T_ci_cn,
                                          const Eigen::Vector2f& zi,
                                          const Eigen::Vector2f& zn,
                                          Eigen::Vector3f& out_guess) const {

    // Solve for the depth by means of a system of equations on the form:
    // A x = b
    // A^T A x = A^T b
    // x = (A^T A)^-1 A^T b
    //
    // Where x is the depth

    const Eigen::Vector3f m = T_ci_cn.linear() *
                              Eigen::Vector3f(zn.x(), zn.y(), 1.0f);

    const Eigen::Vector2f A(m.x() - zi.x() * m.z(), m.y() - zi.y() * m.z());

    const Eigen::Vector2f b(
        zi.x() * T_ci_cn.translation().z() - T_ci_cn.translation().x(),
        zi.y() * T_ci_cn.translation().z() - T_ci_cn.translation().y());

    const float depth = (A.transpose() * A).inverse() * A.transpose() * b;

    out_guess.x() = zn.x() * depth;
    out_guess.y() = zn.y() * depth;
    out_guess.z() = depth;
}

#endif
