#include "feature_tracking.h"

#include <math.h>

#ifndef CPU_MIMXRT1176DVMAA
#include "visualization.h"
#endif

namespace frontend {

    using namespace Eigen;

    void track_features(
        PatchPyramid& previous_patch_pyramid,
        ImagePyramid& next_image_pyramid,
        etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES>& previous_features,
        etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES>& next_features,
        etl::vector<uint8_t, MAX_NUMBER_OF_FEATURES>& inliers,
        const bool use_next_feature_estimate,
        const int_fast32_t max_iterations,
        const float incremental_flow_threshold,
        const float eigenvalue_similarity_threshold) {

        Matrix<float, 2, PATCH_SIZE * PATCH_SIZE> AT;
        Matrix<float, PATCH_SIZE * PATCH_SIZE, 1> b;
        Matrix2f S = Matrix2f::Zero();

        for (int_fast32_t feature_index = 0;
             feature_index < (int_fast32_t)previous_features.size();
             feature_index++) {

            if (!inliers[feature_index]) {
                continue;
            }

            Vector2f pyramid_level_flow = Vector2f(0.0f, 0.0f);

            for (int pyramid_level = PYRAMID_LEVELS - 1; pyramid_level >= 0;
                 pyramid_level--) {

                Image* next_image_at_pyramid_level = next_image_pyramid.at(
                    pyramid_level);

                Patch* previous_image_patch =
                    previous_patch_pyramid.at(pyramid_level, feature_index);

                Patch previous_patch_dx = previous_image_patch->dx();
                Patch previous_patch_dy = previous_image_patch->dy();

                // LK is based on the following:
                //
                // A * v = b
                //
                // Where A contains the x and y gradients from the previous
                // image, v is the flow vector and b is the time derivatives
                // with respect to the previous image and the current image
                //
                // The equations is solved by taking the pseudo-inverse:
                //
                // A * v        = b
                // A^T * A * v  = A^T * b
                // v            = (A^T * A)^-1 * A^T * b
                // v            = S^-1 * A^T * b
                //
                // Where we let S = A^T * A, which is the structure tensor
                //
                // A will thus be on the following form:
                //
                //     [ I0x(p_1) I0y(p_1) ]
                // A = [ I0x(p_2) I0y(p_2) ]
                //     [       ...         ]
                //     [ I0x(p_n) I0y(p_n) ]
                //
                // Whereas b will be on the following form (the time derivative
                // is equal to the intensity difference between the images)
                //
                //     [I1(p_1) - I0(p_1)]
                // b = [I1(p_2) - I0(p_2)]
                //     [       ...       ]
                //     [I1(p_n) - I0(p_n)]
                //
                // clang-format off
                //
                // This can be rewritten as:
                //
                //     [ Sum(I0x(p_i)^2)            Sum(I0x(p_i) * I0y(p_i)) ]^-1  [ -Sum(I0x(p_i) * (I1(p_i) - I0(p_i))) ]
                // v = [                                                     ]     [                                      ]
                //     [ Sum(I0x(p_i) * I0y(p_i))   Sum(I0y(p_i)^2)          ]     [ -Sum(I0y(p_i) * (I1(p_i) - I0(p_i))) ]
                //
                // clang-format on

                S.setZero();

                for (int_fast32_t j = 0; j < PATCH_SIZE; j++) {
                    const float* Ix = previous_patch_dx.row(j);
                    const float* Iy = previous_patch_dy.row(j);

                    for (int_fast32_t i = 0; i < PATCH_SIZE; i++) {

                        S(0, 0) += Ix[i] * Ix[i];
                        S(1, 1) += Iy[i] * Iy[i];

                        S(0, 1) += Ix[i] * Iy[i];

                        AT(0, j * PATCH_SIZE + i) = Ix[i];
                        AT(1, j * PATCH_SIZE + i) = Iy[i];
                    }
                }

                // Off-diagonal entries are equal
                S(1, 0) = S(0, 1);

                // Just jump to next feature if structure tensor is
                // non-invertible
                if (S.determinant() == 0.0f) {
                    continue;
                }

                Matrix2f Sinv = S.inverse();

                // Check eigenvalues of the structure tensor S, where at this
                // point, since S is invertible, we should have that:
                //
                //      lambda_1 >= lambda_2 > 0
                //
                // If lambda_1 / lambda_2 is too large, this means that the
                // point is an edge and should be discarded

                const Vector2f eigenvalues = S.eigenvalues().real();
                const float l1 = fmax(eigenvalues(0), eigenvalues(1));
                const float l2 = fmin(eigenvalues(0), eigenvalues(1));

                if (l1 / l2 > eigenvalue_similarity_threshold) {
                    inliers[feature_index] = 0;
                    continue;
                }

                int_fast32_t iterations = 0;

                Vector2f flow(0.0f, 0.0f);
                Vector2f incremental_flow(0.0f, 0.0f);

                float initial_x = previous_image_patch->origin[0];
                float initial_y = previous_image_patch->origin[1];

                if (use_next_feature_estimate) {
                    initial_x = (next_features[feature_index][0] -
                                 ((float)PATCH_SIZE) / 2.0f) /
                                powf(2, pyramid_level);
                    initial_y = (next_features[feature_index][1] -
                                 ((float)PATCH_SIZE) / 2.0f) /
                                powf(2, pyramid_level);
                }

                do {

                    const Vector2f total_flow(flow + pyramid_level_flow);

                    Patch next_image_patch(initial_x + total_flow[0],
                                           initial_y + total_flow[1],
                                           *next_image_at_pyramid_level);

                    Patch patch_dt = Patch::dt(*previous_image_patch,
                                               next_image_patch);

                    for (int_fast32_t j = 0; j < PATCH_SIZE; j++) {

                        const float* It = patch_dt.row(j);

                        for (int_fast32_t i = 0; i < PATCH_SIZE; i++) {

                            b(j * PATCH_SIZE + i, 0) = -It[i];
                        }
                    }

                    // Important for run-time that that AT * b is performed
                    // first, so the parantheses should be there
                    incremental_flow = Sinv * (AT * b);

                    flow += incremental_flow;

                } while (incremental_flow.norm() > incremental_flow_threshold &&
                         iterations++ < max_iterations);

                Vector2f pyramid_level_displacement(flow + pyramid_level_flow);

                if (0 < pyramid_level) {
                    pyramid_level_flow = 2 * pyramid_level_displacement;

                } else {
                    next_features[feature_index][0] =
                        (initial_x + ((float)PATCH_SIZE) / 2.0f +
                         pyramid_level_displacement[0]);

                    next_features[feature_index][1] =
                        (initial_y + ((float)PATCH_SIZE) / 2.0f +
                         pyramid_level_displacement[1]);

                    const Vector2f& feature = next_features[feature_index];

                    if ((feature[0] < 0) || (feature[1] < 0) ||
                        (feature[0] > next_image_pyramid.at(0)->width - 1) ||
                        (feature[1] > next_image_pyramid.at(0)->height - 1)) {

                        inliers[feature_index] = 0;
                    }
                }
            }
        }
    }
}
