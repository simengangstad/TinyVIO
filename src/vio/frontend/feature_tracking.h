#ifndef FEATURE_TRACKING_H
#define FEATURE_TRACKING_H

#include "image.h"
#include "patch.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include <etl/vector.h>

#include <stddef.h>

namespace frontend {

    /**
     * @brief Tracks features from a image to another.
     *
     * @param previous_patch_pyramid The previous patch pyramid where the
     * which contains patches around the features.
     * @param next_image_pyramid The pyramid of the image where the features
     * are to be found.
     * @param previous_features The features captured in the previous frame.
     * @param next_features Buffer for where the features found in @p
     * next_image are placed after tracking.
     * @param use_next_feature_estimate If there is an estimate for the next
     * features, and the estimates are filled in @p next_features, then the
     * algorithm will use this as a starting point.
     * @param max_iterations The maximum number of iterations for Lucas-Kanade
     * flow calculation at a given pyramid level and for a given feature.
     * @param incremental_flow_treshold If the incremental flow from one
     * iteration of Lucas-Kanade to the next is below this value, the flow
     * calculation for that specific pyramid level and feature will end.
     * @param eigenvalue_similarity_threshold If the eigenvalues of the
     * structure tensor of the patch divided by each other is above this value,
     * the feature is discared as the eigenvalues then are not of the same
     * magnitude.
     */
    void track_features(
        PatchPyramid& previous_patch_pyramid,
        ImagePyramid& next_image_pyramid,
        etl::vector<Eigen::Vector2f, MAX_NUMBER_OF_FEATURES>& previous_features,
        etl::vector<Eigen::Vector2f, MAX_NUMBER_OF_FEATURES>& next_features,
        etl::vector<uint8_t, MAX_NUMBER_OF_FEATURES>& inliers,
        const bool use_next_feature_estimate        = false,
        const int_fast32_t max_iterations           = 30,
        const float incremental_flow_treshold       = 0.01f,
        const float eigenvalue_similarity_threshold = 20.0f);

}

#endif
