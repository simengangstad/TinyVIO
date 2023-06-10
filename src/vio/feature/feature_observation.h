#ifndef FEATURE_OBSERVATION_H
#define FEATURE_OBSERVATION_H

#include <etl/vector.h>

#include "vio_config.h"
#include "vio_typedefs.h"

/**
 * @brief Signifies a single feature observation, with its current coordinate
 * and velocity and the initial coordinate and velocity.
 */
class FeatureObservation {
  public:
    /**
     * @brief Identifier of the feature.
     */
    FeatureIdentifier identifier = 0;

    /**
     * @brief Normalised coordinate.
     */
    float u = 0.0f, v = 0.0f;

    /**
     * @brief Normalised initial coordinate.
     */
    float u_init = 0.0f, v_init = 0.0f;

    /**
     * @brief Normalised coordinate velocity.
     */
    float u_velocity = 0.0f, v_velocity = 0.0f;

    /**
     * @brief Initial normalised coordinate velocity.
     */
    float u_init_velocity = 0.0f, v_init_velocity = 0.0f;
};

/**
 * @brief Convenience container for a range of feature observations at a given
 * camera/image frame time point.
 */
class FeatureObservations {
  public:
    /**
     * @brief Timestamp of the feature observations.
     */
    double timestamp = 0.0;

    /**
     * @brief The tracked and newly detected features from processing a
     * image in the frontend.
     */
    etl::vector<FeatureObservation, MAX_NUMBER_OF_FEATURES> observations{};
};

#endif
