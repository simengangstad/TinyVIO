#ifndef FEATURE_EXTRACTION_H_
#define FEATURE_EXTRACTION_H_

#include <stddef.h>
#include <stdint.h>

#include "image.h"
#include "vio_config.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include <etl/vector.h>

namespace frontend {

    /**
     * @brief Performs FAST on a given image.
     *
     * @param image_buffer [in] Buffer for the image.
     * @param width [in] The width of the image.
     * @param height [in] The height of the image.
     * @param threshold [in] Threshold used for determining if a pixel is a
     * corner/feature or not.
     * @param out_features [out] Features/corners detected are placed in
     * this buffer.
     */
    void extract_features(
        const uint8_t* image_buffer,
        const int_fast32_t width,
        const int_fast32_t height,
        const uint8_t threshold,
        etl::vector<Eigen::Vector2f, MAX_NUMBER_OF_FEATURES>& out_features);
}

#endif
