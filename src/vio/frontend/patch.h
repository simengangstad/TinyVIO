/**
 * @brief This module represents a patch, which is an excerpt of an image as
 * well as a patch pyramid, which represents an excerpt of an image at different
 * levels of scale.
 */

#ifndef PATCH_H
#define PATCH_H

#include <stddef.h>
#include <stdint.h>

#include "image.h"
#include "vio_config.h"

#include <etl/vector.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

constexpr uint16_t PATCH_SIZE             = 7;
constexpr uint16_t PATCH_SIZE_WITH_BORDER = PATCH_SIZE + 2;

namespace frontend {
    /**
     * @brief A patch of a given image.
     *
     * @note The data of the patch is stored in floats, in order to make
     * gradients more precise.
     */
    struct Patch {
      private:
        /**
         * @brief The data of the patch. Note that we store a border around
         * the patch in order e.g. be able to take the gradient of the whole
         * patch (excluding the border), without having omit pixels. We also
         * need the border in order to do bi-linear interpolation without losing
         * information.
         */
        float data[PATCH_SIZE_WITH_BORDER * PATCH_SIZE_WITH_BORDER];

      public:
        /**
         * @brief The upper left start point of the patch. Note that is is
         * not from the border, but the start of the actual patch.
         */
        Eigen::Vector2f origin;

        /**
         * @brief Initializes the patch with empty data.
         */
        Patch() {
            origin.setZero();
            memset(data, 0, sizeof(data));
        }

        /**
         * @brief Initializes the Patch with a start point and a reference
         * to the image where the data should be extracted from. Applies
         * bi-linear filtering.
         *
         * @param x Start position of patch in x direction.
         * @param y Start position of patch in y direction.
         * @param image Where to grab the values for the patch from.
         */
        Patch(const float x, const float y, const Image& image);

        /**
         * @return A pointer to the @p index row. The pointer will point to
         * the value after the border at each row.
         */
        float* row(int index);

        /**
         * @brief Takes the x gradient of the patch with convolution using a
         * Sobel kernel.
         */
        Patch dx();

        /**
         * @brief Takes the y gradient of the patch with convolution using a
         * Sobel kernel.
         */
        Patch dy();

        /**
         * @brief Takes the time gradient from @p first and @p second.
         */
        static Patch dt(const Patch& first, const Patch& second);

        void print();
    };

    /**
     * @brief Contains N pyramids of patches, where each level in the
     * pyramid is down-sampled by half for each axis.
     */
    struct PatchPyramid {
      private:
        /**
         * @brief Max number of patches per level.
         */
        size_t max_number_of_patches = 0;

        /**
         * The patches in the pyramid.
         */
        Patch* patch_buffer;

      public:
        /**
         * @brief Initializes the patch pyramid.
         *
         * @param patch_buffer Pointer to the buffer where the patches will be
         * stored.
         * @param max_number_of_patches Maximum number of patches per level in
         * the patch pyramid.
         */
        PatchPyramid(Patch* patch_buffer, const size_t max_number_of_patches);

        /**
         * @brief Constructs a patch pyramid with N pyramids equal to the
         * amount of features and M pyramid levels.
         *
         * @param image_pyramid The pyramid to construct the patches from.
         * @param features The feature points which will be the centre of the
         * patches.
         */
        void construct(ImagePyramid& image_pyramid,
                       const etl::vector<Eigen::Vector2f,
                                         MAX_NUMBER_OF_FEATURES>& features);

        /**
         * @return Pointer to the patch at the given @p level and @p index.
         */
        Patch* at(const size_t level, const size_t index);
    };
}

#endif
