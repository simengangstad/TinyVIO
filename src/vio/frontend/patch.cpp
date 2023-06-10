#include "patch.h"

#include <etl/vector.h>

#ifdef CPU_MIMXRT1176DVMAA

#include "board.h"
#include "logger.h"

#else

#define AT_QUICKACCESS_SECTION_CODE_ALLOW_INLINE(func) func

#endif

AT_QUICKACCESS_SECTION_CODE_ALLOW_INLINE(
    int_fast32_t inline clamp(int_fast32_t value,
                              int_fast32_t min,
                              int_fast32_t max)) {

    if (value < min) {
        return min;
    }

    if (value > max) {
        return max;
    }

    return value;
}

namespace frontend {

    using namespace Eigen;

    Patch::Patch(const float x, const float y, const Image& image) {

        origin.x() = x;
        origin.y() = y;

        int_fast32_t start_point_x = floor(x);
        int_fast32_t start_point_y = floor(y);

        // First want to copy over the data and append an extra
        // border to the left and top edge for interpolation
        uint8_t
            buffer[(PATCH_SIZE_WITH_BORDER + 2) * (PATCH_SIZE_WITH_BORDER + 2)];

        for (int_fast32_t j = 0; j < (int_fast32_t)PATCH_SIZE_WITH_BORDER + 2;
             j++) {

            // Make sure that the patch don't go outside bounds of the
            // image. If that is the case, just extrapolate with the
            // intensity values at the border.
            int_fast32_t ys = clamp(start_point_y + j - 2, 0, image.height - 1);

            for (int_fast32_t i = 0;
                 i < (int_fast32_t)PATCH_SIZE_WITH_BORDER + 2;
                 i++) {
                int_fast32_t xs =
                    clamp(start_point_x + i - 2, 0, image.width - 1);

                buffer[j * (PATCH_SIZE_WITH_BORDER + 2) + i] =
                    image.data[ys * image.width + xs];
            }
        }

        float alpha_x = fabs(x - start_point_x);
        float alpha_y = fabs(y - start_point_y);

        for (int_fast32_t j = 0; j < (int_fast32_t)PATCH_SIZE_WITH_BORDER;
             j++) {

            for (int_fast32_t i = 0; i < (int_fast32_t)PATCH_SIZE_WITH_BORDER;
                 i++) {

                // Store values for interpolating at sub-pixel level
                const float I =
                    buffer[(j + 1) * (PATCH_SIZE_WITH_BORDER + 2) + i + 1];
                const float I_x_shift =
                    buffer[(j + 1) * (PATCH_SIZE_WITH_BORDER + 2) + i + 2];
                const float I_y_shift =
                    buffer[(j + 2) * (PATCH_SIZE_WITH_BORDER + 2) + i + 1];
                const float I_xy_shift =
                    buffer[(j + 2) * (PATCH_SIZE_WITH_BORDER + 2) + i + 2];

                data[j * PATCH_SIZE_WITH_BORDER + i] =
                    (1.0f - alpha_x) * (1.0f - alpha_y) * I +
                    alpha_x * (1.0f - alpha_y) * I_x_shift +
                    (1.0f - alpha_x) * alpha_y * I_y_shift +
                    alpha_x * alpha_y * I_xy_shift;
            }
        }
    }

    float* Patch::row(int index) {
        return &data[(index + 1) * (PATCH_SIZE_WITH_BORDER) + 1];
    }

    Patch Patch::dx() {

        Patch destination;

        destination.origin = origin;

        float kernel[3][3] = {{-1, 1}, {-2, 2}, {-1, 1}};

        for (int_fast32_t j = 1; j < PATCH_SIZE + 1; j++) {
            for (int_fast32_t i = 1; i < PATCH_SIZE + 1; i++) {

                // Here we utilize the border so that we can take the gradient
                // on the patch without having to omit pixels
                destination.data[j * PATCH_SIZE_WITH_BORDER + i] =
                    data[(j - 1) * PATCH_SIZE_WITH_BORDER + (i - 1)] *
                        kernel[0][0] +
                    data[(j - 1) * PATCH_SIZE_WITH_BORDER + (i + 1)] *
                        kernel[0][1] +

                    data[j * PATCH_SIZE_WITH_BORDER + (i - 1)] * kernel[1][0] +
                    data[j * PATCH_SIZE_WITH_BORDER + (i + 1)] * kernel[1][1] +

                    data[(j + 1) * PATCH_SIZE_WITH_BORDER + (i - 1)] *
                        kernel[2][0] +
                    data[(j + 1) * PATCH_SIZE_WITH_BORDER + (i + 1)] *
                        kernel[2][1];
            }
        }

        return destination;
    }

    Patch Patch::dy() {

        Patch destination;

        destination.origin = origin;

        float kernel[2][3] = {{-1, -2, -1}, {1, 2, 1}};

        for (int_fast32_t j = 1; j < PATCH_SIZE + 1; j++) {
            for (int_fast32_t i = 1; i < PATCH_SIZE + 1; i++) {

                // Here we utilize the border so that we can take the gradient
                // on the patch without having to omit pixels
                destination.data[j * PATCH_SIZE_WITH_BORDER + i] =
                    data[(j - 1) * PATCH_SIZE_WITH_BORDER + (i - 1)] *
                        kernel[0][0] +
                    data[(j - 1) * PATCH_SIZE_WITH_BORDER + i] * kernel[0][1] +

                    data[(j - 1) * PATCH_SIZE_WITH_BORDER + (i + 1)] *
                        kernel[0][2] +
                    data[(j + 1) * PATCH_SIZE_WITH_BORDER + (i - 1)] *
                        kernel[1][0] +

                    data[(j + 1) * PATCH_SIZE_WITH_BORDER + i] * kernel[1][1] +
                    data[(j + 1) * PATCH_SIZE_WITH_BORDER + (i + 1)] *
                        kernel[1][2];
            }
        }

        return destination;
    }

    Patch Patch::dt(const Patch& first, const Patch& second) {

        Patch destination;

        destination.origin = first.origin;

        for (int_fast32_t j = 0; j < PATCH_SIZE_WITH_BORDER; j++) {
            for (int_fast32_t i = 0; i < PATCH_SIZE_WITH_BORDER; i++) {
                destination.data[j * PATCH_SIZE_WITH_BORDER + i] =
                    second.data[j * PATCH_SIZE_WITH_BORDER + i] -
                    first.data[j * PATCH_SIZE_WITH_BORDER + i];
            }
        }

        return destination;
    }

    void Patch::print() {

        for (size_t j = 0; j < PATCH_SIZE_WITH_BORDER; j++) {
            for (size_t i = 0; i < PATCH_SIZE_WITH_BORDER; i++) {

#ifdef CPU_MIMXRT1176DVMAA
                logger::rawf("%3.3f ", data[j * PATCH_SIZE_WITH_BORDER + i]);
#else
                printf("%3.3f ", data[j * PATCH_SIZE_WITH_BORDER + i]);
#endif
            }

#ifdef CPU_MIMXRT1176DVMAA
            logger::rawf("\r\n");
#else
            printf("\r\n");
#endif
        }
    }

    PatchPyramid::PatchPyramid(Patch* patch_buffer_,
                               const size_t max_number_of_patches_) {
        this->patch_buffer          = patch_buffer_;
        this->max_number_of_patches = max_number_of_patches_;
    }

    void PatchPyramid::construct(
        ImagePyramid& image_pyramid,
        const etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES>& features) {

        if (features.size() > max_number_of_patches) {

#ifdef CPU_MIMXRT1176DVMAA
            logger::errorf("Patch pyramid can't hold %d entries, max: %d\r\n",
                           features.size(),
                           max_number_of_patches);
#else
            printf("Patch pyramid can't hold %lu entries, max: %ld\r\n",
                   features.size(),
                   max_number_of_patches);

#endif
            exit(1);
        }

        for (int pyramid_level = 0; pyramid_level < PYRAMID_LEVELS;
             pyramid_level++) {

            for (int feature_index = 0;
                 feature_index < (int_fast32_t)features.size();
                 feature_index++) {

                const float x = (features[feature_index].x() -
                                 ((float)PATCH_SIZE) / 2.0f) /
                                powf(2, pyramid_level);

                const float y = (features[feature_index].y() -
                                 ((float)PATCH_SIZE) / 2.0f) /
                                powf(2, pyramid_level);

                patch_buffer[pyramid_level * max_number_of_patches +
                             feature_index] =
                    Patch(x, y, *image_pyramid.at(pyramid_level));
            }
        }
    }

    Patch* PatchPyramid::at(const size_t level, const size_t index) {
        return &patch_buffer[level * max_number_of_patches + index];
    }
}
