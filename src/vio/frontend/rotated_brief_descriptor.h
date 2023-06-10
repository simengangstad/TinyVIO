/**
 * @brief Module for computing the rotated BRIEF descriptor of a feature.
 * Inspiration for this module comes from the OpenCV project.
 */

#ifndef ROTATED_BRIEF_DESCRIPTOR_H
#define ROTATED_BRIEF_DESCRIPTOR_H

#include "image.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include <etl/vector.h>

namespace frontend {

    /**
     * @brief The BRIEF descriptor length of a given feature.
     */
    constexpr int BRIEF_DESCRIPTOR_LENGTH = 32;

    /**
     * @brief The number of points in the BRIEF pattern.
     */
    static constexpr uint32_t BRIEF_NUMBER_OF_POINTS_IN_PATTERN = 512;

    /**
     * @brief Convenience typedef for a descriptor bitstring.
     */
    typedef Eigen::Vector<uint8_t, BRIEF_DESCRIPTOR_LENGTH> Descriptor;

    class RotatedBriefDescriptor {

      public:
        /**
         * @brief constructs the rotated BRIEF descriptor with the @p patch_size
         * the descriptor will use.
         *
         * @param patch_size [in] The patch size which will be used in BRIEF,
         * where the pattern will be constructed within this patch.
         */
        explicit RotatedBriefDescriptor(uint32_t patch_size = MAX_PATCH_SIZE);

        /**
         * @brief Resets the descriptor with a new image. All the new
         * descriptors will be computed from this image. The image is NOT
         * copied, so the image passed to this has to be valid an in memory as
         * long as the descriptor is utilised.
         *
         * @param image [in] The image to use.
         */
        void reset(const Image& image);

        /**
         * @brief Computes the descriptors for a range of @p feature_points.
         *
         * @param feature_points [in] The feature points.
         * @param out_descriptors [out] The descriptors for the feature points
         * are placed in this vector.
         */
        void compute_descriptors(
            const etl::vector<Eigen::Vector2f, MAX_NUMBER_OF_FEATURES>&
                feature_points,
            etl::vector<Descriptor, MAX_NUMBER_OF_FEATURES>& out_descriptors);

        /**
         * @brief Computes the Hamming distance between two descriptors, which
         * effectively is the logical XOR between the first and the second
         * descriptor, followed by counting the number of set bits.
         *
         * @param a [in] The first descriptor.
         * @param b [in] The second descriptor.
         *
         * @return The number of different bits.
         */
        static inline uint32_t
        compute_descriptor_distance(const Descriptor& a, const Descriptor& b) {

            const uint32_t* a_ptr = (uint32_t*)a.data();
            const uint32_t* b_ptr = (uint32_t*)b.data();

            uint32_t distance = 0;

            for (size_t i = 0; i < BRIEF_DESCRIPTOR_LENGTH / sizeof(uint32_t);
                 i++, a_ptr++, b_ptr++) {

                uint32_t v = *a_ptr ^ *b_ptr;

                // Count the number of bits set after the XOR. For more
                // information about this, see:
                //
                // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

                v = v - ((v >> 1) & 0x55555555);
                v = (v & 0x33333333) + ((v >> 2) & 0x33333333);

                distance += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
            }

            return distance;
        }

      private:
        /**
         * @brief The maximum patch size of for the brief pattern.
         */
        static constexpr uint32_t MAX_PATCH_SIZE = 31;

        /**
         * @brief The half patch size of the descriptor.
         */
        uint32_t half_patch_size;

        /**
         * @brief The BRIEF pattern.
         */
        etl::vector<Eigen::Vector2i, BRIEF_NUMBER_OF_POINTS_IN_PATTERN> pattern;

        /**
         * @brief The image to utilise for calculating the descriptors. Note
         * that a copy does not occur in the descriptor. The image will be set
         * on #reset, and has to be valid and in memory for the lifetime of the
         * descriptor.
         */
        Image image;

        /**
         * @brief Calculate the angle (in radians) between the "centre of
         * intensity" and the origin of the patch. This allows for representing
         * the patch in a canonical rotated form.
         *
         * @param feature [in] The position of the feature.
         */
        [[nodiscard]] float patch_angle(const Eigen::Vector2f& feature) const;
    };

}

#endif
