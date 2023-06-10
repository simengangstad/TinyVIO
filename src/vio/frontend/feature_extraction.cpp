#include "feature_extraction.h"

#include <string.h>

#include "board.h"
#include "logger.h"

#define BELOW_THRESHOLD_RANGE  (1)
#define WITHIN_THRESHOLD_RANGE (0)
#define ABOVE_THRESHOLD_RANGE  (2)

/**
 * @brief The amount of pixels which all has to be either (pixel value -
 * threshold) or (pixel value + threshold) for us to classify it as a corner.
 */
#define CONSECUTIVE_PIXELS_IN_PATTERN (9)

/**
 * @brief This is the amount of pixels in the pattern (16) plus the consecutive
 * pixels needed for it to be classified as a corner (9). We append the amount
 * of consecutive pixels in the pattern here so that if we start at e.g. pixel
 * index 15, we can access the remaining pixels without doing any wrap around
 * logic
 */
#define PIXELS_IN_PATTERN_WITH_WRAP_AROUND (25)

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))

namespace frontend {

    using namespace Eigen;

    /**
     * @brief Calculates the corner score of a pixel candidate.
     *
     * @note Profiling shows that it is not beneficial to inline this.
     *
     * @param ptr [in] Pointer to the pixel candidate.
     * @param pattern [in] Pattern offset used to retrieve the pattern values
     * around the pixel candidate.
     * @param threshold [in] The threshold to compare against.
     *
     * @return The corner score of the pixel candiate.
     */
    AT_QUICKACCESS_SECTION_CODE_ALLOW_INLINE(
        static int calculate_corner_score(const uint8_t* ptr,
                                          const int pattern[],
                                          int threshold)) {

        const int K = 8, N = 16 + K + 1;
        int k, v           = ptr[0];

        short d[N];
        for (k = 0; k < N; k++) d[k] = (short)(v - ptr[pattern[k]]);

        int a0 = threshold;

        for (k = 0; k < 16; k += 2) {

            int a = min((int)d[k + 1], (int)d[k + 2]);

            a = min(a, (int)d[k + 3]);

            if (a <= a0) {
                continue;
            }

            a = min(a, (int)d[k + 4]);
            a = min(a, (int)d[k + 5]);
            a = min(a, (int)d[k + 6]);
            a = min(a, (int)d[k + 7]);
            a = min(a, (int)d[k + 8]);

            a0 = max(a0, min(a, (int)d[k]));
            a0 = max(a0, min(a, (int)d[k + 9]));
        }

        int b0 = -a0;

        for (k = 0; k < 16; k += 2) {

            int b = max((int)d[k + 1], (int)d[k + 2]);

            b = max(b, (int)d[k + 3]);
            b = max(b, (int)d[k + 4]);
            b = max(b, (int)d[k + 5]);

            if (b >= b0) {
                continue;
            }

            b = max(b, (int)d[k + 6]);
            b = max(b, (int)d[k + 7]);
            b = max(b, (int)d[k + 8]);

            b0 = min(b0, max(b, (int)d[k]));
            b0 = min(b0, max(b, (int)d[k + 9]));
        }

        threshold = -b0 - 1;

        return threshold;
    }

    /**
     * @brief Clears a buffer with 0s by iterating 4 bytes at a time;
     *
     * @note Profiling shows that it is not beneficial to inline this.
     *
     * @param buffer [in-out] Input buffer.
     * @param length [in] Length of the bufffer, has to be a multiple of 4.
     */
    AT_QUICKACCESS_SECTION_CODE_ALLOW_INLINE(
        static void fast_clear_buffer(uint8_t* buffer, const size_t length)) {
        for (size_t i = 0; i < length; i += 4) {
            *((uint32_t*)(buffer + i)) = 0x0;
        }
    }

    /**
     * @brief Checks if a given pixel candidate is a FAST corner and returns its
     * score.
     *
     * @param pixel_ptr [in] Pointer to the pixel candidate.
     * @param threshold [in] Threshold for the pixel pattern.
     * @param threshold_lookup_table [in] Contains the lookup table for quickly
     * rejecting the candidate's pixel pattern if it is within the threshold.
     * @param pattern_offset [in] Used to retrieve the pattern around the pixel
     * candidate.
     *
     * @return The score of the pixel candidate. Will return 0 if the pixel is
     * not a candidate.
     */
    AT_QUICKACCESS_SECTION_CODE_ALLOW_INLINE(
        static inline int evaluate_corner_candidate(
            const uint8_t* pixel_ptr,
            const int threshold,
            const uint32_t threshold_lookup_table[512],
            const int pattern_offset[PIXELS_IN_PATTERN_WITH_WRAP_AROUND])) {

        int pixel_value = pixel_ptr[0];

        const uint32_t* threshold_lookup = &threshold_lookup_table[0] -
                                           pixel_value + 255;

        // Here we do checks along all 8 diagonals. If both of
        // the pattern pixel values are inside the threshold,
        // the center pixel can't be a candidate for a corner
        // and we just reject the candidate

        // Profiling has showed that it is beneficial to check
        // for if this is the case after each test along a
        // diagonal

        // Top to bottom
        int mask = threshold_lookup[pixel_ptr[pattern_offset[0]]] |
                   threshold_lookup[pixel_ptr[pattern_offset[8]]];
        if (mask == 0) {
            return 0;
        }

        // Left to right
        mask &= threshold_lookup[pixel_ptr[pattern_offset[4]]] |
                threshold_lookup[pixel_ptr[pattern_offset[12]]];
        if (mask == 0) {
            return 0;
        }

        // Lower right to upper left
        mask &= threshold_lookup[pixel_ptr[pattern_offset[2]]] |
                threshold_lookup[pixel_ptr[pattern_offset[10]]];
        if (mask == 0) {
            return 0;
        }

        // Upper right to lower left
        mask &= threshold_lookup[pixel_ptr[pattern_offset[6]]] |
                threshold_lookup[pixel_ptr[pattern_offset[14]]];
        if (mask == 0) {
            return 0;
        }

        // Lower right to top left
        mask &= threshold_lookup[pixel_ptr[pattern_offset[1]]] |
                threshold_lookup[pixel_ptr[pattern_offset[9]]];
        if (mask == 0) {
            return 0;
        }

        // Right lower to left upper
        mask &= threshold_lookup[pixel_ptr[pattern_offset[3]]] |
                threshold_lookup[pixel_ptr[pattern_offset[11]]];
        if (mask == 0) {
            return 0;
        }

        // Right upper to left lower
        mask &= threshold_lookup[pixel_ptr[pattern_offset[5]]] |
                threshold_lookup[pixel_ptr[pattern_offset[13]]];
        if (mask == 0) {
            return 0;
        }

        // Top right to bottom left
        mask &= threshold_lookup[pixel_ptr[pattern_offset[7]]] |
                threshold_lookup[pixel_ptr[pattern_offset[15]]];

        if (mask == 0) {
            return 0;
        }

        // Given the mask, if we either have information that the surrounding
        // pixels are below or above the threshold,
        //
        // Based on that, do a finer check on the whole pattern
        if (mask & BELOW_THRESHOLD_RANGE) {
            int pixel_minus_threshold = pixel_value - threshold;
            int count                 = 0;

            for (int k = 0; k < PIXELS_IN_PATTERN_WITH_WRAP_AROUND; k++) {
                uint8_t pattern_pixel_value = pixel_ptr[pattern_offset[k]];

                if (pattern_pixel_value < pixel_minus_threshold) {
                    if (++count == CONSECUTIVE_PIXELS_IN_PATTERN) {
                        return calculate_corner_score(pixel_ptr,
                                                      pattern_offset,
                                                      threshold);
                    }
                } else {
                    count = 0;
                }
            }
        }

        if (mask & ABOVE_THRESHOLD_RANGE) {
            int pixel_plus_threshold = pixel_value + threshold;
            int count                = 0;

            for (int k = 0; k < PIXELS_IN_PATTERN_WITH_WRAP_AROUND; k++) {
                uint8_t pattern_pixel_value = pixel_ptr[pattern_offset[k]];

                if (pattern_pixel_value > pixel_plus_threshold) {
                    if (++count == CONSECUTIVE_PIXELS_IN_PATTERN) {
                        return calculate_corner_score(pixel_ptr,
                                                      pattern_offset,
                                                      threshold);
                    }
                } else {
                    count = 0;
                }
            }
        }

        return 0;
    }

    AT_QUICKACCESS_SECTION_CODE(void extract_features(
        const uint8_t* image_buffer,
        const int_fast32_t width,
        const int_fast32_t height,
        const uint8_t threshold,
        etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES>& out_features)) {

        // Reset the features just in case
        out_features.clear();

        // Loop variables are declared here so that they have a reserved
        // register place and don't have to be loaded for new loops
        //
        // Kept as signed integers to prevent underflow checks around 0 for
        // speed.
        int_fast32_t i, j, k;

        // Pattern offset is used to retrieve the pattern around a given center
        // pixel from the pointer of that center pixel without having to do any
        // other computation
        int pattern_offset[PIXELS_IN_PATTERN_WITH_WRAP_AROUND];

        // The pixel pattern from Rosten's implementation is the
        // following (where we let point 0 be at the bottom with counter clock
        // wise rotation):
        //
        //
        //    |    | 9  | 8  | 7  |    |
        //----+----+----+----+----+----+---
        //    | 10 |    |    |    | 6  |
        //----+----+----+----+----+----+---
        // 11 |    |    |    |    |    | 5
        //----+----+----+----+----+----+---
        // 12 |    |    | p  |    |    | 4
        //----+----+----+----+----+----+---
        // 13 |    |    |    |    |    | 3
        //----+----+----+----+----+----+---
        //    | 14 |    |    |    | 2  |
        //----+----+----+----+----+----+---
        //    |    | 15 | 0  | 1  |    |
        //
        //
        // The pattern offset lets us access the pattern around a given pixel p,
        // by having the pointer to that pixel. E.g. for accessing point 4, one
        // would do pixel_ptr[pattern[4]]

        pattern_offset[0]  = 0 + width * 3;
        pattern_offset[1]  = 1 + width * 3;
        pattern_offset[2]  = 2 + width * 2;
        pattern_offset[3]  = 3 + width * 1;
        pattern_offset[4]  = 3 + width * 0;
        pattern_offset[5]  = 3 + width * -1;
        pattern_offset[6]  = 2 + width * -2;
        pattern_offset[7]  = 1 + width * -3;
        pattern_offset[8]  = 0 + width * -3;
        pattern_offset[9]  = -1 + width * -3;
        pattern_offset[10] = -2 + width * -2;
        pattern_offset[11] = -3 + width * -1;
        pattern_offset[12] = -3 + width * 0;
        pattern_offset[13] = -3 + width * 1;
        pattern_offset[14] = -2 + width * 2;
        pattern_offset[15] = -1 + width * 3;

        // Wrap around. This allows us to check the consecutive pixels starting
        // from e.g. point 15 and onwards without any further computation
        pattern_offset[16] = 0 + width * 3;
        pattern_offset[17] = 1 + width * 3;
        pattern_offset[18] = 2 + width * 2;
        pattern_offset[19] = 3 + width * 1;
        pattern_offset[20] = 3 + width * 0;
        pattern_offset[21] = 3 + width * -1;
        pattern_offset[22] = 2 + width * -2;
        pattern_offset[23] = 1 + width * -3;
        pattern_offset[24] = 0 + width * -3;

        // Build a packed threshold such that we have a 32 bit integer with the
        // threshold repeated for use in the SIMD 4 byte instructions
        const uint32_t threshold_packed = (threshold << 24) |
                                          (threshold << 16) | (threshold << 8) |
                                          threshold;

        // The threshold look up table is utilized for a fast check whether a
        // given pattern value is below the range of (pixel value - threshold)
        // to (pixel value + threshold), within or above.
        //
        // This is utilized to quickly reject points which can't be a candidate
        // for a corner.
        //
        // This lookup table consists of the following entries:
        //
        // 0			   <= 1 < (255-threshold)
        // (255-threshold) <= 0 < (255+threshold)
        // (255+threshold) <= 2 < 512
        //
        // To then quickly check a given pattern pixel for this, one can do:
        //
        // lookup_table[255 - pixel value + pattern value]
        //
        // For e.g. a threshold of a 100, a pixel value of 16 and a pattern
        // value of 120, this would yield:
        //
        // lookup_table[255 - 16 + 120] = lookup_table[359]
        //
        // Which according to the definition of the table is within the upper
        // range and has a value of 2.
        uint32_t threshold_lookup_table[512];

        for (int n = -255; n <= 255; n++) {
            threshold_lookup_table[n + 255] =
                (n < -threshold  ? BELOW_THRESHOLD_RANGE
                 : n > threshold ? ABOVE_THRESHOLD_RANGE
                                 : WITHIN_THRESHOLD_RANGE);
        }

        // We keep a buffer which holds the scores of three rows in flight
        //
        // We make sure that the buffer is a multiple of 4 in order to do faster
        // clearing
        uint8_t row_scores_buffer[((width * 3) / 4 + 1) * 4];

        // This pointer array is used to reference the different rows with
        // scores in flight
        uint8_t* row_scores[3];
        row_scores[0] = row_scores_buffer;
        row_scores[1] = row_scores[0] + width;
        row_scores[2] = row_scores[1] + width;

        // Also keep a buffer for the corner positions of three rows in flight
        uint16_t
            row_corner_positions_buffer[(width + 1) * 3 * sizeof(uint16_t)];

        // As with the row scores, we keep a convenience array of pointers for
        // the different rows in flight.
        //
        // Note that the number of corners are kept as the -1'th entry
        // here: (row_corner_positions[n])[-1] is the amount of corners for the
        // n'th row. This proved beneficial during profiling. That's why we
        // append + 1 for the buffer pointers.
        uint16_t* row_corner_positions[3];
        row_corner_positions[0] = row_corner_positions_buffer + 1;
        row_corner_positions[1] = row_corner_positions[0] + width + 1;
        row_corner_positions[2] = row_corner_positions[1] + width + 1;

        fast_clear_buffer(row_scores[0], width * 3);

        for (j = 3; j < height - 2; j++) {

            const uint8_t* pixel_ptr = &image_buffer[j * width] + 3;

            uint8_t* current_row_scores = row_scores[(j - 3) % 3];

            // Buffer for the detected corner positions on this row
            uint16_t* current_row_corner_positions =
                row_corner_positions[(j - 3) % 3];

            // Number of corners detected on this row
            uint16_t current_row_number_of_corners = 0;

            if (j < height - 3) {
                i = 3;

                // Every iteration builds on checking a group of 4 sequential
                // pixels for whether they could be candidates for a corner by
                // utilizing SIMD instructions
                for (; i < width - 7; i += 4, pixel_ptr += 4) {

                    // Clear the scores for this group
                    *((uint32_t*)(current_row_scores + i)) = 0x0;

                    // Now two rough checks are performed along the major
                    // diagonals top/bottom and left/right. The idea behind this
                    // is that with the requirement that there are 9 consecutive
                    // pixels which has to be outside the threshold range (and
                    // equal polarity), if e.g. both point 0 and point 8 are
                    // within the threshold range, there does not exist a
                    // combination which yields a corner candidate. Thus we can
                    // reject that candidate early on. The same is true for e.g.
                    // point 4 and point 12, which are along the main diagonal
                    // from left to right.

                    // Both of these are the 4 sequential pixel values +-
                    // threshold. The UQADD8 will clamp the result between 0
                    // and 255 if the result underflows or overflows,
                    // respectively.
                    //
                    // In other words, for e.g. pixels_plus_threshold_packed,
                    // this will be:
                    //
                    // result[31:24] = clamp(pixel[n + 3] + threshold, 0, 255)
                    // result[23:16] = clamp(pixel[n + 2] + threshold, 0, 255)
                    // result[15:08] = clamp(pixel[n + 1] + threshold, 0, 255)
                    // result[07:00] = clamp(pixel[n + 0] + threshold, 0, 255)
                    uint32_t pixels_plus_threshold_packed =
                        __UQADD8(*((uint32_t*)pixel_ptr), threshold_packed);

                    uint32_t pixels_minus_threshold_packed =
                        __UQSUB8(*((uint32_t*)pixel_ptr), threshold_packed);

                    // We operate on a group of 4 in the pattern pixels as well,
                    // here for point 0 and point 8
                    //
                    // The first check checks the main diagonal from top to
                    // bottom, which is point 0 and point 8
                    uint32_t point0_packed = *(
                        (uint32_t*)(pixel_ptr + pattern_offset[0]));

                    uint32_t point8_packed = *(
                        (uint32_t*)(pixel_ptr + pattern_offset[8]));

                    // The pixels_(plus/minus)_threshold_mask represent the
                    // packed pattern points being within the threshold range or
                    // not by just subtracting them from the
                    // pixels_(plus/minus)_threshold and utilizing SEL to
                    // retrieve if there was an underflow or not by reading the
                    // ASPR.GE bits (which there are 4 of for each byte in the
                    // 32 bit integer).
                    //
                    // So if point0[n] pixel value > center[n] pixel value +
                    // threshold, then the APSR.GE[n] bit will be 1 and we have
                    // a candidate since it is outside the threshold range.
                    //
                    // The same logic is utilized for the lower threshold value
                    // (being below center[n] pixel value - threshold)
                    uint32_t pixels_plus_threshold_mask  = 0x0;
                    uint32_t pixels_minus_threshold_mask = 0x0;

                    // If the pixels + threshold is all greater than 255, then
                    // there is no point in doing this check, since a pattern
                    // pixel won't have a greater value than that anyway.
                    //
                    // This is also the case for pixels - threshold. If they all
                    // are 0, there is no point doing the check.
                    if (pixels_plus_threshold_packed < 0xFFFFFFFF) {
                        __USUB8(point8_packed, pixels_plus_threshold_packed);
                        pixels_plus_threshold_mask = __SEL(0xFFFFFFFF,
                                                           0x00000000);

                        __USUB8(point0_packed, pixels_plus_threshold_packed);
                        pixels_plus_threshold_mask |= __SEL(0xFFFFFFFF,
                                                            0x00000000);
                    }

                    if (pixels_minus_threshold_packed > 0x0) {
                        __USUB8(pixels_minus_threshold_packed, point8_packed);
                        pixels_minus_threshold_mask = __SEL(0xFFFFFFFF,
                                                            0x00000000);

                        __USUB8(pixels_minus_threshold_packed, point0_packed);
                        pixels_minus_threshold_mask |= __SEL(0xFFFFFFFF,
                                                             0x00000000);
                    }

                    // The threshold mask will no contain possible candidates
                    // for each pixel in the group of 4, where possibly we have
                    // that the pattern pixels at point 0 and 8 are outside the
                    // threshold range
                    //
                    // So if e.g. this mask is FF00FF00, that means that there
                    // are corner candidates for center pixel 1 and center pixel
                    // 3 (seen from right and null-indexed)
                    uint32_t threshold_mask = pixels_plus_threshold_mask |
                                              pixels_minus_threshold_mask;

                    if (threshold_mask == 0) {
                        continue;
                    }

                    // Here we check if the leading center points are 0, so we
                    // can stop here, reset the indices (i) and the pixel
                    // pointer appropriately and continue on from the start of
                    // the loop. Remember that the least-significant bits are
                    // the bits corresponding to the left-most pixel
                    if (threshold_mask == 0xFFFFFF00) {
                        i -= 3;
                        pixel_ptr -= 3;
                        continue;
                    }

                    if (threshold_mask == 0xFFFF0000) {
                        i -= 2;
                        pixel_ptr -= 2;
                        continue;
                    }

                    if (threshold_mask == 0xFF000000) {
                        i -= 1;
                        pixel_ptr -= 1;
                        continue;
                    }

                    // Here we do left to right diagonal
                    uint32_t point4_packed = *(
                        (uint32_t*)(pixel_ptr + pattern_offset[4]));
                    uint32_t point12_packed = *(
                        (uint32_t*)(pixel_ptr + pattern_offset[12]));

                    if (pixels_plus_threshold_packed < 0xFFFFFFFF) {
                        __USUB8(point4_packed, pixels_plus_threshold_packed);
                        pixels_plus_threshold_mask = __SEL(0xFFFFFFFF,
                                                           0x00000000);
                        __USUB8(point12_packed, pixels_plus_threshold_packed);
                        pixels_plus_threshold_mask |= __SEL(0xFFFFFFFF,
                                                            0x00000000);
                    }

                    if (pixels_minus_threshold_packed > 0x0) {
                        __USUB8(pixels_minus_threshold_packed, point4_packed);
                        pixels_minus_threshold_mask = __SEL(0xFFFFFFFF,
                                                            0x00000000);
                        __USUB8(pixels_minus_threshold_packed, point12_packed);
                        pixels_minus_threshold_mask |= __SEL(0xFFFFFFFF,
                                                             0x00000000);
                    }

                    threshold_mask = (pixels_plus_threshold_mask |
                                      pixels_minus_threshold_mask);

                    if (threshold_mask == 0) {
                        continue;
                    }

                    if (threshold_mask == 0xFFFFFF00) {
                        i -= 3;
                        pixel_ptr -= 3;
                        continue;
                    }

                    if (threshold_mask == 0xFFFF0000) {
                        i -= 2;
                        pixel_ptr -= 2;
                        continue;
                    }

                    if (threshold_mask == 0xFF000000) {
                        i -= 1;
                        pixel_ptr -= 1;
                        continue;
                    }

                    // Profiling shows that doing more checks along other
                    // diagonals in these group of fours would increase the run
                    // time

                    // Now we do finer checks on a per pixel basis in the
                    // pattern
                    for (uint8_t idx = 0; idx < 4; idx++) {

                        current_row_scores[i + idx] = evaluate_corner_candidate(
                            &pixel_ptr[idx],
                            threshold,
                            threshold_lookup_table,
                            pattern_offset);

                        if (current_row_scores[i + idx] != 0) {
                            current_row_corner_positions
                                [current_row_number_of_corners++] = i + idx;
                        }
                    }
                }
            }

            // Here comes the trick to specify the -1 index of the row corner
            // positions buffer is the number of corners. Structuring this way
            // proved beneficial during profiling
            current_row_corner_positions[-1] = current_row_number_of_corners;

            if (j == 3) {
                continue;
            }

            // Here we check the score of the previous row against the second
            // previous row and the current row just examined. The one within a
            // 3x3 grid with the highest score wins and is classified as a
            // feature
            const uint8_t* second_previous_row_scores = row_scores[(j - 2) % 3];

            const uint8_t* previous_row_scores = row_scores[(j - 1) % 3];
            const uint16_t* previous_row_corner_positions =
                row_corner_positions[(j - 1) % 3];
            const uint16_t previous_row_number_of_corners =
                previous_row_corner_positions[-1];

            for (k = 0; k < (int_fast32_t)previous_row_number_of_corners; k++) {

                const uint32_t idx = previous_row_corner_positions[k];
                const uint8_t previous_row_score = previous_row_scores[idx];

                if ((previous_row_score > previous_row_scores[idx + 1] &&
                     previous_row_score > previous_row_scores[idx - 1] &&
                     previous_row_score > second_previous_row_scores[idx - 1] &&
                     previous_row_score > second_previous_row_scores[idx] &&
                     previous_row_score > second_previous_row_scores[idx + 1] &&
                     previous_row_score > current_row_scores[idx - 1] &&
                     previous_row_score > current_row_scores[idx] &&
                     previous_row_score > current_row_scores[idx + 1])) {

                    if (out_features.full()) {

                        logger::debugf(
                            "Did not have enough space in the feature buffer "
                            "to store all features\r\n");

                        return;
                    }

                    out_features.push_back(Vector2f(idx, j - 1));
                }
            }
        }
    }
}
