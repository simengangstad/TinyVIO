#include "image.h"

#include <math.h>
#include <string.h>

#ifdef CPU_MIMXRT1176DVMAA

#include "board.h"

#endif

int_fast32_t clamp(int_fast32_t value, int_fast32_t min, int_fast32_t max) {

    if (value < min) {
        return min;
    }

    if (value > max) {
        return max;
    }

    return value;
}

namespace frontend {

    void Image::blur_inplace(uint8_t* image_data,
                             const size_t image_width,
                             const size_t image_height) {

        // Gaussian blur kernel:
        //
        // [1/16 1/8 1/16]
        // [ 1/8 1/4 1/8 ]
        // [1/16 1/8 1/16]
        //

        // Separable kernel of the Gaussian blur for horizontal and vertical
        // convolution to reduce the amount of multiplication operations that
        // has to be done:
        //
        // [ 1/4 ]
        // [ 1/2 ] * [ 1/4 1/2 1/4] = Gaussian kernel
        // [ 1/4 ]
        //
        // The kernel is interpreted as the amount of right shift that has to be
        // done.
        const uint8_t kernel[3] = {2, 1, 2};

        // In order to do the convolution in place, we have a buffer for the
        // current rows operated on
        uint8_t row_buffer[image_width * 3];

        // The row buffer pointer is shifted along as we go through the rows,
        // pointing to three rows in the row buffer which holds the image values
        // before being modified. This enables the convolution to be made in
        // place.
        //
        // We calculate the pointers for every row to not having to do modulo
        // operations in the for loops below.
        uint8_t* row_buffer_ptr[image_height];

        for (size_t i = 0; i < image_height; i++) {
            row_buffer_ptr[i] = &row_buffer[(i % 3) * image_width];
        }

        memcpy(row_buffer_ptr[0], &image_data[0], image_width);
        memcpy(row_buffer_ptr[1], &image_data[image_width], image_width);

        for (int_fast32_t y = 1; y < (int_fast32_t)image_height - 1; y++) {
            // Fill the bottom row, as the previous rows have been filled by the
            // last iterations of the loop.

            memcpy(row_buffer_ptr[y + 1],
                   &image_data[(y + 1) * image_width],
                   image_width);

            const uint8_t* top_row    = row_buffer_ptr[y - 1];
            const uint8_t* middle_row = row_buffer_ptr[y];
            const uint8_t* bottom_row = row_buffer_ptr[y + 1];

            for (int_fast32_t x = 1; x < (int_fast32_t)image_width - 1; x++) {

                const int_fast32_t x0 = (x - 1);
                const int_fast32_t x1 = x;
                const int_fast32_t x2 = (x + 1);

                // These are the results from separable horizontal convolution
                const uint8_t r00 = top_row[x0] >> kernel[0];
                const uint8_t r01 = top_row[x1] >> kernel[1];
                const uint8_t r02 = top_row[x2] >> kernel[2];

                const uint8_t r10 = middle_row[x0] >> kernel[0];
                const uint8_t r11 = middle_row[x1] >> kernel[1];
                const uint8_t r12 = middle_row[x2] >> kernel[2];

                const uint8_t r20 = bottom_row[x0] >> kernel[0];
                const uint8_t r21 = bottom_row[x1] >> kernel[1];
                const uint8_t r22 = bottom_row[x2] >> kernel[2];

                // Whereas these are the result from the separable vertical
                // convolution
                const uint8_t r0 = (r00 + r01 + r02) >> kernel[0];
                const uint8_t r1 = (r10 + r11 + r12) >> kernel[1];
                const uint8_t r2 = (r20 + r21 + r22) >> kernel[2];

                image_data[y * image_width + x] = r0 + r1 + r2;
            }
        }
    }

    void Image::downsample() {

        const size_t new_width  = width >> 1;
        const size_t new_height = height >> 1;

        uint8_t row_buffer[width * 2];

        // Downscale by taking an average over every 2x2 block, thus we
        // process 2 rows of the original image for each new row in the
        // downscaled image
        for (int_fast32_t j = 0; j < (int_fast32_t)new_height; j++) {

            memcpy(&row_buffer[0], &data[width * j * 2], width);
            memcpy(&row_buffer[width], &data[width * j * 2 + width], width);

            for (int_fast32_t i = 0; i < (int_fast32_t)new_width; i++) {

                int_fast32_t sum =
                    ((int_fast32_t)row_buffer[i * 2] +
                     (int_fast32_t)row_buffer[i * 2 + 1] +
                     (int_fast32_t)row_buffer[width + i * 2] +
                     (int_fast32_t)row_buffer[width + i * 2 + 1]);

                data[j * new_width + i] = sum / 4;
            }
        }

        width  = new_width;
        height = new_height;
    }

    void Image::downsample(const Image& source, Image& destination) {

        destination.width  = source.width >> 1;
        destination.height = source.height >> 1;

#ifdef CPU_MIMXRT1176DVMAA

        // Downscale by taking an average over every 2x2 block, thus we
        // process 2 rows of the original image for each new row in the
        // downscaled image

        uint32_t* top_row    = (uint32_t*)&source.data[0];
        uint32_t* bottom_row = (uint32_t*)&source.data[source.width];

        // Stride jumps over other row, as we're downsampling with a 2x2 block.
        // Since the top and bottom buffers are 32-bit, we divide by 4.
        const uint32_t stride = 2 * source.width / 4;
#endif

        for (int_fast32_t j = 0; j < (int_fast32_t)destination.height; j++) {

#ifdef CPU_MIMXRT1176DVMAA
            for (int_fast32_t i = 0; i < (int_fast32_t)destination.width;
                 i += 2) {

                // Use SIMD to process two blocks at the same time. UHADD8 will
                // two 32-bit numbers divided into 4 8-bit numbers each together
                // and then half the result. From this we thus have the
                // vertical result for two blocks.
                //
                // The column index, i, is divded by 2 here since the top and
                // buttom buffer is 32-bit
                const uint32_t column_sum = __UHADD8(
                    top_row[j * stride + i / 2],
                    bottom_row[j * stride + i / 2]);

                // Extract the bytes for each column
                const uint32_t c0 = column_sum >> 24;
                const uint32_t c1 = (column_sum >> 16) & 0xFF;
                const uint32_t c2 = (column_sum >> 8) & 0xFF;
                const uint32_t c3 = column_sum & 0xFF;

                // Then we add the column for each block and divide by 2 by a
                // right shift
                destination.data[j * destination.width + i] = (c0 + c1) >> 1;
                destination.data[j * destination.width + i + 1] = (c2 + c3) >>
                                                                  1;
            }
#else
            for (int_fast32_t i = 0; i < (int_fast32_t)destination.width; i++) {
                int_fast32_t sum =
                    ((int_fast32_t)source.data[j * 2 * source.width + i * 2] +
                     (int_fast32_t)
                         source.data[j * 2 * source.width + i * 2 + 1] +
                     (int_fast32_t)source
                         .data[j * 2 * source.width + source.width + i * 2] +
                     (int_fast32_t)source.data[j * 2 * source.width +
                                               source.width + i * 2 + 1]);

                destination.data[j * destination.width + i] = sum / 4;
            }
#endif
        }
    }

    void Image::copy_make_border(const Image& source,
                                 Image& destination,
                                 int_fast32_t border_size) {

        if (border_size < 0) {
            border_size = 0;
        }

        // Fill in the image in the middle
        const int_fast32_t image_width_with_border = source.width +
                                                     2 * border_size;

        const int_fast32_t image_width  = source.width;
        const int_fast32_t image_height = source.height;

        for (int_fast32_t row = 0; row < image_height; row++) {
            memcpy(destination.data +
                       (border_size + row) * image_width_with_border +
                       border_size,
                   source.data + row * image_width,
                   image_width);
        }

        // Then construct the reflected borders
        for (int_fast32_t y = 0; y < 2 * border_size + image_height; y++) {
            int_fast32_t reflected_y = y;

            if (y < border_size) {
                reflected_y = border_size * 2 - y;
            }

            if (y >= image_height + border_size) {
                reflected_y = 2 * (image_height + border_size) - 2 - y;
            }

            for (int_fast32_t x = 0; x < 2 * border_size + image_width; x++) {

                if ((y > border_size && y < image_height + border_size) &&
                    (x > border_size && x < image_width + border_size)) {

                    x = source.width + border_size - 1;
                    continue;
                }

                int_fast32_t reflected_x = x;

                if (x < border_size) {
                    reflected_x = border_size * 2 - x;
                }

                if (x >= image_width + border_size) {
                    reflected_x = 2 * (image_width + border_size) - 2 - x;
                }

                destination.data[y * image_width_with_border + x] =
                    destination.data[reflected_y * image_width_with_border +
                                     reflected_x];
            }
        }

        destination.width  = source.width + border_size * 2;
        destination.height = source.height + border_size * 2;
    }

    void Image::blur() { Image::blur_inplace(data, width, height); }

    ImagePyramid::ImagePyramid(const Image& source, uint8_t* pyramid_buffer) {

        images[0].width  = source.width;
        images[0].height = source.height;
        images[0].data   = source.data;

        int_fast32_t offset = 0;

        for (size_t i = 1; i < PYRAMID_LEVELS; i++) {
            images[i].data = pyramid_buffer + offset;

            Image::downsample(images[i - 1], images[i]);

            offset += images[i].width * images[i].height;
        }

        // Blur the images
        for (size_t i = 0; i < PYRAMID_LEVELS; i++) { images[i].blur(); }
    }

    Image* ImagePyramid::at(const size_t pyramid_level) {
        return &images[pyramid_level];
    }
}
