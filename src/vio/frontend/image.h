#ifndef IMAGE_H
#define IMAGE_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "vio_config.h"

namespace frontend {
    /**
     * @brief Struct for a 8 bit, 1 channel grayscale image.
     */
    struct Image {
        /**
         * @brief Buffer of the image data. Note that this has to be passed to
         * the image, the image struct in itself does not allocate any memory.
         */
        uint8_t* data = NULL;

        /**
         * @brief Width of the image.
         */
        size_t width = 0;

        /**
         * @brief Height of the image.
         */
        size_t height = 0;

        /**
         * @brief Initializes an empty image.
         */
        Image() : data(NULL), width(0), height(0) {}

        /**
         * @brief Initializes the image with a data buffer and a resolution. No
         * copy will be made here, so operations on the data will modify the
         * original buffer passed to this image.
         *
         * @param data Pointer to the image buffer.
         * @param image_width Width of the image.
         * @param image_height Height of the image.
         */
        Image(uint8_t* data_buffer,
              const size_t image_width,
              const size_t image_height)
            : data(data_buffer), width(image_width), height(image_height) {}

        void downsample();

        /**
         * @brief Downsamples the image in half for each axis and places the
         * result in @p destination.
         *
         * @param source The image to downscale.
         * @param destination Where the downscaled image is placed.
         */
        static void downsample(const Image& source, Image& destination);

        /**
         * @brief Copies @p source to @p destination while making a reflected
         * border of size @p border_size. Note that the data in the @p
         * destination image has to be pre-allocated.
         *
         * @param source The source image.
         * @param destination Where the source image with the border will be
         * copied to.
         * @param border_size  The border size.
         */
        static void copy_make_border(const Image& source,
                                     Image& destination,
                                     int_fast32_t border_size);

        /**
         * @brief Blurs an image inplace, will this override the existing data
         * in the image.
         */
        void blur();

        /**
         * @brief Blurs an image buffer in-place.
         *
         * @param image_data The image data.
         * @param image_width The image width.
         * @param image_height The image height.
         */
        static void blur_inplace(uint8_t* image_data,
                                 const size_t image_width,
                                 const size_t image_height);
    };

    /**
     * @brief A pyramid consisting of images, each down-sampled in half for
     * each axis for each level.
     */
    struct ImagePyramid {

      private:
        /**
         * @brief The images in the pyramid level.
         */
        Image images[PYRAMID_LEVELS] = {};

      public:
        /**
         * @brief Constructs an image pyramid from a @p image. Note that the
         * first level in the pyramid will refer to the image passed to this
         * constructor. The rest of the images' buffers will be placed in
         * the pyramid buffer, which has to be pre-allocated and made sure
         * to have enough space to fit the images in the pyramid.
         *
         * @param source The source image in the first level of the pyramid.
         * @param pyramid_buffer Where the rest of the images' data will be
         * placed (above the first level).
         */
        ImagePyramid(const Image& source, uint8_t* pyramid_buffer);

        /**
         * @return A pointer to the image at the given @p pyramid_level.
         */
        Image* at(const size_t pyramid_level);
    };
}

#endif
