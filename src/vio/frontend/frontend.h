#ifndef FRONTEND_H
#define FRONTEND_H

#include "feature_observation.h"
#include "image_data_point.h"
#include "imu_data_point.h"
#include "vio_config.h"
#include "vio_typedefs.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include <etl/vector.h>

namespace frontend {

    void configure(const Configuration& configuration);

    /**
     * @brief Processes an image given IMU data between the previous image and
     * the image passed to this routine. This routine will then populate @p
     * out_features with the current features tracked/extracted.
     *
     * @param image_data_point [in] Image to process.
     * @param imu_data [in] IMU data between the previous image and the current
     * image passed to this routine.
     * @param out_feature_data [out] The features to process for the backend is
     * placed in this structure.
     *
     * @return True if the @p out_features contains new data for the backend to
     * process.
     */
    bool process_image(
        const ImageDataPoint& image_data_point,
        etl::vector<ImuDataPoint, MAX_NUMBER_OF_IMU_MEASUREMENTS>& imu_data,
        FeatureObservations& out_feature_data);
}

#endif
