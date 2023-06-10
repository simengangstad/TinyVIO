#include "frontend.h"

#ifndef CPU_MIMXRT1176DVMAA

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>

#define AT_OCRAM_SECTION_DATA(var) var

#else

#include "board.h"
#include "feature_extraction.h"

#endif

#include "backend.h"
#include "feature_tracking.h"
#include "image.h"
#include "patch.h"
#include "rotated_brief_descriptor.h"
#include "vio_typedefs.h"

namespace frontend {

    using namespace Eigen;

#ifndef CPU_MIMXRT1176DVMAA

    static cv::Mat to_cv_image(const Image& image) {

        cv::Mat image_cv(image.height, image.width, CV_8U);
        memcpy(image_cv.data, image.data, image.width * image.height);

        return image_cv;
    }

#endif

    static Configuration configuration;

    enum class ImageState {
        FIRST_INITIALISATION_IMAGE,
        SECOND_INITIALISATION_IMAGE,
        POST_INITIALISATION_STAGE
    };

    /**
     * @brief Used to keep track of where the pipeline is in the initialisation
     * processes or if it's past initialisation.
     */
    static ImageState image_state = ImageState::FIRST_INITIALISATION_IMAGE;

    /**
     * @brief The fast threshold used. Will be adjusted if not enough features
     * are extracted in the initialisation phase.
     */
    static uint8_t fast_threshold;

    /**
     * @brief Keeps track of which ID a newly extracted features has.
     */
    static FeatureIdentifier next_feature_identifier = 0;

    /**
     * @brief Stores the camera intrinsic matrix (referred to as K), for
     * projecting world coordinates to pixel coordinates.
     */
    static Matrix3f camera_intrinsics;

    /**
     * @brief Inverse of image_processor::camera_intrinsics. Use to not having
     * to compute on the fly.
     */
    static Matrix3f camera_intrinsics_inverse;

    /**
     * @brief Parameters for the radial-tangential distortion.
     */
    static Vector4f camera_distortion_parameters;

    /**
     * @brief Transforms a vector from the camera frame to the IMU frame.
     */
    static Isometry3f T_imu_camera;

    /**
     * @brief Inverse of image_processor::T_imu_camera. Used to not having to
     * compute it on the fly.
     */
    static Isometry3f T_camera_imu;

    /**
     * @brief Rotates a vector from the previous image frame to the current
     * image frame.
     */
    static Matrix3f R_currentframe_previousframe;

    /**
     * @brief Holds the new features extracted from the current image.
     */
    static etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES> new_features;

    /**
     * @brief Temporary buffer used to store the predicted track of new
     * features extracted. Declared here to save space on the stack.
     */
    static etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES> new_features_tracked;

    /**
     * @brief Holds the features tracked/extracted from the previous image.
     */
    static etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES> previous_features;

    /**
     * @brief Holds the features tracked in the current image.
     */
    static etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES> current_features;

    /**
     * @brief The feature identifiers for the currently tracked features (@see
     * image_processor::current_features).
     */
    static etl::vector<FeatureIdentifier, MAX_NUMBER_OF_FEATURES>
        feature_identifiers;

    /**
     * @brief Holds how long a given features has been tracked.
     */
    static etl::vector<uint32_t, MAX_NUMBER_OF_FEATURES> feature_lifetimes;

    /**
     * @brief Keeps track of which points which are newly initialized and NOT
     * used during initialization (first two images). So a feature here would
     * NOT be (-1, -1) if the initialization process is done and we are tracking
     * a new feature. It will turn (-1, -1) after the first frame the feature is
     * processed. This also keeps track so that the initialisation velocity
     * isn't overwritten.
     */
    static etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES>
        initialisation_features;

    /**
     * @brief Timestamp of when the previous image arrived (in seconds).
     */
    static double previous_image_time = 0.0;

    /**
     * @brief Timestamp of when the current image arrived (in seconds).
     */
    static double current_image_time = 0.0;

    /**
     * @brief Timestamp of when the last update (finding new features and
     * reporting the features) was done (in seconds).
     */
    static double last_update_time = 0.0;

    /**
     * @brief Is set when the first image arrives and we have corresponding IMU
     * data before the timestamp of the image.
     */
    static bool got_first_image = false;

    /**
     * @brief Used to compute BRIEF descriptors.
     */
    static RotatedBriefDescriptor brief_descriptor;

    /**
     * @brief The descriptors for the new features extracted.
     */
    static etl::vector<Descriptor, MAX_NUMBER_OF_FEATURES>
        new_feature_descriptors;

    /**
     * @brief The descriptors for the current features tracked.
     */
    static etl::vector<Descriptor, MAX_NUMBER_OF_FEATURES>
        current_feature_descriptors;

    /**
     * @brief Temporary variable used for comparing descriptors after tracking
     * to reject outliers.
     */
    static etl::vector<Descriptor, MAX_NUMBER_OF_FEATURES>
        temporary_descriptors;

    /**
     * @brief Buffer for the mask of the current features, use to check if there
     * is a feature in a given area already.
     */
    static uint8_t mask_buffer[IMAGE_WIDTH * IMAGE_HEIGHT];

    /**
     * @brief Image used for checking if newly extracted features overlap with
     * the current features.
     */
    static Image mask;

    /**
     * @brief Temporary buffer used for rejecting outliers. Declared here to
     * save stack usage in the functions which do rejection of outliers.
     */
    static etl::vector<uint8_t, MAX_NUMBER_OF_FEATURES> inliers;

    /**
     * @brief Calculates the size of an image pyramid for a given @p width, @p
     * height and @p pyramid_levels.
     */
    static constexpr size_t
    calculate_image_pyramid_size(const size_t width,
                                 const size_t height,
                                 const size_t pyramid_levels) {

        size_t size = 0;

        for (size_t i = 0; i < pyramid_levels; i++) {

            size_t power = 1;
            for (size_t j = 0; j < i; j++) { power *= 2; }

            size += (width / power) * (height / power);
        }

        return size;
    }

    /**
     * @brief Size in bytes of the pyramid level except the first level.
     */
    constexpr static size_t upper_levels_image_pyramid_buffer_size =
        calculate_image_pyramid_size(IMAGE_WIDTH,
                                     IMAGE_HEIGHT,
                                     PYRAMID_LEVELS) -
        IMAGE_WIDTH * IMAGE_HEIGHT;

    /**
     * @brief Max amount of patches in each pyramid level for the patch pyramid.
     */
    constexpr static size_t max_number_of_patches_in_pyramid_level =
        MAX_NUMBER_OF_FEATURES;

    /**
     * @brief Buffer for the upper levels of the image pyramid.
     */
    AT_OCRAM_SECTION_DATA(static uint8_t upper_levels_image_pyramid_buffer
                              [upper_levels_image_pyramid_buffer_size]);

    /**
     * @brief Patch pyramid for the new features.
     */
    static Patch new_features_patch_pyramid_buffer
        [max_number_of_patches_in_pyramid_level * PYRAMID_LEVELS];

    /**
     * @brief Patch pyramid for the previous features.
     */
    static Patch previous_features_patch_pyramid_buffer
        [max_number_of_patches_in_pyramid_level * PYRAMID_LEVELS];

    /**
     * @brief Stores the pyramid of each patch and will always contain the data
     * from the previous image.
     */
    static PatchPyramid
        previous_features_patch_pyramid(previous_features_patch_pyramid_buffer,
                                        max_number_of_patches_in_pyramid_level);

    /**
     * @brief Stores the patch pyramid of the new features extracted.
     */
    static PatchPyramid
        new_features_patch_pyramid(new_features_patch_pyramid_buffer,
                                   max_number_of_patches_in_pyramid_level);

    /**
     * @brief Integrates the IMU data buffer to provide an estimate for the
     * rotation from the previous camera frame and the current camera frame.
     *
     * @param imu_data [in] The IMU data to integrate.
     * @param out_R_currentframe_previousframe [out] The rotation will be stored
     * in this matrix. This rotation when multiplied with a vector in the
     * previous camera frame will yield the vector in the current camera frame.
     */
    static void integrate_imu_data(
        const etl::vector<ImuDataPoint, MAX_NUMBER_OF_IMU_MEASUREMENTS>&
            imu_data,
        Matrix3f& out_R_currentframe_previousframe) {

        auto start_iterator = imu_data.begin();

        const float imu_frame_time = 1.0f /
                                     static_cast<float>(configuration.imu_rate);

        // This finds the IMU data entry which is right after the previous image
        while (start_iterator != imu_data.end()) {
            if (start_iterator->timestamp - previous_image_time <
                -(imu_frame_time - 0.001)) {
                start_iterator++;
            } else {
                break;
            }
        }

        auto end_iterator = start_iterator;

        // This finds the IMU data entry which is right before the current image
        while (end_iterator != imu_data.end()) {

            if (end_iterator->timestamp - current_image_time <
                (imu_frame_time - 0.001)) {
                end_iterator++;
            } else {
                break;
            }
        }

        Vector3f mean_angular_velocity_imu(0.0f, 0.0f, 0.0f);

        for (auto iterator = start_iterator; iterator < end_iterator;
             iterator++) {

            mean_angular_velocity_imu += iterator->angular_velocity;
        }

        if (end_iterator - start_iterator > 0) {
            mean_angular_velocity_imu *= 1.0f / (end_iterator - start_iterator);
        }

        const double delta_time = current_image_time - previous_image_time;

        // Transform to the camera frame
        const Vector3f mean_angular_velocity_camera =
            delta_time * (T_camera_imu.rotation() * mean_angular_velocity_imu);

        // Now we find the angle axis representation
        const float theta   = mean_angular_velocity_camera.norm();
        const Vector3f axis = mean_angular_velocity_camera / theta;

        // Since these angular velocity values accumulated represents the
        // current image frame, and applying a transformation of a vector on
        // this would rotate the vector in the current frame to the previous
        // frame, we apply the transpose to get the rotation from the
        // previous frame to the current frame
        out_R_currentframe_previousframe =
            AngleAxisf(theta, axis).toRotationMatrix().transpose();
    }

    /**
     * @brief Given an estimate for the rotation between the previous camera
     * frame and the current camera frame, this function will predict the
     * location of the @p input_features in the current frame and store them in
     * @p out_predicted_features.
     *
     * @param input_features [in] The points to predict the location of in the
     * current frame.
     * camera frame to the current camera frame.
     * @param out_predicted_features [out] The predicted points will be placed
     * in this vector.
     */
    static void predict_feature_tracking(
        const etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES>& input_features,
        etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES>& out_predicted_features) {

        if (input_features.empty()) {
            out_predicted_features.clear();
            return;
        }

        // Fill with the number of features we need
        out_predicted_features.resize(input_features.size(),
                                      Vector2f(0.0f, 0.0f));

        // We need to apply the transformation in the image coordinate system,
        // so we need to transform to that on both sides so the input is pixel
        // coordinates and the output of the transformation is pixel coordinates
        const Matrix3f H = camera_intrinsics * R_currentframe_previousframe *
                           camera_intrinsics_inverse;

        for (int_fast32_t i = 0; i < (int_fast32_t)input_features.size(); i++) {

            const Vector3f feature_currentframe =
                H *
                Vector3f(input_features[i].x(), input_features[i].y(), 1.0f);

            // De-homogenise
            out_predicted_features[i].x() = feature_currentframe.x() /
                                            feature_currentframe.z();
            out_predicted_features[i].y() = feature_currentframe.y() /
                                            feature_currentframe.z();
        }
    }

    /**
     * @brief Detects features on the first image.
     *
     * @param image_pyramid The pyramid to detect features in.
     *
     * @return True if the number of features detected is above @def
     * MINIMUM_REQUIRED_FEATURES_FOR_INITIALISATION.
     */
    static bool initialise_first_frame(ImagePyramid& image_pyramid) {

        new_features.clear();

        if (configuration.use_brief) {
            new_feature_descriptors.clear();
        }

#ifndef CPU_MIMXRT1176DVMAA

        std::vector<cv::KeyPoint> cv_keypoints;
        cv::FAST(to_cv_image(*image_pyramid.at(0)),
                 cv_keypoints,
                 fast_threshold,
                 true);
        for (auto cv_keypoint : cv_keypoints) {

            if (new_features.full()) {
                break;
            }

            new_features.push_back(
                Vector2f(cv_keypoint.pt.x, cv_keypoint.pt.y));
        }

#else
        const Image image = *(image_pyramid.at(0));

        frontend::extract_features(image.data,
                                   image.width,
                                   image.height,
                                   fast_threshold,
                                   new_features);

#endif

        if (new_features.size() >
            MINIMUM_REQUIRED_FEATURES_FOR_INITIALISATION) {

            if (configuration.use_brief) {

                brief_descriptor.compute_descriptors(new_features,
                                                     new_feature_descriptors);
            }

            return true;
        } else {
            return false;
        }
    }

    /**
     * @brief Removes outliers from @p vector in place.
     *
     * @param vector [in, out] The vector to remove inliers inplace from.
     * @param markers [in] Markers, where 1 represents an inlier.
     */
    template <typename T, size_t N>
    void remove_outliers_inplace(etl::vector<T, N>& vector,
                                 const etl::vector<uint8_t, N>& markers) {

        static etl::vector<T, N> temp;

        if (vector.size() != markers.size()) {
            return;
        }

        temp.clear();

        for (const T& value : vector) { temp.push_back(value); }

        vector.clear();

        for (size_t i = 0; i < markers.size(); ++i) {
            if (markers[i] == 0) {
                continue;
            }

            vector.push_back(temp[i]);
        }
    }

    /**
     * @brief Tracks the features extacted from @see
     * image_processor::initialise_first_frame.
     *
     * @param image_pyramid [in] The image pyramid of the current image.
     * @param imu_data [in] The IMU data points between the timestamp when the
     * first frame was initialised and the current frame.
     *
     * @return True if the initialisation was successful and was able to track a
     * minimum of @def MINIMUM_REQUIRED_FEATURES_FOR_INITIALISATION.
     */
    static bool initialise_first_features(
        ImagePyramid& image_pyramid,
        const etl::vector<ImuDataPoint, MAX_NUMBER_OF_IMU_MEASUREMENTS>&
            imu_data) {

        integrate_imu_data(imu_data, R_currentframe_previousframe);

        inliers.resize(new_features.size());
        inliers.fill(1);

        predict_feature_tracking(new_features, new_features_tracked);

        frontend::track_features(new_features_patch_pyramid,
                                 image_pyramid,
                                 new_features,
                                 new_features_tracked,
                                 inliers,
                                 true,
                                 configuration.lucas_kanade_max_iterations);

        remove_outliers_inplace(new_features, inliers);
        remove_outliers_inplace(new_features_tracked, inliers);

        if (configuration.use_brief) {
            remove_outliers_inplace(new_feature_descriptors, inliers);
        }

        if (new_features.size() <
            MINIMUM_REQUIRED_FEATURES_FOR_INITIALISATION) {

            return false;
        }

        // Use BRIEF to remove outliers
        if (configuration.use_brief) {

            // Mark as outliers if descriptor distance is too large
            brief_descriptor.compute_descriptors(new_features_tracked,
                                                 temporary_descriptors);

            inliers.resize(new_features.size());

            for (size_t i = 0; i < new_features_tracked.size(); i++) {
                const uint32_t distance =
                    RotatedBriefDescriptor::compute_descriptor_distance(
                        new_feature_descriptors[i],
                        temporary_descriptors[i]);

                if (distance <= configuration.brief_threshold) {
                    inliers[i] = 1;
                } else {
                    inliers[i] = 0;
                }
            }

            remove_outliers_inplace(new_features, inliers);
            remove_outliers_inplace(new_features_tracked, inliers);
            remove_outliers_inplace(new_feature_descriptors, inliers);

            if (new_features.size() <
                MINIMUM_REQUIRED_FEATURES_FOR_INITIALISATION) {

                return false;
            }
        }

        previous_features.clear();
        current_features.clear();
        feature_identifiers.clear();
        feature_lifetimes.clear();
        initialisation_features.clear();

        if (configuration.use_brief) {
            current_feature_descriptors.clear();
        }

        for (size_t i = 0; i < new_features.size(); i++) {
            previous_features.push_back(new_features[i]);
            initialisation_features.push_back(Vector2f(-1, -1));
            current_features.push_back(new_features_tracked[i]);
            feature_identifiers.push_back(next_feature_identifier++);
            feature_lifetimes.push_back(2);

            if (configuration.use_brief) {
                current_feature_descriptors.push_back(
                    new_feature_descriptors[i]);
            }
        }

        if (configuration.use_brief) {
            new_feature_descriptors.clear();
        }

        new_features.clear();

        return true;
    }

    /**
     * @brief Finds new features to be tracked in the current image. If a
     * successful track happens in the next image, the features found in this
     * routine will be added to the current features tracked.
     *
     * @param image [in] The image to extract the new features from.
     */
    static void find_new_features_to_track(Image& image) {

        // Create a mask to avoid re-detecting existing features
        mask.data   = mask_buffer;
        mask.width  = image.width;
        mask.height = image.height;
        memset(mask.data, 255, mask.width * mask.height);

        for (const auto& feature : current_features) {
            int start_row = round(feature.y()) -
                            configuration.minimum_distance_between_features;
            start_row = (start_row < 0) ? 0 : start_row;

            int end_row = round(feature.y()) +
                          configuration.minimum_distance_between_features;
            end_row = (end_row > ((int)image.height) - 1)
                          ? ((int)image.height) - 1
                          : end_row;

            int start_col = round(feature.x()) -
                            configuration.minimum_distance_between_features;
            start_col = (start_col < 0) ? 0 : start_col;

            int end_col = round(feature.x()) +
                          configuration.minimum_distance_between_features;
            end_col = (end_col > ((int)image.width) - 1)
                          ? ((int)image.width) - 1
                          : end_col;

            for (int row = start_row; row <= end_row; row++) {
                for (int col = start_col; col <= end_col; col++) {
                    mask.data[row * mask.width + col] = 0;
                }
            }
        }

        new_features.clear();

        if (configuration.use_brief) {
            new_feature_descriptors.clear();
        }

#ifndef CPU_MIMXRT1176DVMAA

        std::vector<cv::KeyPoint> cv_keypoints;
        cv::FAST(to_cv_image(image), cv_keypoints, fast_threshold, true);

        for (auto cv_keypoint : cv_keypoints) {

            if (mask.data[(int)(round(cv_keypoint.pt.y) * mask.width +
                                round(cv_keypoint.pt.x))] != 0) {

                if (new_features.full()) {
                    break;
                }

                new_features.push_back(
                    Vector2f(cv_keypoint.pt.x, cv_keypoint.pt.y));
            }
        }

#else

        static etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES> extracted_features;

        frontend::extract_features(image.data,
                                   image.width,
                                   image.height,
                                   fast_threshold,
                                   extracted_features);

        for (const Vector2f& point : extracted_features) {

            if (new_features.full()) {
                break;
            }

            if (mask.data[(int)(round(point.y()) * mask.width +
                                round(point.x()))] != 0) {

                new_features.push_back(point);
            }
        }
#endif

        if (configuration.use_brief) {
            brief_descriptor.compute_descriptors(new_features,
                                                 new_feature_descriptors);
        }
    }

    /**
     * @brief Tracks the features extracted in the last image.
     *
     * @param image_pyramid [in] The image pyramid of the current image.
     */
    static void track_new_features(ImagePyramid& image_pyramid) {

        if (new_features.empty()) {
            return;
        }

        inliers.resize(new_features.size());
        inliers.fill(1);

        predict_feature_tracking(new_features, new_features_tracked);

        frontend::track_features(new_features_patch_pyramid,
                                 image_pyramid,
                                 new_features,
                                 new_features_tracked,
                                 inliers,
                                 true,
                                 configuration.lucas_kanade_max_iterations);

        remove_outliers_inplace(new_features, inliers);
        remove_outliers_inplace(new_features_tracked, inliers);

        if (configuration.use_brief) {
            remove_outliers_inplace(new_feature_descriptors, inliers);
        }

        if (new_features.empty()) {
            return;
        }

        // Use BRIEF to remove outliers
        if (configuration.use_brief) {

            // Mark as outliers if descriptor distance is too large
            brief_descriptor.compute_descriptors(new_features_tracked,
                                                 temporary_descriptors);

            inliers.resize(new_features.size());

            for (size_t i = 0; i < new_features_tracked.size(); i++) {

                const uint32_t distance =
                    RotatedBriefDescriptor::compute_descriptor_distance(
                        new_feature_descriptors[i],
                        temporary_descriptors[i]);

                if (distance <= configuration.brief_threshold) {
                    inliers[i] = 1;
                } else {
                    inliers[i] = 0;
                }
            }

            // Remove outliers
            remove_outliers_inplace(new_features, inliers);
            remove_outliers_inplace(new_features_tracked, inliers);
            remove_outliers_inplace(new_feature_descriptors, inliers);
        }

        if (new_features.empty()) {
            return;
        }

        for (size_t i = 0; i < new_features.size(); i++) {

            if (previous_features.full()) {
                break;
            }

            previous_features.push_back(new_features[i]);
            initialisation_features.push_back(new_features[i]);
            current_features.push_back(new_features_tracked[i]);
            feature_identifiers.push_back(next_feature_identifier++);
            feature_lifetimes.push_back(2);

            if (configuration.use_brief) {
                current_feature_descriptors.push_back(
                    new_feature_descriptors[i]);
            }
        }

        if (configuration.use_brief) {
            new_feature_descriptors.clear();
        }

        new_features.clear();
    }

    /**
     * @brief Track the current features on the current image.
     *
     * @param image_pyramid [in] The image pyramid of the current image.
     */
    static void track_features(ImagePyramid& image_pyramid) {

        if (previous_features.empty()) {
            return;
        }

        inliers.resize(previous_features.size());
        inliers.fill(1);

        // Increase the lifetime of the features
        for (uint32_t& feature_lifetime : feature_lifetimes) {
            feature_lifetime++;
        }

        predict_feature_tracking(previous_features, current_features);

        frontend::track_features(previous_features_patch_pyramid,
                                 image_pyramid,
                                 previous_features,
                                 current_features,
                                 inliers,
                                 true,
                                 configuration.lucas_kanade_max_iterations);

        // Remove outliers
        remove_outliers_inplace(previous_features, inliers);
        remove_outliers_inplace(current_features, inliers);
        remove_outliers_inplace(feature_identifiers, inliers);
        remove_outliers_inplace(feature_lifetimes, inliers);
        remove_outliers_inplace(initialisation_features, inliers);

        if (configuration.use_brief) {
            remove_outliers_inplace(current_feature_descriptors, inliers);
        }

        if (previous_features.empty()) {
            current_features.clear();
            feature_identifiers.clear();
            feature_lifetimes.clear();
            initialisation_features.clear();
            current_feature_descriptors.clear();
            return;
        }

        // Use BRIEF to remove outliers
        if (configuration.use_brief) {

            // Mark as outliers if descriptor distance is too large. We compute
            // the current descriptors and match them against the previous ones
            brief_descriptor.compute_descriptors(current_features,
                                                 temporary_descriptors);

            inliers.resize(current_features.size());

            for (size_t i = 0; i < current_features.size(); i++) {

                const uint32_t distance =
                    RotatedBriefDescriptor::compute_descriptor_distance(
                        current_feature_descriptors[i],
                        temporary_descriptors[i]);

                if (distance <= configuration.brief_threshold) {
                    inliers[i] = 1;
                } else {
                    inliers[i] = 0;
                }
            }

            remove_outliers_inplace(previous_features, inliers);
            remove_outliers_inplace(current_features, inliers);
            remove_outliers_inplace(feature_identifiers, inliers);
            remove_outliers_inplace(feature_lifetimes, inliers);
            remove_outliers_inplace(initialisation_features, inliers);
            remove_outliers_inplace(current_feature_descriptors, inliers);
        }

        // Remove feature's which has surpassed track length to allow for new
        // features
        for (size_t i = 0; i < current_features.size(); i++) {

            if (feature_lifetimes[i] > configuration.maximum_feature_lifetime) {
                inliers[i] = 0;
            }
        }

        remove_outliers_inplace(previous_features, inliers);
        remove_outliers_inplace(current_features, inliers);
        remove_outliers_inplace(feature_identifiers, inliers);
        remove_outliers_inplace(feature_lifetimes, inliers);
        remove_outliers_inplace(initialisation_features, inliers);
        remove_outliers_inplace(current_feature_descriptors, inliers);

        if (previous_features.empty()) {
            current_features.clear();
            feature_identifiers.clear();
            feature_lifetimes.clear();
            initialisation_features.clear();
            current_feature_descriptors.clear();
            return;
        }
    }

    /**
     * @brief Undistorts the given @p input_features and places them in @p
     * out_undistorted_points.
     */
    static void undistort_points(
        const etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES>& input_features,
        etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES>& out_undistorted_points) {

        if (input_features.empty()) {
            return;
        }

        out_undistorted_points.clear();

        const float c_x = camera_intrinsics(0, 2);
        const float c_y = camera_intrinsics(1, 2);

        const float f_x = camera_intrinsics(0, 0);
        const float f_y = camera_intrinsics(1, 1);

        const float k1 = camera_distortion_parameters[0];
        const float k2 = camera_distortion_parameters[1];
        const float p1 = camera_distortion_parameters[2];
        const float p2 = camera_distortion_parameters[3];

        for (const auto& input_feature : input_features) {
            float x = (input_feature.x() - c_x) / f_x;
            float y = (input_feature.y() - c_y) / f_y;

            const float x_d = x;
            const float y_d = y;

            for (int i = 0; i < 5; i++) {

                const float r2 = x * x + y * y;
                const float r4 = r2 * r2;

                const float k_inv = 1.0f / (1.0f + k1 * r2 + k2 * r4);

                const float tangential_x = 2 * p1 * x * y +
                                           p2 * (r2 + 2 * x * x);
                const float tangential_y = 2 * p2 * x * y +
                                           p1 * (r2 + 2 * y * y);

                x = (x_d - tangential_x) * k_inv;
                y = (y_d - tangential_y) * k_inv;
            }

            out_undistorted_points.push_back(Vector2f(x, y));
        }
    }

    /**
     * @brief Constructs a set of feature observations from the current features
     * tracked.
     *
     * @param out_feature_observations [out] The features observations will be
     * stored in this structure.
     */
    static void construct_feature_observations(
        FeatureObservations& out_feature_observations) {

        out_feature_observations.timestamp = current_image_time;

        static etl::vector<Vector2f, MAX_NUMBER_OF_FEATURES>
            current_features_undistorted, initialisation_features_undistorted,
            previous_features_undistorted;

        undistort_points(current_features, current_features_undistorted);
        undistort_points(initialisation_features,
                         initialisation_features_undistorted);
        undistort_points(previous_features, previous_features_undistorted);

        // time interval between current and previous image
        const float dt_1        = current_image_time - previous_image_time;
        const bool prev_is_last = previous_image_time == last_update_time;
        const float dt_2 =
            (prev_is_last ? dt_1 : previous_image_time - last_update_time);

        for (size_t i = 0; i < feature_identifiers.size(); ++i) {
            out_feature_observations.observations.push_back(
                FeatureObservation());

            out_feature_observations.observations[i].identifier =
                feature_identifiers[i];
            out_feature_observations.observations[i].u =
                current_features_undistorted[i].x();
            out_feature_observations.observations[i].v =
                current_features_undistorted[i].y();
            out_feature_observations.observations[i].u_velocity =
                (current_features_undistorted[i].x() -
                 previous_features_undistorted[i].x()) /
                dt_1;
            out_feature_observations.observations[i].v_velocity =
                (current_features_undistorted[i].y() -
                 previous_features_undistorted[i].y()) /
                dt_1;

            if (initialisation_features[i].x() == -1 &&
                initialisation_features[i].y() == -1) {
                out_feature_observations.observations[i].u_init = -1;
                out_feature_observations.observations[i].v_init = -1;
            } else {
                out_feature_observations.observations[i].u_init =
                    initialisation_features_undistorted[i].x();
                out_feature_observations.observations[i].v_init =
                    initialisation_features_undistorted[i].y();

                initialisation_features[i].x() = -1;
                initialisation_features[i].y() = -1;

                if (prev_is_last) {
                    out_feature_observations.observations[i].u_init_velocity =
                        (current_features_undistorted[i].x() -
                         initialisation_features_undistorted[i].x()) /
                        dt_2;
                    out_feature_observations.observations[i].v_init_velocity =
                        (current_features_undistorted[i].y() -
                         initialisation_features_undistorted[i].y()) /
                        dt_2;
                } else {
                    out_feature_observations.observations[i].u_init_velocity =
                        (previous_features_undistorted[i].x() -
                         initialisation_features_undistorted[i].x()) /
                        dt_2;
                    out_feature_observations.observations[i].v_init_velocity =
                        (previous_features_undistorted[i].y() -
                         initialisation_features_undistorted[i].y()) /
                        dt_2;
                }
            }
        }
    }

    void configure(const Configuration& config) {

        configuration = config;

        image_state             = ImageState::FIRST_INITIALISATION_IMAGE;
        next_feature_identifier = 0;

        camera_intrinsics         = configuration.camera_intrinsics;
        camera_intrinsics_inverse = configuration.camera_intrinsics.inverse();
        camera_distortion_parameters =
            configuration.camera_distortion_parameters;

        T_camera_imu = configuration.T_camera_imu;
        T_imu_camera = configuration.T_imu_camera;

        R_currentframe_previousframe = Matrix3f::Identity();

        new_features.clear();
        new_feature_descriptors.clear();
        previous_features.clear();
        current_features.clear();
        feature_identifiers.clear();
        feature_lifetimes.clear();
        initialisation_features.clear();

        previous_image_time = 0.0;
        current_image_time  = 0.0;
        last_update_time    = 0.0;
        got_first_image     = false;

        fast_threshold = config.fast_threshold;
    }

    bool process_image(
        const ImageDataPoint& image_data_point,
        etl::vector<ImuDataPoint, MAX_NUMBER_OF_IMU_MEASUREMENTS>& imu_data,
        FeatureObservations& out_feature_observations) {

        if (!got_first_image) {

            // Here we just check if the image we are using has got some
            // associated IMU data before the image was captured
            if (!imu_data.empty() &&
                (imu_data.begin()->timestamp - image_data_point.timestamp <=
                 0.0f)) {
                got_first_image = true;
            } else {
                return false;
            }
        }

        ImagePyramid image_pyramid(image_data_point.image,
                                   upper_levels_image_pyramid_buffer);

        if (configuration.use_brief) {
            brief_descriptor.reset(image_data_point.image);
        }

        current_image_time = image_data_point.timestamp;

        bool have_features = false;

        switch (image_state) {
        case ImageState::FIRST_INITIALISATION_IMAGE:
            if (initialise_first_frame(image_pyramid)) {
                image_state      = ImageState::SECOND_INITIALISATION_IMAGE;
                last_update_time = current_image_time;
            }

            break;

        case ImageState::SECOND_INITIALISATION_IMAGE:

            if (!initialise_first_features(image_pyramid, imu_data)) {
                image_state = ImageState::FIRST_INITIALISATION_IMAGE;
            } else {

                if (current_image_time - last_update_time >=
                    0.9 * (1.0f / configuration.update_frequency)) {

                    find_new_features_to_track(*image_pyramid.at(0));

                    construct_feature_observations(out_feature_observations);

                    last_update_time = current_image_time;
                    have_features    = true;
                }

                image_state = ImageState::POST_INITIALISATION_STAGE;
            }

            break;

        case ImageState::POST_INITIALISATION_STAGE:

            integrate_imu_data(imu_data, R_currentframe_previousframe);

            track_features(image_pyramid);

            track_new_features(image_pyramid);

            if (current_image_time - last_update_time >=
                0.9f * (1.0f / configuration.update_frequency)) {
                find_new_features_to_track(*image_pyramid.at(0));

                construct_feature_observations(out_feature_observations);

                last_update_time = current_image_time;
                have_features    = true;
            }

            break;
        }

        previous_image_time = current_image_time;

        // Swap previous and current points
        previous_features.clear();

        for (const auto& current_feature : current_features) {

            if (previous_features.full()) {
                break;
            }

            previous_features.push_back(current_feature);
        }

        current_features.clear();

        new_features_patch_pyramid.construct(image_pyramid, new_features);
        previous_features_patch_pyramid.construct(image_pyramid,
                                                  previous_features);

        // If the pipeline hasn't been able to grab enough features from the
        // first frame or the previous amount of features was low, we reduce the
        // FAST threshold
        if ((!backend::has_initialised() &&
             previous_features.size() <
                 configuration
                     .threshold_for_reduced_fast_before_initialisation) ||
            (backend::has_initialised() &&
             previous_features.size() <
                 configuration
                     .threshold_for_reduced_fast_after_initialisation)) {
            fast_threshold = configuration.reduced_fast_threshold;
        } else {
            fast_threshold = configuration.fast_threshold;
        }

        return have_features;
    }
}
