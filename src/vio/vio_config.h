#ifndef VIO_CONFIG_H
#define VIO_CONFIG_H

#define MAX_NUMBER_OF_AUGMENTED_STATES               (15)
#define MAX_NUMBER_OF_IMU_MEASUREMENTS               (3000)
#define MAX_NUMBER_OF_FEATURES                       (50)
#define MAX_NUMBER_OF_OBSERVATIONS                   (MAX_NUMBER_OF_AUGMENTED_STATES)
#define MINIMUM_REQUIRED_FEATURES_FOR_INITIALISATION (20)

#define MAX_NUMBER_OF_FEATURE_TRACKS (2 * MAX_NUMBER_OF_FEATURES)

#define IMAGE_WIDTH  (752)
#define IMAGE_HEIGHT (480)

#define PYRAMID_LEVELS (4)

#include <etl/vector.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#ifndef CPU_MIMXRT1176DVMAA
#define PRINTF printf
#else
#include "logger.h"
#define PRINTF logger::infof
#endif

struct Configuration {

    bool visualise;

    // --- Backend switches ---

    bool estimate_extrinsics;
    bool estimate_time_compensation;

    // --- Camera intrinsics ----

    Eigen::Vector2i camera_resolution;

    Eigen::Matrix3f camera_intrinsics;
    Eigen::Vector4f camera_distortion_parameters;

    float initial_timestamp_compensation;

    // --- Camera extrinsics ---

    Eigen::Isometry3f T_camera_imu;
    Eigen::Isometry3f T_imu_camera;

    // --- Frontend parameters ---

    bool use_brief;
    uint32_t brief_threshold;

    int fast_threshold;

    /**
     * @brief The FAST threshold used in the frontend will be set to this when
     * either the frontend didn't extract enough features during initialisation
     * or the size of the set of features from the previous frame was low.
     */
    int reduced_fast_threshold;

    /**
     * @brief If the number of tracked features is below this threshold before
     * initialisation, the frontend will reduce its FAST threshold to
     * #reduced_fast_threshold in order get a better chance at initialisation.
     */
    size_t threshold_for_reduced_fast_before_initialisation;

    /**
     * @brief If the number of tracked features is below this threshold after
     * initialisation, the frontend will reduce its FAST threshold to
     * #reduced_fast_threshold in order to attempt to get more features.
     */
    size_t threshold_for_reduced_fast_after_initialisation;

    int lucas_kanade_max_iterations;

    /**
     * @brief If a feature extracted is within this threshold, it will be deemed
     * the same feature and discarded.
     */
    int minimum_distance_between_features;

    /**
     * @brief Used to make sure that we are discarding features which have been
     * tracked for a given number of frames, to force a better distribution of
     * features in the image.
     */
    uint32_t maximum_feature_lifetime;

    /**
     * @brief Rate of features published by front-end. This variable is used to
     * determine the timing threshold of each iteration of the filter. And
     * decide how many images to be used for static initialiser.
     */
    int update_frequency;

    /**
     * @brief Threshold for determine reference frames.
     */
    float rotation_threshold;
    float translation_threshold;
    float tracking_rate_threshold;

    // --- Feature triangulation parameters ---

    /**
     * @brief Least amount of observations for a valid feature.
     */
    int least_amount_of_observations;

    /**
     * @brief Max track length for feature.
     */
    int max_track_length;

    float feature_translation_threshold;

    // --- IMU and camera measurement noise parameters ---

    double noise_gyro;
    double noise_accelerometer;
    double noise_gyro_bias;
    double noise_accelerometer_bias;
    double noise_feature;

    // --- Initial covariance ---

    double initial_covariance_position;
    double initial_covariance_velocity;
    double initial_covariance_orientation;
    double initial_covariance_gyro_bias;
    double initial_covariance_accelerometer_bias;
    double initial_covariance_extrinsics_rotation;
    double initial_covariance_extrinsics_translation;

    // --- Zero update settings ---

    double zero_update_max_feature_distance;
    double zero_update_noise_v;
    double zero_update_noise_p;
    double zero_update_noise_q;

    // --- Static initialisation settings ---

    /**
     * @brief Static scene duration for inclinometer-initializer to utlize, in
     * seconds
     */
    float static_duration;

    // --- Measurement rate ---

    int image_rate;
    int imu_rate;

    Configuration() {}

    static Configuration euroc_configuration() {
        Configuration configuration;

        configuration.visualise = false;

        // --- Backend switches ---

        configuration.estimate_extrinsics        = true;
        configuration.estimate_time_compensation = true;

        // --- Camera intrinsics ----

        configuration.camera_resolution.x() = 752;
        configuration.camera_resolution.y() = 480;

        const float f_x = 458.654;
        const float f_y = 457.296;
        const float c_x = 367.215;
        const float c_y = 248.375;

        configuration.camera_intrinsics << f_x, 0.0f, c_x, 0.0f, f_y, c_y, 0.0f,
            0.0f, 1.0f;

        configuration.camera_distortion_parameters << -0.28340811, 0.07395907,
            0.00019359, 1.76187114e-05;

        configuration.initial_timestamp_compensation = 0.0f;

        // --- Camera extrinsics ---

        configuration.T_camera_imu.linear() << 0.014865542981794,
            0.999557249008346, -0.025774436697440, -0.999880929698575,
            0.014967213324719, 0.003756188357967, 0.004140296794224,
            0.025715529947966, 0.999660727177902;

        configuration.T_camera_imu.translation() << 0.065222909535531,
            -0.020706385492719, -0.008054602460030;

        configuration.T_imu_camera = configuration.T_camera_imu.inverse();

        // --- Frontend parameters ---
        configuration.use_brief       = true;
        configuration.brief_threshold = 70;

        configuration.fast_threshold                                   = 80;
        configuration.reduced_fast_threshold                           = 20;
        configuration.threshold_for_reduced_fast_before_initialisation = 15;
        configuration.threshold_for_reduced_fast_after_initialisation  = 5;
        configuration.lucas_kanade_max_iterations                      = 30;
        configuration.minimum_distance_between_features                = 20;
        configuration.update_frequency                                 = 10;
        configuration.maximum_feature_lifetime                         = 60;

        // --- Online reset thresholds ---

        configuration.rotation_threshold      = 0.2618;
        configuration.translation_threshold   = 0.4;
        configuration.tracking_rate_threshold = 0.5;

        // --- Feature triangulation parameters ---

        configuration.least_amount_of_observations  = 3;
        configuration.max_track_length              = 6;
        configuration.feature_translation_threshold = -1.0;

        // --- IMU and camera measurement noise parameters ---

        configuration.noise_gyro               = 0.004;
        configuration.noise_accelerometer      = 0.08;
        configuration.noise_gyro_bias          = 2.0e-6;
        configuration.noise_accelerometer_bias = 4.0e-5;
        configuration.noise_feature            = 0.008;

        // Convert standard deviation to variance
        configuration.noise_gyro *= configuration.noise_gyro;
        configuration.noise_accelerometer *= configuration.noise_accelerometer;
        configuration.noise_gyro_bias *= configuration.noise_gyro_bias;
        configuration.noise_accelerometer_bias *=
            configuration.noise_accelerometer_bias;
        configuration.noise_feature *= configuration.noise_feature;

        // --- Initial covariance ---

        configuration.initial_covariance_orientation            = 4.0e-4;
        configuration.initial_covariance_velocity               = 0.25;
        configuration.initial_covariance_position               = 1.0;
        configuration.initial_covariance_gyro_bias              = 4.0e-4;
        configuration.initial_covariance_accelerometer_bias     = 0.01;
        configuration.initial_covariance_extrinsics_rotation    = 3.0462e-8;
        configuration.initial_covariance_extrinsics_translation = 9.0e-8;

        // --- Zero update settings ---

        configuration.zero_update_max_feature_distance = 2.0e-3;
        configuration.zero_update_noise_v              = 1.0e-2;
        configuration.zero_update_noise_p              = 1.0e-2;
        configuration.zero_update_noise_q              = 3.4e-2;
        configuration.zero_update_noise_v *= configuration.zero_update_noise_v;
        configuration.zero_update_noise_p *= configuration.zero_update_noise_p;
        configuration.zero_update_noise_q *= configuration.zero_update_noise_q;

        // --- Static initialisation settings ---

        configuration.static_duration = 1.0;

        // --- Measurement rate ---

        configuration.image_rate = 20;
        configuration.imu_rate   = 200;

        return configuration;
    }

    void print() const {
        printf("\r\n\r\n");
        PRINTF("====================== Config ======================\r\n");

        PRINTF("Fast threshold: %d\r\n", fast_threshold);

        if (use_brief) {
            PRINTF("BRIEF threshold: %d\r\n", brief_threshold);
        }

        if (estimate_time_compensation) {
            PRINTF("Estimating time compensation, initial value: %.2f\r\n",
                   initial_timestamp_compensation);
        }

        if (estimate_extrinsics) {
            PRINTF("Estimating extrinsics\r\n");
        }

        PRINTF("====================================================\r\n\r\n");
    }
};

#endif
