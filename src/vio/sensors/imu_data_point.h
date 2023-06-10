#ifndef IMU_DATA_POINT_H
#define IMU_DATA_POINT_H

#include "data_point.h"
#include "vio_config.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#include <etl/vector.h>

#ifndef CPU_MIMXRT1176DVMAA
#include <iostream>
#endif

struct ImuDataPoint : DataPoint {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3f linear_acceleration;
    Eigen::Vector3f angular_velocity;

    ImuDataPoint() : DataPoint(DataPointType::Imu) {}

    ImuDataPoint(float timestamp_,
                 float wx,
                 float wy,
                 float wz,
                 float ax,
                 float ay,
                 float az)
        : DataPoint(DataPointType::Imu) {

        timestamp = timestamp_;

        linear_acceleration.x() = ax;
        linear_acceleration.y() = ay;
        linear_acceleration.z() = az;

        angular_velocity.x() = wx;
        angular_velocity.y() = wy;
        angular_velocity.z() = wz;
    }

    ImuDataPoint(const double timestamp_,
                 const Eigen::Vector3f& angular_velocity_,
                 const Eigen::Vector3f& linear_acceleration_)
        : DataPoint(DataPointType::Imu) {

        timestamp           = timestamp_;
        linear_acceleration = linear_acceleration_;
        angular_velocity    = angular_velocity_;
    }

    void print();

#ifndef CPU_MIMXRT1176DVMAA
    friend std::ostream& operator<<(std::ostream& os,
                                    const ImuDataPoint& imu_data_point);
#endif
};

/**
 * @brief Convenience typedef for a range of IMU data points.
 */
typedef etl::vector<ImuDataPoint, MAX_NUMBER_OF_IMU_MEASUREMENTS> IMUBuffer;

#endif
