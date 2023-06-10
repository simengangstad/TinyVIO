#ifndef GROUND_TRUTH_DATA_POINT_H
#define GROUND_TRUTH_DATA_POINT_H

#include "data_point.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

#ifndef CPU_MIMXRT1176DVMAA
#include <iostream>
#endif

struct GroundTruthDataPoint : public DataPoint {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
    Eigen::Vector3f velocity;
    Eigen::Vector3f acceleration_bias;
    Eigen::Vector3f angular_velocity_bias;

    GroundTruthDataPoint() : DataPoint(DataPointType::GroundTruth) {}

    void print();

#ifndef CPU_MIMXRT1176DVMAA

    friend std::ostream&
    operator<<(std::ostream& os,
               const GroundTruthDataPoint& ground_truth_data_point);

#endif
};
#endif
