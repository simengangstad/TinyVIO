#include "ground_truth_data_point.h"

#include <stdio.h>

void GroundTruthDataPoint::print() {
    printf("Timestamp: %lf\r\n", timestamp);

    printf("Position: [%f, %f, %f]\r\n",
           position.x(),
           position.y(),
           position.z());

    printf("Orientation: [%f, %f, %f, %f]\r\n",
           orientation.w(),
           orientation.x(),
           orientation.y(),
           orientation.z());

    printf("Velocity: [%f, %f, %f]\r\n",
           velocity.x(),
           velocity.y(),
           velocity.z());

    printf("Acceleration bias: [%f, %f, %f]\r\n",
           acceleration_bias.x(),
           acceleration_bias.y(),
           acceleration_bias.z());

    printf("Angular velocity bias: [%f, %f, %f]\r\n",
           angular_velocity_bias.x(),
           angular_velocity_bias.y(),
           angular_velocity_bias.z());
}

#ifndef CPU_MIMXRT1176DVMAA

#include <iomanip>
#include <iostream>
#include <ostream>

static Eigen::IOFormat eigen_io_formatter(4, 0, ", ", "\n", "[", "]");

std::ostream& operator<<(std::ostream& os,
                         const GroundTruthDataPoint& ground_truth_data_point) {

    os << std::left << std::setw(20) << "Time:" << std::setprecision(15)
       << ground_truth_data_point.timestamp << "\n";

    os << std::left << std::setw(20) << "Position:"
       << ground_truth_data_point.position.transpose().format(
              eigen_io_formatter)
       << "\n";

    os << std::left << std::setw(20) << "Velocity:"
       << ground_truth_data_point.velocity.transpose().format(
              eigen_io_formatter)
       << "\n";

    os << std::left << std::setw(20) << "Orientation:" << std::setprecision(4)
       << "[" << ground_truth_data_point.orientation.w() << ", "
       << ground_truth_data_point.orientation.x() << ", "
       << ground_truth_data_point.orientation.y() << ", "
       << ground_truth_data_point.orientation.z() << "]\n";

    os << std::left << std::setw(20) << "Acceleation bias:"
       << ground_truth_data_point.acceleration_bias.transpose().format(
              eigen_io_formatter)
       << "\n";

    os << std::left << std::setw(20) << "Angular velocity bias:"
       << ground_truth_data_point.angular_velocity_bias.transpose().format(
              eigen_io_formatter)
       << "\n";

    return os;
}

#endif
