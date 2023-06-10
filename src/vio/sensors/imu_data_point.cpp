#include "imu_data_point.h"

#include <stdio.h>

void ImuDataPoint::print() {
    printf("Timestamp: %lf\r\n", timestamp);

    printf("Acceleration: [%f, %f, %f]\r\n",
           linear_acceleration.x(),
           linear_acceleration.y(),
           linear_acceleration.z());

    printf("Angular velocity: [%f, %f, %f]\r\n",
           angular_velocity.x(),
           angular_velocity.y(),
           angular_velocity.z());
}

#ifndef CPU_MIMXRT1176DVMAA

#include <iomanip>
#include <iostream>
#include <ostream>

static Eigen::IOFormat eigen_io_formatter(4, 0, ", ", "\n", "[", "]");

std::ostream& operator<<(std::ostream& os, const ImuDataPoint& imu_data_point) {

    os << std::left << std::setw(20) << "Timestamp:" << std::setprecision(15)
       << imu_data_point.timestamp << "\n";

    os << std::left << std::setw(20) << "Acceleation:"
       << imu_data_point.linear_acceleration.transpose().format(
              eigen_io_formatter)
       << "\n";

    os << std::left << std::setw(20) << "Angular velocity:"
       << imu_data_point.angular_velocity.transpose().format(eigen_io_formatter)
       << "\n";

    return os;
}
#endif
