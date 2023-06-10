#include "data_point.h"

#ifndef CPU_MIMXRT1166DVM6A

#include <iomanip>
#include <iostream>
#include <ostream>

std::ostream& operator<<(std::ostream& os, const DataPoint& data_point) {

    os << std::left << std::setw(20) << "Time:" << std::setprecision(15)
       << data_point.timestamp << "\n";

    std::string type;

    switch (data_point.type) {
    case DataPointType::Imu:
        type = "IMU";
        break;
    case DataPointType::Image:
        type = "Image";
        break;
    case DataPointType::GroundTruth:
        type = "Ground truth";
        break;
    case DataPointType::None:
        type = "None";
        break;
    }

    os << std::left << std::setw(20) << "Type:" << type << "\n";

    return os;
}
#endif
