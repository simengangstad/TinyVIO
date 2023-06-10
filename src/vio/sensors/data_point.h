#ifndef DATA_POINT_H
#define DATA_POINT_H

#ifndef CPU_MIMXRT1176DVMAA
#include <iostream>
#endif

enum class DataPointType { Imu = 0, Image = 1, GroundTruth = 2, None = 99 };

struct DataPoint {
    double timestamp;
    DataPointType type;

    DataPoint(const DataPointType data_point_type)
        : timestamp(0.0), type{data_point_type} {}

    virtual ~DataPoint() = default;

#ifndef CPU_MIMXRT1176DVMAA
    friend std::ostream& operator<<(std::ostream& os,
                                    const DataPoint& data_point);
#endif
};

#endif
