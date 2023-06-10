#ifndef IMAGE_DATA_POINT_H
#define IMAGE_DATA_POINT_H

#include "data_point.h"

#include "image.h"

#ifndef CPU_MIMXRT1176DVMAA
#include <filesystem>
#include <iostream>
#endif

struct ImageDataPoint : public DataPoint {
    frontend::Image image;

    ImageDataPoint() : DataPoint(DataPointType::Image) {}

    void print();

#ifndef CPU_MIMXRT1176DVMAA

    std::filesystem::path image_file_path;

    friend std::ostream& operator<<(std::ostream& os,
                                    const ImageDataPoint& image_data_point);
#endif
};

#endif
