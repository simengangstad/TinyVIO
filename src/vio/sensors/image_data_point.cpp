#include "image_data_point.h"

#include <stdio.h>

void ImageDataPoint::print() {
    printf("Timestamp: %lf\r\n", timestamp);
    printf("Resolution: %u x %u\r\n", image.width, image.height);
}

#ifndef CPU_MIMXRT1176DVMAA

#include <iomanip>
#include <iostream>
#include <ostream>

std::ostream& operator<<(std::ostream& os,
                         const ImageDataPoint& image_data_point) {

    os << std::left << std::setw(20) << "Time:" << std::setprecision(15)
       << image_data_point.timestamp << "\n";

    os << std::left << std::setw(20)
       << "File name:" << image_data_point.image_file_path << "\n";

    return os;
}

#endif
