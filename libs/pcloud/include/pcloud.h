#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "point.h"
#include <string>
#include <vector>

namespace pcloud {
std::vector<Point> build(
    const int& w, const int& h, const int16_t* pCloudData, const uint8_t* bgra);

void write(const int& w, const int& h, const int16_t* pCloudData,
    const uint8_t* rgbData, const std::string& file);

void write(const std::vector<Point>& pCloud, const std::string& file);
}
#endif // POINT_CLOUD_H
