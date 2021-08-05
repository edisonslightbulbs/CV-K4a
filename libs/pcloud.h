#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <string>
#include <vector>

struct t_rgbd {
    int16_t xyz[3];
    uint8_t rgb[3];
};

namespace pcloud {
std::vector<t_rgbd> retrieve(const int& w, const int& h, const int16_t* pCloudData, const uint8_t* rgbData);

void write(const int& w, const int& h, const int16_t* pCloudData, const uint8_t* rgbData, const std::string& file);

    void write(const std::vector<t_rgbd> &pCloud, const std::string &file);
}
#endif // POINT_CLOUD_H
