#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <fstream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

#define PLY_HEADER                                                             \
    std::ofstream ofs(FILE);                                                   \
    ofs << "ply" << std::endl;                                                 \
    ofs << "format ascii 1.0" << std::endl;                                    \
    ofs << "element vertex"                                                    \
        << " " << pCloud.size() << std::endl;                                  \
    ofs << "property float x" << std::endl;                                    \
    ofs << "property float y" << std::endl;                                    \
    ofs << "property float z" << std::endl;                                    \
    ofs << "property uchar red" << std::endl;                                  \
    ofs << "property uchar green" << std::endl;                                \
    ofs << "property uchar blue" << std::endl;                                 \
    ofs << "end_header" << std::endl;                                          \
    ofs.close()

struct t_rgbd {
    int16_t xyz[3];
    uint8_t rgb[3];
};

namespace pcl {
void write(const k4a_image_t& pclImage, const k4a_image_t& rgbImage,
    const std::string& FILE)

    void write(const int& width, const int, int16_t* pCloudData, uint8_t* rgbData)
{
    std::vector<t_rgbd> pCloud;
    int width = k4a_image_get_width_pixels(pclImage);
    int height = k4a_image_get_height_pixels(rgbImage);

    auto* point_cloud_image_data
        = (int16_t*)(void*)k4a_image_get_buffer(pclImage);
    uint8_t* color_image_data = k4a_image_get_buffer(rgbImage);

    for (int i = 0; i < width * height; i++) {
        t_rgbd point {};
        point.xyz[0] = point_cloud_image_data[3 * i + 0];
        point.xyz[1] = point_cloud_image_data[3 * i + 1];
        point.xyz[2] = point_cloud_image_data[3 * i + 2];
        if (point.xyz[2] == 0) {
            continue;
        }
        point.rgb[0] = color_image_data[4 * i + 0];
        point.rgb[1] = color_image_data[4 * i + 1];
        point.rgb[2] = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];

        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0
            && alpha == 0) {
            continue;
        }
        pCloud.push_back(point);
    }
    /** write to file */
    PLY_HEADER;
    std::stringstream ss;
    for (auto& point : pCloud) {
        // k4a color image is in fact BGR (not RGB)
        ss << (float)point.xyz[0] << " " << (float)point.xyz[1] << " "
           << (float)point.xyz[2];
        ss << " " << (float)point.rgb[2] << " " << (float)point.rgb[1] << " "
           << (float)point.rgb[0];
        ss << std::endl;
    }
    std::ofstream ofs_text(FILE, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}
}
#endif // POINT_CLOUD_H
