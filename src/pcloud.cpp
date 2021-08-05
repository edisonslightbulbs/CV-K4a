#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include  "pcloud.h"

#define PLY_HEADER                                                             \
    std::ofstream ofs(file);                                                   \
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


std::vector<t_rgbd> pcloud::retrieve(const int& w, const int& h,
    const int16_t* pCloudData, const uint8_t* rgbData)
{
    std::vector<t_rgbd> pCloud;
    for (int i = 0; i < w * h; i++) {
        t_rgbd point {};
        point.xyz[0] = pCloudData[3 * i + 0];
        point.xyz[1] = pCloudData[3 * i + 1];
        point.xyz[2] = pCloudData[3 * i + 2];
        if (point.xyz[2] == 0) {
            continue;
        }
        point.rgb[0] = rgbData[4 * i + 0];
        point.rgb[1] = rgbData[4 * i + 1];
        point.rgb[2] = rgbData[4 * i + 2];
        uint8_t alpha = rgbData[4 * i + 3];

        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0
            && alpha == 0) {
            continue;
        }
        pCloud.push_back(point);
    }
    return pCloud;
}

void pcloud::write(const int& w, const int& h, const int16_t* pCloudData,
    const uint8_t* rgbData, const std::string& file)
{
    std::vector<t_rgbd> pCloud = retrieve(w, h, pCloudData, rgbData);

    PLY_HEADER;
    std::stringstream ss;
    for (auto& point : pCloud) {
        int16_t x = point.xyz[0];
        int16_t y = point.xyz[1];
        int16_t z = point.xyz[2];

        // k4a color image is in fact BGR (not RGB)
        auto r = (float)point.rgb[2];
        auto g = (float)point.rgb[1];
        auto b = (float)point.rgb[0];

        ss << x << " " << y << " " << z << " ";
        ss << r << " " << g << " " << b << std::endl;
    }
    std::ofstream ofs_text(file, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}
