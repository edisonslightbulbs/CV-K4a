#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "pcloud.h"

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

std::vector<Point> pcloud::build(
    const int& w, const int& h, const int16_t* pCloudData, const uint8_t* bgra)
{
    std::vector<Point> pCloud;
    for (int i = 0; i < w * h; i++) {
        Point point {};
        point.m_xyz[0] = pCloudData[3 * i + 0];
        point.m_xyz[1] = pCloudData[3 * i + 1];
        point.m_xyz[2] = pCloudData[3 * i + 2];
        if (point.m_xyz[2] == 0) {
            continue;
        }
        uint8_t r = bgra[4 * i + 2];
        uint8_t g = bgra[4 * i + 1];
        uint8_t b = bgra[4 * i + 0];
        uint8_t a = bgra[4 * i + 3];
        uint8_t rgba[4] = { r, g, b, a };
        point.setRGBA(rgba);

        if (point.m_rgba[0] == 0 && point.m_rgba[1] == 0 && point.m_rgba[2] == 0
            && point.m_rgba[3] == 0) {
            continue;
        }
        pCloud.push_back(point);
    }
    return pCloud;
}

void pcloud::write(const int& w, const int& h, const int16_t* pCloudData,
    const uint8_t* rgbData, const std::string& file)
{
    std::vector<Point> pCloud = build(w, h, pCloudData, rgbData);

    PLY_HEADER;
    std::stringstream ss;
    for (auto& point : pCloud) {
        int16_t x = point.m_xyz[0];
        int16_t y = point.m_xyz[1];
        int16_t z = point.m_xyz[2];

        // k4a color image is in fact BGR (not RGB)
        auto r = (float)point.m_rgba[2];
        auto g = (float)point.m_rgba[1];
        auto b = (float)point.m_rgba[0];

        ss << x << " " << y << " " << z << " ";
        ss << r << " " << g << " " << b << std::endl;
    }
    std::ofstream ofs_text(file, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

void pcloud::write(const std::vector<Point>& pCloud, const std::string& file)
{
    PLY_HEADER;
    std::stringstream ss;
    for (auto& point : pCloud) {
        int16_t x = point.m_xyz[0];
        int16_t y = point.m_xyz[1];
        int16_t z = point.m_xyz[2];

        // k4a color image is in fact BGR (not RGB)
        auto r = (float)point.m_rgba[2];
        auto g = (float)point.m_rgba[1];
        auto b = (float)point.m_rgba[0];

        ss << x << " " << y << " " << z << " ";
        ss << r << " " << g << " " << b << std::endl;
    }
    std::ofstream ofs_text(file, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}
