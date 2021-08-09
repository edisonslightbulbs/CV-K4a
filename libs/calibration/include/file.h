#ifndef PARAMETERS
#define PARAMETERS

#include <opencv2/opencv.hpp>
#include <string>

namespace parameters {
bool write(const std::string& name, cv::Mat cameraMatrix,
    cv::Mat distortionCoefficients);

bool read(const std::string& name, cv::Mat& cameraMatrix,
    cv::Mat& distanceCoefficients);
}
#endif // PARAMETERS
