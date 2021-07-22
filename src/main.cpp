// #include <iostream>
//
// int main()
// {
//     std::cout << "-- see in examples directory for the code " << std::endl;
//     std::cout << "-- see in build/bin directory for the binaries " <<
//     std::endl;
// }

#include "homography.h"
#include <opencv2/opencv.hpp>

int main()
{
    cv::Mat backgroundImg, foregroundImg;
    backgroundImg = cv::imread("resources/background.png", cv::IMREAD_COLOR);
    foregroundImg = cv::imread("resources/foreground.png", cv::IMREAD_COLOR);
    homography::initialize(backgroundImg, foregroundImg);
}
