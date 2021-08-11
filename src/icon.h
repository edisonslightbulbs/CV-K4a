#ifndef ICON_H
#define ICON_H

#include <opencv2/opencv.hpp>
#include <string>

namespace icon {

    cv::Mat load(const std::string& path){
        cv::Mat icon = cv::imread(path, cv::IMREAD_UNCHANGED);

        // split channels of transparent icon
        cv::Mat bgra;
        cv::cvtColor(icon, bgra, cv::COLOR_BGR2BGRA);
        cv::Mat bgraSplit[4];
        cv::split(bgra, bgraSplit);

        // channel of interest to BGR image format
        cv::Mat layer;
        cv::cvtColor(bgraSplit[3], layer, cv:: COLOR_GRAY2BGR);
        return layer;
    }


}
#endif // ICON_H
