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
    void scale()
    {
        // initialize resources
        const std::string INPUT = "scale-input";
        const std::string OUTPUT = "scale-output";
        cv::Mat src, dst;

        src = cv::imread("./resources/sample.png");

        // scale
        cv::Size dSize = cv::Size(300, 300);
        cv::resize(src, dst, dSize, 0, 0, cv::INTER_AREA);

        // show window
        cv::imshow(INPUT, src);
        cv::imshow(OUTPUT, dst);
        cv::waitKey();
    }

    void transform()
    {
        // initialize resources
        const std::string INPUT = "transform-input";
        const std::string OUTPUT = "transform-output";

        cv::Mat src, dst, M;
        src = cv::imread("./resources/sample.png");
        cv::Size dSize = cv::Size(src.rows, src.cols);

        // transform
        double arr[6] = { 1, 0, 50, 0, 1, 100 };
        M = cv::Mat(2, 3, CV_64FC1, arr);
        cv::warpAffine(src, dst, M, dSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                       cv::Scalar());

        // show window
        cv::imshow(INPUT, src);
        cv::imshow(OUTPUT, dst);
        cv::waitKey();
    }

    void rotate()
    {
        // initialize resources
        const std::string INPUT = "rotate-input";
        const std::string OUTPUT = "rotate-output";

        cv::Mat src, dst, M;
        src = cv::imread("./resources/sample.png");
        cv::Size dSize = cv::Size(src.rows, src.cols);

        // transform
        cv::Point center = cv::Point(src.cols / 2, src.rows / 2);
        M = cv::getRotationMatrix2D(center, 45, 1);
        cv::warpAffine(src, dst, M, dSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                       cv::Scalar());

        // show window
        cv::imshow(INPUT, src);
        cv::imshow(OUTPUT, dst);
        cv::waitKey();
    }


}
#endif // ICON_H
