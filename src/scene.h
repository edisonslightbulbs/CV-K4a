#ifndef SCENE_H
#define SCENE_H

#include <opencv2/opencv.hpp>
#include <string>

namespace scene {

void load(std::vector<cv::Mat>& scene)
{
    const std::string file_0 = "./output/scene/black.png";
    const std::string file_1 = "./output/scene/white.png";
    cv::Mat img_0 = cv::imread(file_0, cv::IMREAD_COLOR);
    cv::Mat img_1 = cv::imread(file_1, cv::IMREAD_COLOR);
    scene[0] = img_0;
    scene[1] = img_1;
}

void write(std::vector<cv::Mat> scene)
{
    const std::string file_0 = "./output/scene/black.png";
    const std::string file_1 = "./output/scene/white.png";
    cv::imwrite(file_0, scene[0]);
    cv::imwrite(file_1, scene[1]);
}

cv::Mat contrast(const bool& contrast, const int& w, const int& h)
{
    cv::Mat black(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat white(h, w, CV_8UC3, cv::Scalar(255, 255, 255));

    if (contrast) {
        return white;
    } else {
        return black;
    }
}

#if __linux__
void flicker(std::shared_ptr<Kinect>& sptr_kinect, const std::string& window,
    const int& w, const int& h, std::vector<cv::Mat>& scene)
{
    bool contrast = false;
    // flicker black then white images
    while (true) {
        cv::Mat img = contrastBackground(contrast, w, h);
        cv::imshow(window, img);
        cv::moveWindow(window, 3000, 0);
        if (cv::waitKey(2000) >= 0) {
            break;
        }

        // capture scene
        cv::Mat frame = grabFrame(sptr_kinect);
        scene.emplace_back(frame);
        contrast = !contrast;

        if (scene.size() == 2) {
            cv::waitKey(1000);
            break;
        }
    }
}
#endif

}
#endif // SCENE_H
