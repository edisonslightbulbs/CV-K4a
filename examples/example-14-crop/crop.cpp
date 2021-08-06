#include "scene.h"
#include <opencv2/opencv.hpp>

#if __linux__
#include "kinect.h"
cv::Mat grabFrame(std::shared_ptr<Kinect>& sptr_kinect)
{
    sptr_kinect->capture();
    sptr_kinect->depthCapture();
    sptr_kinect->pclCapture();
    sptr_kinect->imgCapture();
    sptr_kinect->c2dCapture();
    sptr_kinect->transform(RGB_TO_DEPTH);

    auto* rgbData = k4a_image_get_buffer(sptr_kinect->m_c2d);
    int w = k4a_image_get_width_pixels(sptr_kinect->m_c2d);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_c2d);

    cv::Mat frame
        = cv::Mat(h, w, CV_8UC4, (void*)rgbData, cv::Mat::AUTO_STEP).clone();

    sptr_kinect->releaseK4aCapture();
    sptr_kinect->releaseK4aImages();

    return frame;
}
#endif

int main()
{
    std::vector<cv::Mat> scene(2);

#if __linux__
    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);
#endif

#if __linux__
    scene::flicker(sptr_kinect, window, w, h, scene);
    scene::write(scene);
#endif

#if __APPLE__
    // load background scene images
    scene::load(scene);
#endif

    // initialize images
    cv::Mat gray_0, gray_1, grayDiff, thresh;
    cv::cvtColor(scene[0], gray_0, cv::COLOR_BGR2GRAY);
    cv::cvtColor(scene[1], gray_1, cv::COLOR_BGR2GRAY);

    // image subtraction
    grayDiff = gray_0 - gray_1;

    // cropping region of interest using
    cv::threshold(grayDiff, thresh, 0, 255, cv::THRESH_OTSU);
    cv::Rect roiBoundary = cv::boundingRect(thresh);
    cv::Mat roi = scene[0](roiBoundary);
    cv::imshow("Cropped ROI", roi);
    cv::imshow("OTSU threshold", thresh);

    cv::waitKey();
    return 0;
}
