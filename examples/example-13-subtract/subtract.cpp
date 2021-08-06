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
    cv::Mat gray_0, gray_1, mask, grayDiff, colorDiff, thresh;
    cv::cvtColor(scene[0], gray_0, cv::COLOR_BGR2GRAY);
    cv::cvtColor(scene[1], gray_1, cv::COLOR_BGR2GRAY);

#define ABSOLUTE 0
#define BACKGROUND 0
#define KNN_MODEL cv::createBackgroundSubtractorKNN()
#define MOG2_MODEL cv::createBackgroundSubtractorMOG2()

#if ABSOLUTE == 1
    cv::absdiff(gray_0, gray_1, grayDiff);
    cv::absdiff(scene[0], scene[1], colorDiff);
    cv::imshow("Gray background difference", grayDiff);
    cv::imshow("Color background difference", colorDiff);
#elif BACKGROUND == 1
    // create background subtractor: use either MOG2 or KNN
    cv::Ptr<cv::BackgroundSubtractor> subtractor = MOG2_MODEL;
    subtractor->apply(scene[0], mask);
    imshow("Frame", scene[0]);
    imshow("Mask", mask);
#else
    grayDiff = gray_0 - gray_1;
    colorDiff = scene[1] - scene[0];
    cv::imshow("Gray background difference", grayDiff);
    cv::imshow("Color background difference", colorDiff);
#endif

    cv::waitKey();
    return 0;
}
