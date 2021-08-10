#ifndef SCENE_H
#define SCENE_H

#include <opencv2/opencv.hpp>
#include <string>

#include "file.h"
#include "kinect.h"
#include "usage.h"

namespace scene {

void reproject(const std::string& window, const int& w, const int& h,
    cv::Mat& img, const cv::Mat& R, const cv::Mat& t)
{
    cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);

    // scale
    cv::Size dSize = cv::Size(w, h);
    cv::resize(img, img, dSize, 0, 0, cv::INTER_AREA);

    cv::namedWindow(window, cv::WINDOW_NORMAL);
    cv::setWindowProperty(
        window, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::imshow(window, img);
    cv::moveWindow(window, 3000, 0);
    cv::waitKey();
}

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

cv::Mat screenColor(const bool& contrast, const int& w, const int& h)
{
    cv::Mat black(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat white(h, w, CV_8UC3, cv::Scalar(255, 255, 255));

    if (contrast) {
        return white;
    } else {
        return black;
    }
}

// toggle white and black background image projections
void toggle(std::shared_ptr<Kinect>& sptr_kinect, const std::string& window,
    const int& w, const int& h, std::vector<cv::Mat>& scene)
{
    bool contrast = false;
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    cv::setWindowProperty(
        window, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    while (true) {
        cv::Mat img = screenColor(contrast, w, h);
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
            cv::destroyWindow(window);
            break;
        }
    }
}

// un-distort scene images
void undistort(cv::Mat& frame)
{
    // initialize image and camera matrix
    cv::Mat undistortedFrame, K, refinedK, distortionCoefficients;
    K = cv::Mat::eye(3, 3, CV_64F);
    usage::prompt(LOADING_CALIBRATION_PARAMETERS);

    std::string file = "./output/calibration/samples/camera.txt";
    parameters::read(file, K, distortionCoefficients);

    int alpha = 1;
    cv::Size dSize = cv::Size(frame.rows, frame.cols);
    refinedK = cv::getOptimalNewCameraMatrix(
        K, distortionCoefficients, dSize, alpha, dSize);
    cv::undistort(frame, undistortedFrame, K, distortionCoefficients, refinedK);
}

void saturate(const cv::Mat& src, cv::Mat& dst)
{
    int beta = 100;     // brightness | range 1 - 100
    double alpha = 3.0; // contrast | range 1.0 - 3.0]

    dst = cv::Mat::zeros(src.size(), src.type());
    for (int y = 0; y < src.rows; y++) {
        for (int x = 0; x < src.cols; x++) {
            for (int c = 0; c < src.channels(); c++) {
                dst.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(
                    alpha * src.at<cv::Vec3b>(y, x)[c] + beta);
            }
        }
    }
}

cv::Rect aop(const cv::Mat& background, const cv::Mat& foreground)
{
    // image subtraction
    cv::Mat colorDiff, gray_0, gray_1, grayDiff, sharpGray;
    cv::cvtColor(background, gray_0, cv::COLOR_BGR2GRAY);
    cv::cvtColor(foreground, gray_1, cv::COLOR_BGR2GRAY);

    colorDiff = background - foreground;
    // undistort(colorDiff);
    grayDiff = gray_0 - gray_1;

    // sharpen gray diff
    cv::Mat saturated;
    saturate(colorDiff, saturated);
    cv::cvtColor(saturated, sharpGray, cv::COLOR_BGR2GRAY);

    // remove noise using smoothing
    cv::Mat blurred, thresholded;
    cv::Size dBlur = cv::Size(33, 33);
    cv::GaussianBlur(sharpGray, blurred, dBlur, 0);

    // threshold to extract flux
    cv::threshold(
        blurred, thresholded, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

#define show 0
#if show == 1
    cv::imshow("Color difference", colorDiff);
    cv::imshow("Gray difference", grayDiff);
    cv::imshow("Saturated difference", saturated);
    cv::imshow("Sharpened gray difference", sharpGray);
    cv::imshow("Blurred gray difference", blurred);
    cv::imshow("OTSU threshold", thresholded);
#endif
    // get ROI boundary
    return cv::boundingRect(thresholded);
}

}
#endif // SCENE_H
