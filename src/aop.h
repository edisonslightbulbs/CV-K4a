#ifndef AREA_OF_PROJECTION_H
#define AREA_OF_PROJECTION_H

#include <opencv2/opencv.hpp>

namespace aop {

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

cv::Rect find(const cv::Mat& background, const cv::Mat& foreground)
{
    // todo: load camera calibration and undistort !!
    // image subtraction
    cv::Mat colorDiff, gray_0, gray_1, grayDiff, sharpGray;
    cv::cvtColor(background, gray_0, cv::COLOR_BGR2GRAY);
    cv::cvtColor(foreground, gray_1, cv::COLOR_BGR2GRAY);

    colorDiff = background - foreground;
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
#endif // AREA_OF_PROJECTION_H
