#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

cv::Mat background;                     // background image
cv::Mat foreground;                     // foreground image
// std::vector<cv::Point2f> backgroundRoi; // background region (4 corners)
// std::vector<cv::Point2f> foregroundRoi; // projection region (4 corners)

void overlay(cv::Mat& src, cv::Mat& dst)
{
    cv::Mat gray, grayCopy, grayInv, grayInvCopy;
    cv::cvtColor(dst, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, gray, 0, 255, cv::THRESH_BINARY);
    cv::bitwise_not(gray, grayInv);

    dst.copyTo(grayCopy, gray);
    src.copyTo(grayInvCopy, grayInv);

    cv::Mat homography = grayInvCopy + grayCopy;
    cv::imshow("homography", homography);
    cv::waitKey(0);
}

int main()
{
    background = imread("./main.jpg", cv::IMREAD_COLOR);
    foreground = imread("./logo.jpg", cv::IMREAD_COLOR);

    // initialize correspondence between background and foreground images
    backgroundRoi.emplace_back(float(0), float(0));
    backgroundRoi.emplace_back(float(0), float(foreground.rows));
    backgroundRoi.emplace_back(float(foreground.cols), float(foreground.rows));
    backgroundRoi.emplace_back(float(foreground.cols), float(0));

    cv::namedWindow("homography", cv::WINDOW_AUTOSIZE); // Create a window for display.
    cv::imshow("homography", background);

    std::cout << "-- computing homography " << std::endl;
    cv::Mat homography = cv::findHomography(backgroundRoi, foregroundRoi, 0);
    cv::warpPerspective(foreground, warpedForeground, homography, background.size());
    overlay(background, warpedForeground);

    // while (true) {
    //     int key = cv::waitKey(10);
    //     if (key == 27)
    //         break;
    // }

    return 0;
}
