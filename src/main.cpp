// #include "scene.h"
//
// void project(const std::string& window, const int& w, const int& h,  std::shared_ptr<kinect>& sptr_kinect){
//     cv::Mat surface = scene::grabFrame(sptr_kinect);
//     cv::rotate(surface, surface, cv::ROTATE_90_CLOCKWISE);
//
//     // scale
//     cv::Size dSize = cv::Size(w, h);
//     cv::resize(surface, surface, dSize, 0, 0, cv::INTER_AREA);
//
//     cv::namedWindow(window, cv::WINDOW_NORMAL);
//     cv::setWindowProperty(
//             window, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
//     cv::imshow(window, surface);
//     cv::moveWindow(window, 3000, 0);
//
//     //cv::waitKey();
// }
//
// // todo:
// //  prep: abstract homography into a library (just do a homography over the whole image and not clicked points)
// //  then ...
// //  A. get scene using kinect
// //  B. extract projection area as a foreground image
// //  D. overlay foreground image on background image
// //  then...
// //  create 3 windows
// //  (1) showing the original kinect image
// //  (2) showing the black image
// //  (3) show the overlayed image
//
//
// int main()
// {
//     std::vector<cv::Mat> sceneImages;
//
//     // initialize kinect
//     std::shared_ptr<kinect> sptr_kinect(new kinect);
//
//     int w = 1366;
//     int h = 768;
//     const std::string window = "re-projection window";
//     // -- scene::alternateDisplayColor(sptr_kinect, window, w, h, sceneImages);
//
//     // find area of projection (aop)
//     // -- cv::Rect boundary = scene::findProjectionArea(sceneImages[1], sceneImages[0]);
//     // -- cv::Mat background = sceneImages[1];
//
//     // scene::undistort(background);
//     // -- cv::Mat roi = background(boundary);
//
//     // reproject aop
//     // -- cv::Mat R, t;
//     // scene::predistort(background);
//     // -- scene::project(window, w, h, roi, R, t);
//     // cv::imshow("Region of interest", roi);
//
//
//     // while(true){
//     //     project(window, w, h, sptr_kinect);
//     //
//     //     if (cv::waitKey(1) == 27) {
//     //         exit(0);
//     //     }
//     // }
//
//     return 0;
// }
//
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

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
    cv::Mat background = imread("./main.jpg", cv::IMREAD_COLOR); // background image
    cv::Mat foreground = imread("./logo.jpg", cv::IMREAD_COLOR); // foreground image
    std::vector<cv::Point2f> backgroundRoi;                                     // background region (4 corners)
    std::vector<cv::Point2f> foregroundRoi;                                     // projection region (4 corners)
    cv::Mat warpedForeground;                                                   // homography-warped image

    cv::namedWindow("background", cv::WINDOW_AUTOSIZE); // Create a window for display.
    cv::namedWindow("foreground", cv::WINDOW_AUTOSIZE); // Create a window for display.
    cv::namedWindow("homography", cv::WINDOW_AUTOSIZE); // Create a window for display.

    // initialize correspondence between background and foreground images
    backgroundRoi.emplace_back(float(0), float(0));
    backgroundRoi.emplace_back(float(0), float(foreground.rows));
    backgroundRoi.emplace_back(float(foreground.cols), float(foreground.rows));
    backgroundRoi.emplace_back(float(foreground.cols), float(0));

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
