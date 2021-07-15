// #include <iostream>
//
// int main(){
//     std::cout << "-- have a look the examples directory for code examples! "
//     << std::endl; std::cout << "-- have a look the build/bin directory for
//     the binaries! " << std::endl;
// }
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "../examples/07/parameters.h"
#include "aruco.h"
#include "kinect.h"

#define READING_CALIBRATION_PARAMETERS 4
#define FINDING_ARUCO_MARKERS 5

void usage(const int& code)
{
    switch (code) {
    case (4):
        std::cout << "-- loading calibration parameters from disk" << std::endl;
        break;
    case (5):
        std::cout << "-- searching for aruco markers" << std::endl;
        break;
    default:
        std::cout << "-- well now, wasn't expecting that one bit" << std::endl;
        break;
    }
}

cv::Mat grabFrame(std::shared_ptr<Kinect>& sptr_kinect)
{
    sptr_kinect->getFrame(RGB_TO_DEPTH);
    uint8_t* data = k4a_image_get_buffer(sptr_kinect->m_rgbImage);
    int w = k4a_image_get_width_pixels(sptr_kinect->m_rgbImage);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_rgbImage);
    sptr_kinect->release();
    return cv::Mat(h, w, CV_8UC4, (void*)data, cv::Mat::AUTO_STEP);
}

int main()
{
    // initialize kinect and get image dimensions
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // setup camera matrix and initialize coefficients
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat coefficients;

    // initialize named window and frames for superimposing
    cv::namedWindow("kinect", cv::WINDOW_AUTOSIZE);
    cv::Mat frame, frameCopy;

    usage(READING_CALIBRATION_PARAMETERS);
    parameters::read("calibration.txt", cameraMatrix, coefficients);
    usage(FINDING_ARUCO_MARKERS);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCorners;

    cv::Ptr<cv::aruco::Dictionary> markerDictionary
        = cv::aruco::getPredefinedDictionary(
            cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    std::vector<cv::Vec3d> rVectors, tVectors;
    while (true) {
        frame = grabFrame(sptr_kinect);
        cv::cvtColor(frame, frame, cv::COLOR_BGRA2RGB);
        cv::aruco::detectMarkers(
            frame, markerDictionary, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners,
            aruco::arucoSquareDimension, cameraMatrix, coefficients, rVectors,
            tVectors);

        // draw axis on detected marker
        for (int i = 0; i < markerIds.size(); i++) {
            cv::aruco::drawAxis(frame, cameraMatrix, coefficients, rVectors[i],
                tVectors[i], 0.1f);
        }

        // show frame
        cv::imshow("kinect", frame);
        if (cv::waitKey(30) >= 0)
            break;
    }
}
