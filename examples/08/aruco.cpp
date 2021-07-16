#include <iostream>
#include <thread>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "kinect.h"
#include "parameters.h"
#include "usage.h"

cv::Mat grabFrame(std::shared_ptr<Kinect>& sptr_kinect)
{
    sptr_kinect->capture();
    sptr_kinect->imgCapture();
    uint8_t* data = k4a_image_get_buffer(sptr_kinect->m_img);
    int w = k4a_image_get_width_pixels(sptr_kinect->m_img);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_img);
    sptr_kinect->releaseK4aCapture();
    sptr_kinect->releaseK4aImages();
    return cv::Mat(h, w, CV_8UC4, (void*)data, cv::Mat::AUTO_STEP).clone();
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

    usage::prompt(LOADING_CALIBRATION_PARAMETERS);
    parameters::read("calibration.txt", cameraMatrix, coefficients);
    usage::prompt(FINDING_ARUCO_MARKERS);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCorners;

    cv::Ptr<cv::aruco::Dictionary> markerDictionary
        = cv::aruco::getPredefinedDictionary(
            cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    // measurement of square side in meters
    const float ARUCO_SQUARE_DIMENSION = 0.0565f;

    std::vector<cv::Vec3d> rVectors, tVectors;
    while (true) {
        frame = grabFrame(sptr_kinect);
        cv::cvtColor(frame, frame, cv::COLOR_BGRA2RGB);
        cv::aruco::detectMarkers(
            frame, markerDictionary, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners,
            ARUCO_SQUARE_DIMENSION, cameraMatrix, coefficients, rVectors,
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
