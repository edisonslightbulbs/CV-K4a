#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "file.h"
#include "kinect.h"
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
    // initialize camera
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // initialize camera matrix and distortion coefficients
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortionCoefficients;

    // setup images and window
    cv::Mat src;
    std::string window = "World full of markers";
    cv::namedWindow(window, cv::WINDOW_AUTOSIZE);

    usage::prompt(LOADING_CALIBRATION_PARAMETERS);
    std::string file = "./output/calibration/samples/camera.txt";
    parameters::read(file, K, distortionCoefficients);

    usage::prompt(FINDING_ARUCO_MARKERS);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCorners;
    cv::Ptr<cv::aruco::Dictionary> markerDictionary
        = cv::aruco::getPredefinedDictionary(
            cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    const float ARUCO_BLOCK_WIDTH = 0.0565f;
    std::vector<cv::Vec3d> R, t;

    while (true) {
        src = grabFrame(sptr_kinect);
        cv::cvtColor(src, src, cv::COLOR_BGRA2RGB);
        cv::aruco::detectMarkers(
            src, markerDictionary, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(
            markerCorners, ARUCO_BLOCK_WIDTH, K, distortionCoefficients, R, t);

        // draw axis on detected marker
        for (int i = 0; i < markerIds.size(); i++) {
            cv::aruco::drawAxis(
                src, K, distortionCoefficients, R[i], t[i], 0.1f);
        }

        // show frame
        cv::imshow(window, src);

        int key = cv::waitKey(30);
        if (key == 27)
            break;
    }
}
