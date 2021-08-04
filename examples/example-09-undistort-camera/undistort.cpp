#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

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

    // initialize resources
    const std::string INPUT = "un-distorting: input image";
    const std::string OUTPUT = "un-distorting: output image";
    const std::string CAMERA_PARAMETERS = "./output/calibration/camera/calibration.txt";

    cv::namedWindow(INPUT, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(OUTPUT, cv::WINDOW_AUTOSIZE);

    cv::Mat src, dst, cameraMatrix, newCameraMatrix, k;

    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    usage::prompt(LOADING_CALIBRATION_PARAMETERS);

    parameters::read( CAMERA_PARAMETERS, cameraMatrix, k);

    src = grabFrame(sptr_kinect);
    cv::Size dSize = cv::Size(src.rows, src.cols);

    /* UN-DISTORT:
     *   - alpha=0 returns undistorted image with min-unwanted pixels
     *   - alpha=1 returns undistorted image retaining black-image patches?
     */
    int alpha = 1;
    newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, k, dSize, alpha, dSize);
    cv::undistort(src, dst, cameraMatrix, k, newCameraMatrix);

    // todo: crop iff necessary

    // show
    cv::imshow(INPUT, src);
    cv::imshow(OUTPUT, dst);
    cv::waitKey();

    // todo: stream implementation
    // while (true) {
    //     frame = grabFrame(sptr_kinect);

    //     // show frame
    //     cv::imshow("kinect", frame);

    //     int key = cv::waitKey(30);
    //     if (key == 27)
    //         break;
    // }
}
