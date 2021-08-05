#include <opencv2/opencv.hpp>

#include "file.h"
#include "kinect.h"
#include "usage.h"

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

int main()
{
    // initialize kinect and get image dimensions
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // setup windows
    const std::string INPUT = "un-distorting: input image";
    const std::string OUTPUT = "un-distorting: output image";
    cv::namedWindow(INPUT, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(OUTPUT, cv::WINDOW_AUTOSIZE);

    // initialize image and camera matrix
    cv::Mat src, dst, K, refinedK, distortionCoefficients;
    K = cv::Mat::eye(3, 3, CV_64F);
    usage::prompt(LOADING_CALIBRATION_PARAMETERS);

    std::string file = "./output/calibration/samples/camera.txt";
    parameters::read(file, K, distortionCoefficients);

    src = grabFrame(sptr_kinect);
    cv::Size dSize = cv::Size(src.rows, src.cols);

    /* UN-DISTORT:
     *   - alpha=0 returns undistorted image with min-unwanted pixels
     *   - alpha=1 returns undistorted image retaining black-image patches?
     */
    int alpha = 1;
    refinedK = cv::getOptimalNewCameraMatrix(
            K, distortionCoefficients, dSize, alpha, dSize);
    cv::undistort(src, dst, K, distortionCoefficients, refinedK);

    // todo: crop iff necessary

    // show
    cv::imshow(INPUT, src);
    cv::imshow(OUTPUT, dst);
    cv::waitKey();
    return 0;
}
