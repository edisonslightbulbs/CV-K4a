#include <opencv2/opencv.hpp>

#include "chessboard.h"
#include "file.h"
#include "kinect.h"
#include "pcloud.h"
#include "point.h"
#include "projector.h"
#include "usage.h"

using t_RGBD = std::pair<cv::Mat, std::vector<Point>>;

bool calibrationProjector(
    Projector& projector, const cv::Size& dChessboard, const std::string& file)
{
    bool done = false;
    if (projector.m_RGBDCollection.size() > 15) {
        usage::prompt(CALIBRATING);
        projector.calibrate(dChessboard, chessboard::R_BLOCK_WIDTH);
        usage::prompt(SAVING_PARAMETERS);
        parameters::write(
            file, projector.m_K, projector.m_distortionCoefficients);
        done = true;
    } else {
        usage::prompt(MORE_IMAGES_REQUIRED);
    }
    return done;
}

t_RGBD getRGBData(std::shared_ptr<Kinect>& sptr_kinect)
{
    sptr_kinect->capture();
    sptr_kinect->depthCapture();
    sptr_kinect->pclCapture();
    sptr_kinect->imgCapture();
    sptr_kinect->c2dCapture();
    sptr_kinect->transform(RGB_TO_DEPTH);

    // get depth image dimensions
    int w = k4a_image_get_width_pixels(sptr_kinect->m_depth);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_depth);

    // get synchronous RGB-D captures
    auto* pCloudData
        = (int16_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_pcl);
    auto* rgbData = k4a_image_get_buffer(sptr_kinect->m_c2d);

    const std::string file = "./output/pcloud/3dPCloud.ply";
    std::vector<Point> pCloud = pcloud::build(w, h, pCloudData, rgbData);
    // pcloud::write(rgbdPCloud, file);
    cv::Mat frame
        = cv::Mat(h, w, CV_8UC4, (void*)rgbData, cv::Mat::AUTO_STEP).clone();

    sptr_kinect->releaseK4aCapture();
    sptr_kinect->releaseK4aImages();

    t_RGBD data = std::make_pair(frame, pCloud);

    // couple frame and point cloud
    return data;
}

int main()
{
    // initialize image frames and  kinect
    cv::Mat src, dst;
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // create projector
    Projector projector;

    // project chessboard
    cv::Size dChessboard = cv::Size(9, 6);
    chessboard::project(dChessboard);

    // setup calibration window
    std::string window = "calibration window";
    cv::namedWindow(window, cv::WINDOW_AUTOSIZE);

    // calibrate projector
    bool done = false;
    usage::prompt(USAGE);
    std::string file = "./output/calibration/projector.txt";

    while (!done) {
        t_RGBD rgbdData = getRGBData(sptr_kinect);

        src = rgbdData.first; // grab image from RGBD
        bool pass = chessboard::overlay(src, dst, dChessboard, window);

        int key = cv::waitKey(30);
        switch (key) {
        case ENTER_KEY: // capture synchronous RGBD
            chessboard::capture(pass, rgbdData, projector.m_RGBDCollection);
            break;
        case ESCAPE_KEY: // start calibration
            done = calibrationProjector(projector, dChessboard, file);
        default:
            break;
        }
    }
    return 0;
}
