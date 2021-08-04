#include <opencv2/opencv.hpp>

#include "camera.h"
#include "file.h"
#include "kinect.h"
#include "usage.h"

bool calibrateCamera(
    Camera& camera, const cv::Size& dChessboard, const std::string& file)
{
    bool done = false;
    if (camera.m_images.size() > 15) {
        usage::prompt(CALIBRATING);
        camera.calibrate(dChessboard, chessboard::R_BLOCK_WIDTH);
        usage::prompt(SAVING_PARAMETERS);
        parameters::write(file, camera.m_K, camera.m_distortionCoefficients);
        done = true;
    } else {
        usage::prompt(MORE_IMAGES_REQUIRED);
    }
    return done;
}

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
    // initialize camera, image frames, and  window
    Camera camera;
    cv::Mat src, dst;
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    std::string window = "calibration window";
    cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
    const cv::Size dChessboard = cv::Size(9, 6);

    // start calibration
    bool done = false;
    usage::prompt(USAGE);
    std::string file = "./output/calibration/camera.txt";

    while (!done) {
        src = grabFrame(sptr_kinect);
        bool pass = chessboard::overlay(src, dst, dChessboard, window);

        int key = cv::waitKey(30);
        switch (key) {
        case ENTER_KEY: // capture images
            chessboard::capture(pass, src, camera.m_images);
            break;
        case ESCAPE_KEY: // start calibration
            done = calibrateCamera(camera, dChessboard, file);
        default:
            break;
        }
    }
    return 0;
}
