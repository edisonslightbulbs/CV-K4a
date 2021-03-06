#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "kinect.h"

cv::Mat grabFrame(std::shared_ptr<Kinect>& sptr_kinect)
{
    sptr_kinect->capture();
    sptr_kinect->imgCapture();
    uint8_t* rgbData = k4a_image_get_buffer(sptr_kinect->m_img);
    int w = k4a_image_get_width_pixels(sptr_kinect->m_img);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_img);

    cv::Mat frame
        = cv::Mat(h, w, CV_8UC4, (void*)rgbData, cv::Mat::AUTO_STEP).clone();

    sptr_kinect->releaseK4aCapture();
    sptr_kinect->releaseK4aImages();

    return frame;
}

int main()
{
    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    while (true) {
        cv::Mat frame = grabFrame(sptr_kinect);

        cv::imshow("kinect", frame);
        if (cv::waitKey(1000 / 20) >= 0) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(2));
    }
    return 0;
}
