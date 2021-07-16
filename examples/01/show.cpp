#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "kinect.h"

cv::Mat grabFrame(std::shared_ptr<Kinect>& sptr_kinect)
{
    sptr_kinect->capture();
    sptr_kinect->imgCapture();
    uint8_t* data = k4a_image_get_buffer(sptr_kinect->m_img);
    int w = k4a_image_get_width_pixels(sptr_kinect->m_img);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_img);
    sptr_kinect->releaseK4aCapture();
    sptr_kinect->releaseK4aImages();
    return cv::Mat(h, w, CV_8UC4, (void*)data, cv::Mat::AUTO_STEP);
}

int main()
{
    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    cv::Mat img = grabFrame(sptr_kinect);

    // show images and wait for keypress
    cv::imshow("", img);
    cv::waitKey();
}
