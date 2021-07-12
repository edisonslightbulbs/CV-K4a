#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "kinect.h"

int main()
{
    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // get k4a image and dims
    uint8_t* color_image_data = k4a_image_get_buffer(sptr_kinect->m_rgbImage);
    int rgbWidth = k4a_image_get_width_pixels(sptr_kinect->m_rgbImage);
    int rgbHeight = k4a_image_get_height_pixels(sptr_kinect->m_rgbImage);

    // clone and convert to OpenCV Mat
    cv::Mat img = cv::Mat(rgbHeight, rgbWidth, CV_8UC4, (void*)color_image_data,
                          cv::Mat::AUTO_STEP).clone();

    // release k4a resources
    sptr_kinect->release();

    // show images and wait for keypress
    cv::imshow("", img);
    cv::waitKey();
}
