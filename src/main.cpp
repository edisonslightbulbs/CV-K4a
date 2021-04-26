#include <opencv2/opencv.hpp>

#include "kinect.h"

int main(int argc, char* argv[])
{
    /** initialize kinect */
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    const std::string IMAGE = "./output/scene.png";
    int rgbWidth = k4a_image_get_width_pixels(sptr_kinect->m_rgbImage);
    int rgbHeight = k4a_image_get_height_pixels(sptr_kinect->m_rgbImage);

    /** get image from kinect */
    uint8_t* color_image_data = k4a_image_get_buffer(sptr_kinect->m_rgbImage);

    /** cast to cv::Mat */
    cv::Mat img = cv::Mat(rgbHeight, rgbWidth, CV_8UC4, (void*)color_image_data, cv::Mat::AUTO_STEP);

    /** write image */
    cv::imwrite(IMAGE, img);

    /** load file */
    cv::Mat imgFile = cv::imread(IMAGE, cv::IMREAD_COLOR);
    cv::Mat gImgFile = cv::imread(IMAGE, cv::IMREAD_GRAYSCALE);

    /** show image */
    cv::imshow("kinectC", imgFile);
    cv::imshow("kinectG", gImgFile);
    cv::waitKey();

    /** release resources */
    sptr_kinect->release();
    sptr_kinect->close();
}
