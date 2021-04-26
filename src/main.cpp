#include <opencv2/opencv.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "kinect.h"

int main(int argc, char* argv[])
{
    /** initialize kinect */
    const std::string IMAGE = "./output/scene.png";
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);
    int rgbWidth = k4a_image_get_width_pixels(sptr_kinect->m_rgbImage);
    int rgbHeight = k4a_image_get_height_pixels(sptr_kinect->m_rgbImage);

    /** get image from kinect */
    uint8_t* color_image_data = k4a_image_get_buffer(sptr_kinect->m_rgbImage);

    /** release resources */
    sptr_kinect->release();

    /** cast to cv::Mat */
    cv::Mat img = cv::Mat(rgbHeight, rgbWidth, CV_8UC4, (void*)color_image_data, cv::Mat::AUTO_STEP);

    /** write image */
    cv::imwrite(IMAGE, img);

    /** load color image */
    cv::Mat colorImage = cv::imread(IMAGE, cv::IMREAD_COLOR);
    // *img from kinect needs to be cast into a cv color image, i.e.,
    // into a 3 channel image first before using split. Not casting it
    // before using split will cause a seg fault.

    /** split R G B channels */
    cv::Mat rgbChannel[3];
    cv::split(colorImage, rgbChannel);

    cv::imshow("color", img);
    cv::imshow("blue", rgbChannel[0]);
    cv::imshow("green", rgbChannel[1]);
    cv::imshow("red", rgbChannel[2]);
    cv::waitKey();
}
