#include "kinect.h"
#include <opencv2/opencv.hpp>

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
    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // clone and convert to OpenCV Mat
    cv::Mat frame = grabFrame(sptr_kinect);

    // write image
    const std::string IMAGE = "./scene.png";
    cv::imwrite(IMAGE, frame);

    // The image from the kinect needs to be cast into a cv color image,
    // i.e., into a 3 channel image first before using split.
    // Not casting it before using split will cause a seg fault.
    //
    cv::Mat rgbImg = cv::imread(IMAGE, cv::IMREAD_COLOR);

    // split colors
    cv::Mat rgbChannel[3];
    cv::split(rgbImg, rgbChannel);

    // show split channels
    cv::imshow("rgb", frame);
    cv::imshow("blue", rgbChannel[0]);
    cv::imshow("green", rgbChannel[1]);
    cv::imshow("red", rgbChannel[2]);

    // zero the red channel
    rgbChannel[2] = cv::Mat::zeros(rgbChannel[2].size(), CV_8UC1);

    // merge the modified red channel to from a new output
    cv::Mat output;
    cv::merge(rgbChannel, 3, output);
    cv::imshow("modified", output);
    cv::waitKey();
}
