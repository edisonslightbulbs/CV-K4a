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

    cv::Mat frame = grabFrame(sptr_kinect);

    // write image
    const std::string IMAGE = "./scene.png";
    cv::imwrite(IMAGE, frame);

    // load image (read gray scale component)
    cv::Mat greyImg = cv::imread(IMAGE, cv::IMREAD_GRAYSCALE);
    cv::Mat greyImgMod = cv::imread(IMAGE, cv::IMREAD_GRAYSCALE);

    // modifying: reduce brightness
    for (int r = 0; r < greyImg.rows; r++) {
        for (int c = 0; c < greyImg.cols; c++) {
            greyImgMod.at<uint8_t>(r, c)
                = (unsigned char)(greyImgMod.at<uint8_t>(r, c) * 0.5);
        }
    }

    // load image (read RGB component)
    cv::Mat rgbImg = cv::imread(IMAGE, cv::IMREAD_COLOR);
    cv::Mat rgbImgMod = cv::imread(IMAGE, cv::IMREAD_COLOR);

    // modify: remove red channel
    for (int r = 0; r < rgbImg.rows; r++) {
        for (int c = 0; c < rgbImg.cols; c++) {
            rgbImgMod.at<cv::Vec3b>(r, c)[0]
                = rgbImgMod.at<cv::Vec3b>(r, c)[0] * 0;
        }
    }

    // show images and wait for keypress
    cv::imshow("Grey", greyImg);
    cv::imshow("Modified Grey", greyImgMod);
    cv::imshow("RGB", rgbImg);
    cv::imshow("Modified RGB", rgbImgMod);
    cv::waitKey();
}
