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

    // write image
    const std::string IMAGE = "./scene.png";
    cv::imwrite(IMAGE, img);

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
