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

    /** load gray scale */
    cv::Mat gImgFile = cv::imread(IMAGE, cv::IMREAD_GRAYSCALE);
    cv::Mat modifiedGray = cv::imread(IMAGE, cv::IMREAD_GRAYSCALE);

    /** modifying gray-scale image */
    for(int r = 0; r < gImgFile.rows; r++){
        for(int c = 0; c < gImgFile.cols; c++ ){
            modifiedGray.at<uint8_t>(r, c) = modifiedGray.at<uint8_t>(r, c) * 0.5;
        }
    }

    /** load color image*/
    cv::Mat imgFile = cv::imread(IMAGE, cv::IMREAD_COLOR);
    cv::Mat modifiedColor = cv::imread(IMAGE, cv::IMREAD_COLOR);

    /** modifying color image */
    for(int r = 0; r < imgFile.rows; r++){
        for(int c = 0; c < imgFile.cols; c++ ){
            modifiedColor.at<cv::Vec3b>(r, c)[0] = modifiedColor.at<cv::Vec3b>(r, c)[0] * 0;
        }
    }


    /** show images */
    cv::imshow("gray", gImgFile);
    cv::imshow("modifiedGray", modifiedGray);

    cv::imshow("color", imgFile);
    cv::imshow("modifiedColor", modifiedColor);

    cv::waitKey();

    /** release resources */
    sptr_kinect->release();
}
