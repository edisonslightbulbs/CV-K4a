#include <opencv2/opencv.hpp>

#include "kinect.h"

void createGaussian(cv::Size& size, cv::Mat& output, int uX, int uY,
    float sigmaX, float sigmaY, float amplitude = 1.0f)
{
    cv::Mat temp = cv::Mat(size, CV_32F);
    for (int r = 0; r < size.height; r++) {
        for (int c = 0; c < size.width; c++) {
            float x = (float)((c - uX) * (c - uX)) / (2.0f * sigmaX * sigmaX);
            float y = (float)((c - uY) * (c - uY)) / (2.0f * sigmaY * sigmaY);
            float value = amplitude * exp(-(x + y));
            temp.at<float>(r, c) = value;
        }
        cv::normalize(temp, temp, 0.0f, 1.0f, cv::NORM_MINMAX);
        output = temp;
    }
}

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
    cv::Mat img = cv::Mat(rgbHeight, rgbWidth, CV_8UC4, (void*)color_image_data,
        cv::Mat::AUTO_STEP);

    /** write image */
    cv::imwrite(IMAGE, img);

    /** load grey-scale image */
    cv::Mat grayImage = cv::imread(IMAGE, cv::IMREAD_GRAYSCALE);
    // *img from kinect needs to be cast into a cv color image, i.e.,
    // into a 3 channel image first before using split. Not casting it
    // before using split will cause a seg fault.

    //  actual gaussian computation starts here:
    //
    cv::Mat output;
    cv::Size s = cv::Size(256, 256);

    createGaussian(s, output, 256 / 2, 256 / 2, 10, 10, 1.0f);
    cv::imshow("gaussian", output);

    cv::waitKey();
}
