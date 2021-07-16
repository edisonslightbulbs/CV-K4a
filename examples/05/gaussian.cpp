#include "kinect.h"
#include <opencv2/opencv.hpp>

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

    // load image in grey scale
    cv::Mat grayImage = cv::imread(IMAGE, cv::IMREAD_GRAYSCALE);

    //  do gaussian computation
    cv::Mat output;
    cv::Size s = cv::Size(256, 256);
    createGaussian(s, output, 256 / 2, 256 / 2, 10, 10, 1.0f);
    cv::imshow("gaussian", output);
    cv::waitKey();
}
