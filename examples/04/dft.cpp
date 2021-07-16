#include <opencv2/opencv.hpp>
#include "kinect.h"

void computeDft(cv::Mat& source, cv::Mat& destination)
{
    cv::Mat grayImageComplex[2]
            = { source, cv::Mat::zeros(source.size(), CV_32F) };

    cv::Mat dft;
    cv::Mat grayImageDft;

    cv::merge(grayImageComplex, 2, dft);
    cv::dft(dft, grayImageDft, cv::DFT_COMPLEX_OUTPUT);

    destination = grayImageDft;
}

// only applied for visualization purposes
//
void recenterDft(cv::Mat& source)
{
    int centerX = source.cols / 2;
    int centerY = source.rows / 2;

    cv::Mat q1(source, cv::Rect(0, 0, centerX, centerY));
    cv::Mat q2(source, cv::Rect(centerX, 0, centerX, centerY));
    cv::Mat q3(source, cv::Rect(0, centerY, centerX, centerY));
    cv::Mat q4(source, cv::Rect(centerX, centerY, centerX, centerY));

    cv::Mat swapMap;

    q1.copyTo(swapMap);
    q4.copyTo(q1);
    swapMap.copyTo(q4);

    q2.copyTo(swapMap);
    q3.copyTo(q2);
    swapMap.copyTo(q3);
}

void showDft(cv::Mat& source)
{
    cv::Mat splitChannel[2] = { cv::Mat::zeros(source.size(), CV_32F),
                                cv::Mat::zeros(source.size(), CV_32F) };

    cv::split(source, splitChannel);

    cv::Mat dftMagnitude;
    cv::magnitude(splitChannel[0], splitChannel[1], dftMagnitude);
    dftMagnitude += cv::Scalar::all(1);

    cv::log(dftMagnitude, dftMagnitude);
    cv::normalize(dftMagnitude, dftMagnitude, 0, 1, cv::NORM_MINMAX);

#define VISUALIZE 1
// recenter dft is for cosmetics and should
// only be used for visualization
//
#if VISUALIZE == 1
    recenterDft(dftMagnitude);
#endif

    // output
    cv::imshow("dft", dftMagnitude);
    cv::waitKey();
}

// inverting back from the frequency domain
void invertDft(cv::Mat& source, cv::Mat& destination)
{
    cv::Mat inverse;
    cv::dft(source, inverse,
            cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT | CV_HAL_DFT_SCALE);
    destination = inverse;
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

    cv::Mat img = grabFrame(sptr_kinect);

    // write image
    const std::string IMAGE = "./scene.png";
    cv::imwrite(IMAGE, img);

    // load grey-scale image
    cv::Mat greyImg = cv::imread(IMAGE, cv::IMREAD_GRAYSCALE);

    // do dft
    cv::Mat grayImageFloat;
    greyImg.convertTo(grayImageFloat, CV_32FC1, 1.0 / 255.0);

    cv::Mat imgDft;
    computeDft(grayImageFloat, imgDft);
    showDft(imgDft);

    cv::Mat invertedDft;
    invertDft(imgDft, invertedDft);
    cv::imshow("inverted dft", invertedDft);
    cv::waitKey();
}
