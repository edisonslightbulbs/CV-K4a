#include <opencv2/opencv.hpp>

#include "kinect.h"

void computeDft(cv::Mat& source, cv::Mat& destination){
    cv::Mat grayImageComplex[2] = { source, cv::Mat::zeros(source.size(), CV_32F) };

    cv::Mat dft;
    cv::Mat grayImageDft;

    cv::merge(grayImageComplex, 2, dft);
    cv::dft(dft, grayImageDft, cv::DFT_COMPLEX_OUTPUT);

    destination = grayImageDft;
}
// only applied for visualization and should not be
// applied during computation on the dft itself.
void recenterDft(cv::Mat& source){
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

void showDft(cv::Mat& source){
    cv::Mat splitChannel[2] = { cv::Mat::zeros(source.size(), CV_32F), cv::Mat::zeros(source.size(), CV_32F) };
    cv:split(source, splitChannel);

    cv::Mat dftMagnitude;
    cv::magnitude(splitChannel[0], splitChannel[1], dftMagnitude);
    dftMagnitude += cv::Scalar::all(1);

    cv::log(dftMagnitude, dftMagnitude);
    cv::normalize(dftMagnitude, dftMagnitude, 0, 1, cv::NORM_MINMAX);

    // recenterDft(dftMagnitude);
    cv::imshow("dft", dftMagnitude);
    // cv::waitKey();
}

//inverting back from the frequency domain
void invertDft(cv::Mat& source, cv::Mat& destination){
    cv::Mat inverse;
    cv::dft(source, inverse, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT | CV_HAL_DFT_SCALE);
    destination = inverse;
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
    cv::Mat img = cv::Mat(rgbHeight, rgbWidth, CV_8UC4, (void*)color_image_data, cv::Mat::AUTO_STEP);

    /** write image */
    cv::imwrite(IMAGE, img);

    /** load grey-scale image */
    cv::Mat grayImage = cv::imread(IMAGE, cv::IMREAD_GRAYSCALE);
    // *img from kinect needs to be cast into a cv color image, i.e.,
    // into a 3 channel image first before using split. Not casting it
    // before using split will cause a seg fault.

    //  actual dft computation starts here:
    //
    cv::Mat grayImageFloat;
    grayImage.convertTo(grayImageFloat, CV_32FC1, 1.0 / 255.0);

    cv::Mat grayImageDft;

    computeDft(grayImageFloat, grayImageDft);
    showDft(grayImageDft);

    cv::Mat invertedDft;
    invertDft(grayImageDft, invertedDft);
    cv::imshow("inverted dft", invertedDft);
    cv::waitKey();
}
