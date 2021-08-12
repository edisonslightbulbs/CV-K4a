#include "scene.h"
#include <opencv2/opencv.hpp>

#if __linux__
#include "kinect.h"
cv::Mat grabFrame(std::shared_ptr<Kinect>& sptr_kinect)
{
    sptr_kinect->capture();
    sptr_kinect->depthCapture();
    sptr_kinect->pclCapture();
    sptr_kinect->imgCapture();
    sptr_kinect->c2dCapture();
    sptr_kinect->transform(RGB_TO_DEPTH);

    auto* rgbData = k4a_image_get_buffer(sptr_kinect->m_c2d);
    int w = k4a_image_get_width_pixels(sptr_kinect->m_c2d);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_c2d);

    cv::Mat frame
            = cv::Mat(h, w, CV_8UC4, (void*)rgbData, cv::Mat::AUTO_STEP).clone();

    sptr_kinect->releaseK4aCapture();
    sptr_kinect->releaseK4aImages();

    return frame;
}
#endif

void saturate(const cv::Mat& src, cv::Mat& dst)
{
    int beta = 100;     // brightness | range 1 - 100
    double alpha = 3.0; // contrast | range 1.0 - 3.0]

    dst = cv::Mat::zeros(src.size(), src.type());
    for (int y = 0; y < src.rows; y++) {
        for (int x = 0; x < src.cols; x++) {
            for (int c = 0; c < src.channels(); c++) {
                dst.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(
                        alpha * src.at<cv::Vec3b>(y, x)[c] + beta);
            }
        }
    }
}

int main()
{
    std::vector<cv::Mat> scene(2);

#if __linux__
    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);
#endif

#if __linux__
    scene::flicker(sptr_kinect, window, w, h, scene);
    scene::write(scene);
#endif

#if __APPLE__
    // load background scene images
    scene::load(scene);
#endif

    cv::Mat bgr[3];
    cv::Mat diff, contrast, thresholded;

    // extract area of projection | max flux AOE
    // cv::equalizeHist(bgr[0], contrast); // contrast
    diff = scene[0] - scene[1]; // diff
    saturate(diff, contrast);   // contrast
    cv::split(contrast, bgr);   // split
    cv::threshold(
            bgr[0], thresholded, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

    cv::Mat element, roi, final;
    element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3));
    cv::morphologyEx(thresholded, final, cv::MORPH_OPEN, element);

    // remove noise using smoothing
    cv::Mat blurred, other;
    cv::Size dBlur = cv::Size(35, 35);
    cv::GaussianBlur(final, blurred, dBlur, 0);

    // threshold to extract flux
    cv::threshold(
            blurred, other, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

    // crop region of interest
    cv::Rect roiBoundary = cv::boundingRect(other);
    roi = scene[0](roiBoundary);

    cv::imshow("1: image subtraction", diff);
    cv::imshow("2: image contrasting", contrast);
    cv::imshow("3: blue channel", bgr[0]);
    cv::imshow("4: thresholding", thresholded);
    cv::imshow("5: final", final);
    cv::imshow("6: final", other);
    cv::imshow("7: roi", roi);

    cv::waitKey();
    return 0;
}
