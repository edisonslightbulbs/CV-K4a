#include "scene.h"
#include <opencv2/opencv.hpp>

#include "kinect.h"

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
    std::vector<cv::Mat> scene;

    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    int w = 1366;
    int h = 768;
    const std::string window = "setup window";
    scene::flicker(sptr_kinect, window, w, h, scene);

    // image subtraction
    cv::Mat colorDiff, gray_0, gray_1, grayDiff, sharpGray;
    cv::cvtColor(scene[1], gray_0, cv::COLOR_BGR2GRAY);
    cv::cvtColor(scene[0], gray_1, cv::COLOR_BGR2GRAY);

    // cv::absdiff(scene[1], scene[0], colorDiff);
    // cv::absdiff(gray_1, gray_0, grayDiff);
    colorDiff = scene[1] - scene[0];
    grayDiff = gray_0 - gray_1;

    // sharpen gray diff
    // cv::equalizeHist(grayDiff, sharpGray); // <- iffy output
    cv::Mat saturated;
    saturate(colorDiff, saturated);
    cv::cvtColor(saturated, sharpGray, cv::COLOR_BGR2GRAY);

    // remove noise using smoothing
    cv::Mat blurred, thresholded, roi;
    cv::Size dBlur = cv::Size(33, 33);
    cv::GaussianBlur(sharpGray, blurred, dBlur, 0);

    // threshold to extract flux
    cv::threshold(
        blurred, thresholded, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

    // crop region of interest
    cv::Rect roiBoundary = cv::boundingRect(thresholded);
    roi = scene[0](roiBoundary);

    // show results (flux AOE)
    cv::imshow("Color difference", colorDiff);
    cv::imwrite("./output/colordiff.jpg", colorDiff);

    // cv::imshow("Gray difference", grayDiff);
    // cv::imwrite("./output/graydiff.png", grayDiff);

    cv::imshow("Saturated", saturated);
    cv::imwrite("./output/saturated.png", saturated);

    cv::imshow("Sharpened gray diff", sharpGray);
    cv::imwrite("./output/sharpgray.png", sharpGray);

    cv::imshow("Blurred gray diff", blurred);
    cv::imwrite("./output/blurred.png", blurred);

    cv::imshow("OTSU threshold", thresholded);
    cv::imwrite("./output/thresholded.png", thresholded);

    cv::imshow("Region of interest", roi);
    cv::imwrite("./output/roi.png", roi);

    cv::waitKey();
    return 0;
}
