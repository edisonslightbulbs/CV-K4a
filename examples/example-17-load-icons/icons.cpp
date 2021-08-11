#include <string>

#include <opencv2/opencv.hpp>
#include "icon.h"

int main()
{
    // create full screen window
    const std::string window = "scene";
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    cv::setWindowProperty(window, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    // create background image
    const int h = 768;
    const int w = 1366;
    cv::Mat background(h, w, CV_8UC3, cv::Scalar(0, 0, 0));

    // create foreground
    cv::Mat foreground_1 = icon::load("./resources/icons/spotify.png");
    cv::Mat foreground_2 = icon::load("./resources/icons/discord.png");
    cv::Mat foreground_3 = icon::load("./resources/icons/facebook.png");

    // saturate foreground image
    int beta = 0;     // brightness | range 1 - 100
    double alpha = 3.0; //   contrast | range 1.0 - 3.0
    icon::saturate(foreground_1, beta, alpha);
    icon::saturate(foreground_2, beta, alpha);
    icon::saturate(foreground_3, beta, alpha);

    // scale foreground image to some width and height
    int scaleWidth = 60;
    int scaleHeight = 60;
    icon::scale(foreground_1, scaleWidth, scaleHeight);
    icon::scale(foreground_2, scaleWidth, scaleHeight);
    icon::scale(foreground_3, scaleWidth, scaleHeight);

    // initialize starting position for drawing
    // foreground image on background image
    int xMin = background.cols/2;
    int yMin = background.rows/2;

    // get width and height of the foreground image
    int width = foreground_1.cols;
    int height = foreground_1.rows;

    // create roi using starting position and size of foreground
    cv::Rect roi_1 = cv::Rect(xMin, yMin, width, height);
    cv::Rect roi_2 = cv::Rect(xMin + (xMin/2), yMin + (yMin/2), width, height);
    cv::Rect roi_3 = cv::Rect(xMin - (xMin/2), yMin - (yMin/2), width, height);

    // get roi from background image
    cv::Mat backgroundRoi_1 = background(roi_1);
    cv::Mat backgroundRoi_2 = background(roi_2);
    cv::Mat backgroundRoi_3 = background(roi_3);

    // overlay foreground on background @ roi
    foreground_1.copyTo(backgroundRoi_1);
    foreground_2.copyTo(backgroundRoi_2);
    foreground_3.copyTo(backgroundRoi_3);

    // show window at desired location
    cv::imshow(window, background);
    cv::moveWindow(window, 3000, 0);
    cv::waitKey();
}
