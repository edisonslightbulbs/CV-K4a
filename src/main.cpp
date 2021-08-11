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
    cv::Mat foreground = icon::load("./resources/icons/spotify.png");
    /// scale it ?
    /// rotate it ?

    // initialize starting position for drawing
    // foreground image on background image
    int xMin = 0;
    int yMin = 0;

    // get width and height of the foreground image
    int width = foreground.cols;
    int height = foreground.rows;

    // create roi using starting position and size of foreground
    cv::Rect roi = cv::Rect(xMin, yMin, width, height);

    // get roi from background image
    cv::Mat backgroundRoi = background(roi);

    // overlay foreground on background @ roi
    foreground.copyTo(backgroundRoi);

    // show window at desired location
    cv::imshow(window, background);
    //cv::moveWindow(window, 3000, 0);
    cv::waitKey();
}
