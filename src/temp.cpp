#include "scene.h"
#include <string>

int main()
{
    // icon resource
    const std::string iconPath = "./resources/icons/spotify.png";
    cv::Mat icon = cv::imread(iconPath, cv::IMREAD_UNCHANGED);
    cv::Mat rgba;

    cv::cvtColor(icon, rgba, cv::COLOR_BGR2BGRA);
    cv::Mat splitRgba[4];
    cv::split(rgba, splitRgba);


    // window dimensions
    const int h = 768;
    const int w = 1366;

    // black image
    cv::Mat background(h, w, CV_8UC3, cv::Scalar(0, 0, 0));

    // create full screen window and specify display location
    const std::string window = "Icon window";
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    cv::setWindowProperty(
        window, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    // icon resource
    cv::Mat foreground = splitRgba[3];

    // make sure images are in the same format
    cv::Mat backgroundGrey, foregroundGrey;
    cv::cvtColor(background, backgroundGrey, cv::COLOR_BGR2GRAY);
    //cv::cvtColor(icon, foregroundGrey, cv::COLOR_BGRA2GRAY);

    cv::imshow("test 1 ",  backgroundGrey);
    //cv::imshow("test 2 ",  foregroundGrey);
    cv::waitKey();

    // // starting (x, y) position of foreground image  in background image
    // int xMin = 0;
    // int yMin = 0;

    // // width and height of the foreground image
    // int width = foreground.cols;
    // int height = foreground.rows;

    // // define region of interest
    // cv::Rect roi = cv::Rect(xMin, yMin, width, height);

    // cv::Mat backgroundRoi = background(roi);
    // foreground.copyTo(backgroundRoi);

    // cv::imshow(window, foreground);
    // //cv::moveWindow(window, 3000, 0);
    // cv::waitKey();
}