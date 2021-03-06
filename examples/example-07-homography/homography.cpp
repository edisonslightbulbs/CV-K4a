#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

cv::Mat background;                     // background image
cv::Mat foreground;                     // foreground image
std::vector<cv::Point2f> backgroundRoi; // background region (4 corners)
std::vector<cv::Point2f> foregroundRoi; // projection region (4 corners)

void overlay(cv::Mat& src, cv::Mat& dst)
{
    cv::Mat gray, grayCopy, grayInv, grayInvCopy;
    cv::cvtColor(dst, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, gray, 0, 255, cv::THRESH_BINARY);
    cv::bitwise_not(gray, grayInv);

    dst.copyTo(grayCopy, gray);
    src.copyTo(grayInvCopy, grayInv);

    cv::Mat homography = grayInvCopy + grayCopy;
    cv::imshow("homography", homography);
    cv::waitKey(0);
}

void showXY(const int& corners, const int& x, const int& y)
{
    switch (corners) {
        case 1:
            std::cout << "    top left corner: ";
            break;
        case 2:
            std::cout << " bottom left corner: ";
            break;
        case 3:
            std::cout << "bottom right corner: ";
            break;
        case 4:
            std::cout << "   top right corner: ";
            break;
        default:
            break;
    }
    std::cout << x << ", " << y << std::endl;
}

void callback(int e, int x, int y, int d, void* ptr)
{
    cv::Mat warpedForeground;

    if (e == cv::EVENT_LBUTTONDOWN) {
        if (foregroundRoi.size() <= 4) {
            foregroundRoi.emplace_back(float(x), float(y));
            showXY((int)foregroundRoi.size(), x, y);
        }
        if (foregroundRoi.size() == 4) {
            std::cout << "-- computing homography " << std::endl;
            cv::Mat homography = cv::findHomography(backgroundRoi, foregroundRoi, 0);
            cv::warpPerspective(foreground, warpedForeground, homography, background.size());

            foregroundRoi.clear();
            overlay(background, warpedForeground);
            //cv::setMouseCallback("homography", nullptr, nullptr);
            cv::setMouseCallback("homography", callback, nullptr);
        }
    }
}

int main()
{
    background = imread("./main.jpg", cv::IMREAD_COLOR);
    foreground = imread("./logo.jpg", cv::IMREAD_COLOR);

    // initialize correspondence between background and foreground images
    backgroundRoi.emplace_back(float(0), float(0));
    backgroundRoi.emplace_back(float(0), float(foreground.rows));
    backgroundRoi.emplace_back(float(foreground.cols), float(foreground.rows));
    backgroundRoi.emplace_back(float(foreground.cols), float(0));

    cv::namedWindow("homography", cv::WINDOW_AUTOSIZE); // Create a window for display.
    cv::imshow("homography", background);

    std::cout << "-- click on four points to overlay computed homography" << std::endl;
    std::cout << "-- n.b., start from the top-left corner and proceed anti-clockwise " << std::endl;
    cv::setMouseCallback("homography", callback, nullptr);

    while (true) {
        int key = cv::waitKey(10);
        if (key == 27)
            break;
    }

    return 0;
}
