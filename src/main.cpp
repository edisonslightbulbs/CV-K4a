// #include <iostream>
//
// int main()
// {
//     std::cout << "-- see in examples directory for the code " << std::endl;
//     std::cout << "-- see in build/bin directory for the binaries " <<
//     std::endl;
// }

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "kinect.h"

#define WIDTH 1366
#define HEIGHT 768
#define TO_PROJECTOR_DISPLAY cv::moveWindow(WINDOW, 3000, 0)

cv::Mat getChessboard(const cv::Size& imgSize, const cv::Size& boardSize,
    std::vector<cv::Point2f>& corners)
{
    int offset
        = 50; // opencv requires white boarders around checkerboard pattern

    // checkerboard image
    cv::Mat imgCheckerboard(imgSize, CV_8UC3, cv::Scalar::all(255));

    // block size
    int squareWidth = floor((imgSize.width - 2 * offset) / boardSize.width);
    int squareHeight = floor((imgSize.height - 2 * offset) / boardSize.height);

    // block color
    unsigned char color = 1;

    //! The order must be consistent with OpenCV order:
    // row first then column, each row sweep from left to right
    for (int y = offset; y < imgSize.height - offset; y = y + squareHeight) {
        color = ~color; // bitwise complement
        if (y + squareHeight > imgSize.height - offset) {
            break;
        }
        for (int x = offset; x < imgSize.width - offset; x = x + squareWidth) {
            color = ~color;
            if (x + squareWidth > imgSize.width - offset) {
                break;
            }
            // save checkerboard points
            if (x > offset && y > offset) {
                corners.emplace_back(x, y);
            }
            // color the block
            cv::Mat block
                = imgCheckerboard(cv::Rect(x, y, squareWidth, squareHeight));
            block.setTo(cv::Scalar::all(color));
        }
    }
    return imgCheckerboard;
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

cv::Mat contrastBackground(const bool& contrast)
{
    // create black and white images
    cv::Mat black(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat white(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));

    if (contrast) {
        return white;
    } else {
        return black;
    }
}

int main()
{
    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // create full screen window
    const std::string WINDOW = "TRACELESS";
    cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);
    cv::setWindowProperty(
        WINDOW, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    // background and background + projection
    std::vector<cv::Mat> background;

    // create chessboard

    std::vector<cv::Point2f> corners;
    cv::Size imgSize = cv::Size(800, 600);

    const int CORNERS_X_DIRECTION = 6;
    const int CORNERS_Y_DIRECTION = 9;
    cv::Size boardSize = cv::Size(9, 6);
    // todo chessboard function is buggy: gives 1 less
    //  corner for the x and y directions

    cv::Mat chessboard = getChessboard(imgSize, boardSize, corners);

    // background contrast flag
    bool contrast = false;

    while (true) {
        cv::Mat img = contrastBackground(contrast);
        cv::imshow(WINDOW, img);
        TO_PROJECTOR_DISPLAY;

        // toggle contrast flag every second
        if (cv::waitKey(1000) >= 0) {
            break;
        }
        contrast = !contrast;

        // grab current contrast
        cv::Mat frame = grabFrame(sptr_kinect);
        background.emplace_back(frame);
        if (background.size() == 2) {
            break;
        }
    }

    while (true) {
        cv::imshow(WINDOW, chessboard);
        TO_PROJECTOR_DISPLAY;

        // toggle contrast flag every second
        if (cv::waitKey(1000) >= 0) {
            break;
        }
    }

    // background subtraction
    // find projection boundary
    // save boundary
    // use boundary with kinect images
    // start calibration

    return 0;
}
