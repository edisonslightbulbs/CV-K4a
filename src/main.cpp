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
#include "parameters.h"
#include "usage.h"
#include "chessboard.h"

#define WIDTH 1366
#define HEIGHT 768
#define CHESSBOARD_WINDOW "TRACELESS"
#define CALIBRATION_WINDOW "CALIBRATING"
#define SETUP_CHESSBOARD_WINDOW cv::moveWindow(CHESSBOARD_WINDOW, 3000, 0)
#define SETUP_CALIBRATION_WINDOW cv::moveWindow(CHESSBOARD_WINDOW, 3000, 0)

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

cv::Mat contrastReference(const bool& contrast)
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

void captureReference(std::vector<cv::Mat>& images, std::shared_ptr<Kinect>& sptr_kinect){
    // background contrast flag
    bool contrast = false;

    while (true) {
        cv::Mat img = contrastReference(contrast);
        cv::imshow(CHESSBOARD_WINDOW, img);
        SETUP_CHESSBOARD_WINDOW;

        // toggle contrast flag every second
        if (cv::waitKey(1000) >= 0) {
            break;
        }
        contrast = !contrast;

        // grab current contrast
        cv::Mat frame = grabFrame(sptr_kinect);
        images.emplace_back(frame);
        if (images.size() == 2) {
            break;
        }
    }
}

void calibrate(std::vector<cv::Mat> images, const cv::Size& boardSize,
               float blockLength, cv::Mat& cameraMatrix, cv::Mat& coefficients)
{
    std::vector<cv::Mat> rVectors, tVectors;
    std::vector<std::vector<cv::Point2f>> imageSpaceCorners;
    std::vector<std::vector<cv::Point3f>> worldSpaceSquareCorners(1);

    chessboard::findImageSpaceCorners(images, imageSpaceCorners, false);
    chessboard::findWorldSpaceCorners(
            boardSize, blockLength, worldSpaceSquareCorners[0]);

    worldSpaceSquareCorners.resize(
            imageSpaceCorners.size(), worldSpaceSquareCorners[0]);
    coefficients = cv::Mat::zeros(8, 1, CV_64F);

    cv::calibrateCamera(worldSpaceSquareCorners, imageSpaceCorners, boardSize,
                        cameraMatrix, coefficients, rVectors, tVectors);
}

void calibrateProjector(std::shared_ptr<Kinect>& sptr_kinect){
    // setup camera matrix and initialize coefficients
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat coefficients;

    // initialize named window and frames for superimposing
    // create full screen window
    cv::namedWindow(CALIBRATION_WINDOW, cv::WINDOW_NORMAL);
    cv::setWindowProperty(CALIBRATION_WINDOW, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::Mat frame, frameCopy;

    // specify chessboard dimensions
    const cv::Size chessboardDim = cv::Size(9, 6);

    // prompt user with usage caveat
    usage::prompt(CHESSBOARD_IMAGES);

    // prompt user with usage caveat
    std::vector<cv::Mat> chessboardImgs;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    bool done = false;
    bool chessboardCornersFound;

    while (!done) {
        // grab frame from kinect
        frame = grabFrame(sptr_kinect);

        // find chessboard corners
        std::vector<cv::Point2f> chessboardCorners;
        chessboardCornersFound
                = cv::findChessboardCorners(frame, chessboardDim, chessboardCorners,
                                            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        // clone frame then draw on and superimpose clone
        frame.copyTo(frameCopy);
        cv::drawChessboardCorners(frameCopy, chessboardDim, chessboardCorners,
                                  chessboardCornersFound);

        if (chessboardCornersFound) {
            cv::imshow(CALIBRATION_WINDOW, frameCopy);
            SETUP_CALIBRATION_WINDOW;
        } else {
            cv::imshow(CALIBRATION_WINDOW, frame);
            SETUP_CALIBRATION_WINDOW;
        }

        int key = cv::waitKey(10);
        switch (key) {
            case ENTER_KEY:
                if (chessboardCornersFound) {
                    cv::Mat temp;
                    frame.copyTo(temp);
                    chessboardImgs.emplace_back(temp);
                    std::cout << "-- # images : " << chessboardImgs.size()
                              << std::endl;
                }
                break;

            case ESCAPE_KEY:
                if (chessboardImgs.size() > 15) {
                    usage::prompt(COMPUTING_CALIBRATION_PARAMETERS);
                    calibrate(chessboardImgs, chessboardDim,
                              chessboard::PROJECTED_BOARD_BLOCK_LENGTH, cameraMatrix,
                              coefficients);
                    usage::prompt(WRITING_CALIBRATION_PARAMETERS);
                    parameters::write(
                            "calibration.txt", cameraMatrix, coefficients);
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    done = true;
                } else {
                    usage::prompt(MORE_CHESSBOARD_IMAGES_REQUIRED);
                }
            default:
                break;
        }
    }
}

void projectChessboard(const cv::Mat& chessboard){
    while (true) {
        cv::imshow(CHESSBOARD_WINDOW, chessboard);
        SETUP_CHESSBOARD_WINDOW;

        // toggle contrast flag every second
        if (cv::waitKey(1000) >= 0) {
            break;
        }
    }
}

int main()
{
    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // create full screen window
    cv::namedWindow(CHESSBOARD_WINDOW, cv::WINDOW_NORMAL);
    cv::setWindowProperty(CHESSBOARD_WINDOW, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    // generate chessboard
    std::vector<cv::Point2f> corners;
    cv::Size imgSize = cv::Size(800, 600);
    cv::Size boardSize = cv::Size(9, 6);
    cv::Mat chessboard = chessboard::generate(imgSize, boardSize, corners);
    std::thread chessboardWorker(projectChessboard, chessboard);

    // tabletop images
    std::vector<cv::Mat> kinectImages;

    // todo implement: reference scene function
    //  ??

    // todo implement: calibrate function
    calibrateProjector(sptr_kinect);

    chessboardWorker.join();
    return 0;
}

