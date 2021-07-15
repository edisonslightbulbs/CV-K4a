#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "chessboard.h"
#include "kinect.h"
#include "parameters.h"

#define CHESSBOARD_IMAGES 0
#define COMPUTING_CALIBRATION_PARAMETERS 1
#define MORE_CHESSBOARD_IMAGES_REQUIRED 2
#define WRITING_CALIBRATION_PARAMETERS 3
#define READING_CALIBRATION_PARAMETERS 4
#define FINDING_ARUCO_MARKERS 5

#define ENTER_KEY 13
#define ESCAPE_KEY 27

void usage(const int& code)
{
    switch (code) {
    case (0):
        std::cout << "INSTRUCTIONS:" << std::endl;
        std::cout << "-- press ENTER to take images" << std::endl;
        std::cout << "-- to at least 20 chessboard images" << std::endl;
        std::cout << "-- press ESCAPE to exit capture mode" << std::endl;
        break;
    case (1):
        std::cout << "-- computing calibration parameters" << std::endl;
        break;
    case (2):
        std::cout << "-- take more chessboard images" << std::endl;
        break;
    case (3):
        std::cout << "-- saving calibration parameters to disk" << std::endl;
        break;
    case (4):
        std::cout << "-- loading calibration parameters from disk" << std::endl;
        break;
    case (5):
        std::cout << "-- searching for aruco markers" << std::endl;
        break;
    default:
        std::cout << "-- well now, wasn't expecting that one bit" << std::endl;
        break;
    }
}

void calibrate(std::vector<cv::Mat> targetImgs, const cv::Size& boardSize,
    float squareEdgeLength, cv::Mat& cameraMatrix, cv::Mat& coefficients)
{
    std::vector<cv::Mat> rVectors, tVectors;
    std::vector<std::vector<cv::Point2f>> imagespaceSquareEdges;
    std::vector<std::vector<cv::Point3f>> worldSpaceSquareCorners(1);

    chessboard::findSquareCorners(targetImgs, imagespaceSquareEdges, false);
    chessboard::findSquareEdges(
        boardSize, squareEdgeLength, worldSpaceSquareCorners[0]);

    worldSpaceSquareCorners.resize(
        imagespaceSquareEdges.size(), worldSpaceSquareCorners[0]);
    coefficients = cv::Mat::zeros(8, 1, CV_64F);

    cv::calibrateCamera(worldSpaceSquareCorners, imagespaceSquareEdges,
        boardSize, cameraMatrix, coefficients, rVectors, tVectors);
}

cv::Mat grabFrame(std::shared_ptr<Kinect>& sptr_kinect)
{
    sptr_kinect->getFrame(RGB_TO_DEPTH);
    uint8_t* data = k4a_image_get_buffer(sptr_kinect->m_rgbImage);
    int w = k4a_image_get_width_pixels(sptr_kinect->m_rgbImage);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_rgbImage);
    sptr_kinect->release();
    return cv::Mat(h, w, CV_8UC4, (void*)data, cv::Mat::AUTO_STEP);
}

int main()
{
    // initialize kinect and get image dimensions
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // setup camera matrix and initialize coefficients
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat coefficients;

    // initialize named window and frames for superimposing
    cv::namedWindow("kinect", cv::WINDOW_AUTOSIZE);
    cv::Mat frame, frameCopy;

    // specify chessboard dimensions
    const cv::Size chessboardDim = cv::Size(9, 6);

    // prompt user with usage caveat
    usage(CHESSBOARD_IMAGES);

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
            cv::imshow("kinect", frameCopy);
        } else {
            cv::imshow("kinect", frame);
        }

        int key = cv::waitKey(100);

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
                usage(COMPUTING_CALIBRATION_PARAMETERS);
                calibrate(chessboardImgs, chessboardDim,
                    chessboard::calibrationSquareDimension, cameraMatrix,
                    coefficients);
                usage(WRITING_CALIBRATION_PARAMETERS);
                parameters::write(
                    "calibration.txt", cameraMatrix, coefficients);
                std::this_thread::sleep_for(std::chrono::seconds(5));
                done = true;
            } else {
                usage(MORE_CHESSBOARD_IMAGES_REQUIRED);
            }
        default:
            break;
        }
    }
    return 0;
}
