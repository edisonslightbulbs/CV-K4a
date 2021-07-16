#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "chessboard.h"
#include "kinect.h"
#include "parameters.h"
#include "usage.h"

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
    sptr_kinect->capture();
    sptr_kinect->imgCapture();
    uint8_t* data = k4a_image_get_buffer(sptr_kinect->m_img);
    int w = k4a_image_get_width_pixels(sptr_kinect->m_img);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_img);
    sptr_kinect->releaseK4aCapture();
    sptr_kinect->releaseK4aImages();
    return cv::Mat(h, w, CV_8UC4, (void*)data, cv::Mat::AUTO_STEP).clone();
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
            cv::imshow("kinect", frameCopy);
        } else {
            cv::imshow("kinect", frame);
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
                    chessboard::CHESSBOARD_SQUARE_DIMENSION, cameraMatrix,
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
    return 0;
}
