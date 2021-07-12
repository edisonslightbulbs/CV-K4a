// #include <iostream>
//
// int main(){
//     std::cout << "-- have a look the examples directory for code examples! " << std::endl;
//     std::cout << "-- have a look the build/bin directory for the binaries! " << std::endl;
// }
#include <chrono>
#include <thread>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "kinect.h"
#include "aruco.h"
#include "chessboard.h"
#include "parameters.h"

void calibrate(std::vector<cv::Mat> calibrationImages,
               const cv::Size& boardSize, float squareEdgeLength, cv::Mat& cameraMatrix,
               cv::Mat& distanceCoefficients)
{
    std::vector<std::vector<cv::Point2f>> checkerboardImageSpacePoints;

    chessboard::findCorners(
            calibrationImages, checkerboardImageSpacePoints, false);

    std::vector<std::vector<cv::Point3f>> worldSpaceCornersPoints(1);

    chessboard::findEdges(
            boardSize, squareEdgeLength, worldSpaceCornersPoints[0]);

    worldSpaceCornersPoints.resize(
            checkerboardImageSpacePoints.size(), worldSpaceCornersPoints[0]);

    std::vector<cv::Mat> rVectors, tVectors;
    distanceCoefficients = cv::Mat::zeros(8, 1, CV_64F);

    cv::calibrateCamera(worldSpaceCornersPoints, checkerboardImageSpacePoints,
                        boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
}

int main()
{
    cv::Mat distanceCoefficients;
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    const cv::Size chessboardDimensions = cv::Size(9, 6);

    cv::Mat frame;
    cv::Mat drawToFrame;

    std::vector<cv::Mat> savedImages;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    /** initialize kinect */
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);
    int rgbWidth = k4a_image_get_width_pixels(sptr_kinect->m_rgbImage);
    int rgbHeight = k4a_image_get_height_pixels(sptr_kinect->m_rgbImage);

    /** defined frames per second */
    const int fps = 20;

    /** create named window */
    cv::namedWindow("kinect", cv::WINDOW_AUTOSIZE);

    bool done = false;
    std::cout << "INSTRUCTIONS:" << std::endl;
    std::cout << "-- press ENTER to take images" << std::endl;
    std::cout << "-- be sure to take a minimum of 20 calibration  images" << std::endl;
    std::cout << "-- press ESCAPE to exit calibration image capture mode" << std::endl;

    while (!done) {
        /** get next frame from kinect */
        sptr_kinect->getFrame(RGB_TO_DEPTH);

        /** get image from kinect */
        uint8_t* color_image_data
                = k4a_image_get_buffer(sptr_kinect->m_rgbImage);

        /** release resources */
        sptr_kinect->release();

        /** cast to cv::Mat */
        frame = cv::Mat(rgbHeight, rgbWidth, CV_8UC4, (void*)color_image_data,
                        cv::Mat::AUTO_STEP);

        /** find corners */
        std::vector<cv::Point2f> foundPoints;
        bool found;
        found = cv::findChessboardCorners(frame, chessboardDimensions,
                                          foundPoints,
                                          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        frame.copyTo(drawToFrame);

        /** if found,  draw them */
        cv::drawChessboardCorners(
                drawToFrame, chessboardDimensions, foundPoints, found);

        if (found) {
            cv::imshow("kinect", drawToFrame);
        } else {
            cv::imshow("kinect", frame);
        }
        int key = cv::waitKey(1000 / fps);

        switch (key) {
            case 13:
                /** save image */
                if (found) {
                    cv::Mat temp;
                    frame.copyTo(temp);
                    savedImages.emplace_back(temp);
                    std::cout << "-- current number of images: " << savedImages.size()
                              << std::endl;
                }
                break;

            case 27:
                /** calibrate */
                if (savedImages.size() > 15) {
                    std::cout << "-- computing intrinsic parameters" << std::endl;
                    calibrate(savedImages, chessboardDimensions,
                              chessboard::calibrationSquareDimension, cameraMatrix,
                              distanceCoefficients);
                    parameters::write(
                            "calibration.txt", cameraMatrix, distanceCoefficients);
                    done = true;
                } else {
                    std::cout << "-- take more images from different angles for better results" << std::endl;
                }
            default:
                break;
        }
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));
    parameters::read("calibration.txt", cameraMatrix, distanceCoefficients);
    aruco::find(cameraMatrix, distanceCoefficients, aruco::arucoSquareDimension);

    return 0;
}
