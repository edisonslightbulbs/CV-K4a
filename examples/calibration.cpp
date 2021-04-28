#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include "kinect.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
const float calibrationSquareDimension = 0.02500f; // meters
const float arucoSquareDimension = 0.0565f;        // meters

void createArucoMarkers()
{
    cv::Mat outputMarker;
    cv::Ptr<cv::aruco::Dictionary> markerDictionary
        = cv::aruco::getPredefinedDictionary(
            cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    for (int i = 0; i < 50; i++) {
        cv::aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
        std::ostringstream convert;
        std::string imageName = "4x4Marker_";
        convert << imageName << i << ".jpg";
        cv::imwrite(convert.str(), outputMarker);
    }
}

void createKnownBoarderPosition(const cv::Size& boardSize,
    float squareEdgeLength, std::vector<cv::Point3f>& corners)
{
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            corners.emplace_back(cv::Point3f((float)j * squareEdgeLength,
                (float)i * squareEdgeLength, 0.0f));
        }
    }
}

void findChessboardCorners(std::vector<cv::Mat>& images,
    std::vector<std::vector<cv::Point2f>>& allFoundCorners,
    bool showResults = false)
{
    for (auto& image : images) {
        std::vector<cv::Point2f> pointBuf;
        bool found = cv::findChessboardCorners(image, cv::Size(9, 6), pointBuf,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            allFoundCorners.emplace_back(pointBuf);
        }

        if (showResults) {
            cv::drawChessboardCorners(image, cv::Size(9, 6), pointBuf, found);
            cv::imshow("Looking for Corners", image);
            cv::waitKey(0);
        }
    }
}

void calibrate(std::vector<cv::Mat> calibrationImages,
    const cv::Size& boardSize, float squareEdgeLength, cv::Mat& cameraMatrix,
    cv::Mat& distanceCoefficients)
{
    std::vector<std::vector<cv::Point2f>> checkerboardImageSpacePoints;
    findChessboardCorners(
        calibrationImages, checkerboardImageSpacePoints, false);

    std::vector<std::vector<cv::Point3f>> worldSpaceCornersPoints(1);

    createKnownBoarderPosition(
        boardSize, squareEdgeLength, worldSpaceCornersPoints[0]);
    worldSpaceCornersPoints.resize(
        checkerboardImageSpacePoints.size(), worldSpaceCornersPoints[0]);

    std::vector<cv::Mat> rVectors, tVectors;
    distanceCoefficients = cv::Mat::zeros(8, 1, CV_64F);

    cv::calibrateCamera(worldSpaceCornersPoints, checkerboardImageSpacePoints,
        boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
}

bool exportCalibration(
    const std::string& name, cv::Mat cameraMatrix, cv::Mat distanceCoefficients)
{
    std::ofstream outStream(name);
    if (outStream) {

        uint16_t rows = cameraMatrix.rows;
        uint16_t columns = cameraMatrix.cols;

        outStream << rows << std::endl;
        outStream << columns << std::endl;

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < columns; c++) {
                double value = cameraMatrix.at<double>(r, c);
                outStream << value << std::endl;
            }
        }

        rows = distanceCoefficients.rows;
        columns = distanceCoefficients.cols;

        outStream << rows << std::endl;
        outStream << columns << std::endl;

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < columns; c++) {
                double value = distanceCoefficients.at<double>(r, c);
                outStream << value << std::endl;
            }
        }
        outStream.close();
        return true;
    }
    return false;
}

bool importCalibration(const std::string& name, cv::Mat& cameraMatrix,
    cv::Mat& distanceCoefficients)
{
    std::ifstream inStream(name);
    if (inStream) {

        uint16_t rows;
        uint16_t columns;

        inStream >> rows;
        inStream >> columns;
        cameraMatrix = cv::Mat(cv::Size(columns, rows), CV_64F);

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < columns; c++) {
                double read = 0.0f;
                inStream >> read;
                cameraMatrix.at<double>(r, c) = read;
                std::cout << cameraMatrix.at<double>(r, c) << "\n";
            }
        }
        inStream >> rows;
        inStream >> columns;
        distanceCoefficients = cv::Mat(cv::Size(columns, rows), CV_64F);

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < columns; c++) {
                double read = 0.0f;
                inStream >> read;
                distanceCoefficients.at<double>(r, c) = read;
                std::cout << cameraMatrix.at<double>(r, c) << "\n";
            }
        }
        inStream.close();
        return true;
    }
    return false;
}

int findArucoMarkers(const cv::Mat& cameraMatrix,
    const cv::Mat& distanceCoefficients, float arucoSquareDimensions)
{
    cv::Mat frame;
    cv::aruco::DetectorParameters parameters;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCorners;

    cv::Ptr<cv::aruco::Dictionary> markerDictionary
        = cv::aruco::getPredefinedDictionary(
            cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    /** create named window */
    cv::namedWindow("kinect", cv::WINDOW_AUTOSIZE);

    /** calibration R and t */
    std::vector<cv::Vec3d> rotationVectors, translationVectors;

    /** initialize kinect */
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);
    int rgbWidth = k4a_image_get_width_pixels(sptr_kinect->m_rgbImage);
    int rgbHeight = k4a_image_get_height_pixels(sptr_kinect->m_rgbImage);

    while (true) {
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

        cv::cvtColor(frame, frame, cv::COLOR_BGRA2RGB);

        cv::aruco::detectMarkers(
            frame, markerDictionary, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners,
            arucoSquareDimension, cameraMatrix, distanceCoefficients,
            rotationVectors, translationVectors);

        /** draw axis on the detected aruco markers */
        for (int i = 0; i < markerIds.size(); i++) {
            cv::aruco::drawAxis(frame, cameraMatrix, distanceCoefficients,
                rotationVectors[i], translationVectors[i], 0.1f);
        }
        cv::imshow("kinect", frame);
        if (cv::waitKey(30) >= 0)
            break;
    }
    return 1;
}

void startChessBoardCalibration(
    cv::Mat& cameraMatrix, cv::Mat distanceCoefficients)
{
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

    while (true) {
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
                std::cout << "current number of images: " << savedImages.size()
                          << std::endl;
            }
            break;
        case 27:
            /** calibrate */
            if (savedImages.size() > 15) {
                calibrate(savedImages, chessboardDimensions,
                    calibrationSquareDimension, cameraMatrix,
                    distanceCoefficients);
                exportCalibration(
                    "calibration.txt", cameraMatrix, distanceCoefficients);
                std::exit(0);
            }
        default:
            break;
        }
    }
}

int main(int argc, char* argv[])
{
    cv::Mat distanceCoefficients;
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

    startChessBoardCalibration(cameraMatrix, distanceCoefficients);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    importCalibration("calibration.txt", cameraMatrix, distanceCoefficients);
    findArucoMarkers(cameraMatrix, distanceCoefficients, arucoSquareDimension);

    return 0;
}
