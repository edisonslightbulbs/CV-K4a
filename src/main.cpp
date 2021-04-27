#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <thread>
#include <chrono>
#include <sstream>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>

#include "kinect.h"

const float calibrationSquareDimension = 0.02500f; // meters
const float arucoSquareDimension = 0.1016f; // meters
const cv::Size chessboardDimensions = cv::Size(9, 6);


void createArucoMarkers(){
    cv::Mat outputMarker;
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    for (int i = 0; i < 50; i++){
        cv::aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
        std::ostringstream convert;
        std::string  imageName = "4x4Marker_";
        convert << imageName << i << ".jpg";
        cv::imwrite(convert.str(), outputMarker);
    }
}

void createKnownBoarderPosition(const cv::Size& boardSize, float squareEdgeLength, std::vector<cv::Point3f>& corners){
    for (int i = 0; i < boardSize.height; i++){
        for (int j = 0; j < boardSize.width; j++){
            corners.emplace_back(cv::Point3f((float)j * squareEdgeLength, (float)i * squareEdgeLength, 0.0f));
        }
    }
}

void getChessboardCorners (std::vector<cv::Mat>& images, std::vector<std::vector<cv::Point2f>>& allFoundCorners, bool showResults = false){
    for (auto & image : images){
        std::vector<cv::Point2f> pointBuf;
        bool found = cv::findChessboardCorners(image,cv::Size(9, 6), pointBuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found){
            allFoundCorners.emplace_back(pointBuf);
        }

        if (showResults){
            cv::drawChessboardCorners(image, cv::Size(9, 6), pointBuf, found);
            cv::imshow("Looking for Corners", image);
            cv::waitKey(0);
        }
    }

}

int main(int argc, char* argv[])
{
    cv::Mat frame;
    cv::Mat drawToFrame;
    cv::Mat distanceCoefficients;

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


        while (true){
            /** get next frame from kinect */
            sptr_kinect->getFrame(RGB_TO_DEPTH);

            /** get image from kinect */
            uint8_t* color_image_data = k4a_image_get_buffer(sptr_kinect->m_rgbImage);

            /** release resources */
            sptr_kinect->release();

            /** cast to cv::Mat */
            frame = cv::Mat(rgbHeight, rgbWidth, CV_8UC4, (void*)color_image_data, cv::Mat::AUTO_STEP);

            /** find corners */
            std::vector<cv::Point2f> foundPoints;
            bool found = false;
            found = cv::findChessboardCorners(frame,chessboardDimensions, foundPoints, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
            frame.copyTo(drawToFrame);

            /** if found,  draw them */
            cv::drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);

            if (found) {
                cv::imshow("kinect", drawToFrame);
            } else {
                cv::imshow("kinect", frame);
            }
            cv::waitKey(1000/fps);
        }
        return 0;
}
