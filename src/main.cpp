#include <opencv2/core.hpp>
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

const float calibrationSquareDimension = 0.01905f; // meters
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
    return 0;
}
