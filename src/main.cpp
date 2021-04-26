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

int main(int argc, char* argv[])
{
    createArucoMarkers();
    return 0;
}
