#include <opencv2/aruco.hpp>
#include  "kinect.h"

namespace aruco {

    const float arucoSquareDimension = 0.0565f;

    void create4x4Markers() {
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
}
