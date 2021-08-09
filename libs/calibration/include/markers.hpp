#ifndef MARKERS_H
#define MARKERS_H

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

namespace markers {
void create4x4Markers()
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
}
#endif // MARKERS_H
