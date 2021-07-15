#include <opencv2/opencv.hpp>

namespace chessboard {

const float calibrationSquareDimension = 0.02500f;

void findSquareCorners(std::vector<cv::Mat>& images,
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

void findSquareEdges(const cv::Size& boardSize, float squareEdgeLength,
    std::vector<cv::Point3f>& corners)
{
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            corners.emplace_back(cv::Point3f((float)j * squareEdgeLength,
                (float)i * squareEdgeLength, 0.0f));
        }
    }
}
}
