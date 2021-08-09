#ifndef CHESSBOARD_H
#define CHESSBOARD_H

#include "pcloud.h"
#include "point.h"

#include <opencv2/opencv.hpp>

using t_pCloudFrame = std::pair<cv::Mat, std::vector<Point>>;

namespace chessboard {

const float R_BLOCK_WIDTH = 0.02600f;
const float P_BLOCK_WIDTH = 0.06050f;

cv::Mat create(const cv::Size& imgSize, const cv::Size& boardSize,
    std::vector<cv::Point2f>& imgSpaceCorners);

void capture(
    const bool& pass, cv::Mat& image, std::vector<cv::Mat>& chessboardImages);

bool overlay(const cv::Mat& src, cv::Mat dst, const cv::Size& dChessboard,
    const std::string& window);

void capture(
    const bool& pass, t_pCloudFrame& data, std::vector<t_pCloudFrame>& pCloud);

void project(const cv::Size& dChessboard);

void checkAspectRatio(const int& width, const int& height);
}

#endif // CHESSBOARD_H
