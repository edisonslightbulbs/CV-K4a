#include <opencv2/opencv.hpp>

#include "camera.h"

void Camera::findWorldSpaceCorners(const cv::Size& boardSize, float blockWidth)
{
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            m_worldSpaceCorners[0].emplace_back(cv::Point3f(
                (float)j * blockWidth, (float)i * blockWidth, 0.0f));
        }
    }
}

void Camera::findCameraSpaceCorners()
{
    cv::Mat gray;
    cv::Size windowSize = cv::Size(5, 5);
    cv::Size zeroZone = cv::Size(-1, -1);
    cv::TermCriteria criteria = cv::TermCriteria(
        cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);

    for (auto& image : m_images) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> chessboardCorners;
        bool found
            = cv::findChessboardCorners(gray, cv::Size(9, 6), chessboardCorners,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        // refine corner locations
        cv::cornerSubPix(
            gray, chessboardCorners, windowSize, zeroZone, criteria);

        // overlay chessboard image
        if (found) {
            m_cameraSpaceCorners.emplace_back(chessboardCorners);
        }
    }
}

void Camera::calibrate(const cv::Size& boardSize, float blockWidth)
{
    findCameraSpaceCorners();
    findWorldSpaceCorners(boardSize, blockWidth);
    m_worldSpaceCorners.resize(
        m_cameraSpaceCorners.size(), m_worldSpaceCorners[0]);
    cv::calibrateCamera(m_worldSpaceCorners, m_cameraSpaceCorners, boardSize,
        m_K, m_distortionCoefficients, m_R, m_t);
}

Camera::Camera()
{
    m_distortionCoefficients = cv::Mat::zeros(8, 1, CV_64F);
    m_K = cv::Mat::eye(3, 3, CV_64F);
    m_worldSpaceCorners = std::vector<std::vector<cv::Point3f>>(1);
}
