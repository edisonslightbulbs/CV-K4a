#ifndef PROJECTOR_H
#define PROJECTOR_H

#include "chessboard.h"
#include "pcloud.h"
#include "point.h"

#include "svd.h" //<-- include svd library
#include <Eigen/Dense>
#include <vector>

using t_pCloudFrame = std::pair<cv::Mat, std::vector<Point>>;

class Projector {
public:
    cv::Mat m_K;
    std::vector<cv::Mat> m_R;
    std::vector<cv::Mat> m_t;
    cv::Mat m_distortionCoefficients;

    std::vector<t_pCloudFrame> m_RGBDCollection;
    std::vector<std::vector<cv::Point3f>> m_worldSpaceCorners;
    std::vector<std::vector<cv::Point2f>> m_cameraSpaceCorners;

    void findWorldSpaceCorners(const cv::Size& boardSize, float blockWidth)
    {
        for (int i = 0; i < boardSize.height; i++) {
            for (int j = 0; j < boardSize.width; j++) {
                m_worldSpaceCorners[0].emplace_back(cv::Point3f(
                    (float)j * blockWidth, (float)i * blockWidth, 0.0f));
            }
        }
    }

    void computeSVD(const std::vector<Point>& pCloud)
    {
        // set svd computation flag
        int flag = Eigen::ComputeThinU | Eigen::ComputeThinV;

        // compute SVD
        SVD usv(pCloud, flag);
    }

    void findCameraSpaceCorners()
    {
        cv::Mat gray;
        cv::Size windowSize = cv::Size(5, 5);
        cv::Size zeroZone = cv::Size(-1, -1);
        cv::TermCriteria criteria = cv::TermCriteria(
            cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);

        for (auto& rgbd : m_RGBDCollection) {
            cv::cvtColor(rgbd.first, gray, cv::COLOR_BGR2GRAY);

            std::vector<cv::Point2f> chessboardCorners;
            bool found = cv::findChessboardCorners(gray, cv::Size(9, 6),
                chessboardCorners,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

            // refine corner locations
            cv::cornerSubPix(
                gray, chessboardCorners, windowSize, zeroZone, criteria);

            // overlay chessboard image
            if (found) {
                m_cameraSpaceCorners.emplace_back(chessboardCorners);
            }
        }
        // todo: actually, at this point lets retrieve the RGB-D values and
        //   start plotting and building world space coordinates using 3D
        //   point clouds -
        //   1. plot points
        //   2. transform points
        //   3. test
    }

    void calibrate(const cv::Size& boardSize, float blockWidth)
    {
        findCameraSpaceCorners();
        findWorldSpaceCorners(
            boardSize, blockWidth); // todo: in principle this is incorrect!
        // todo: implement svd based solution
        m_worldSpaceCorners.resize(
            m_cameraSpaceCorners.size(), m_worldSpaceCorners[0]);
        cv::calibrateCamera(m_worldSpaceCorners, m_cameraSpaceCorners,
            boardSize, m_K, m_distortionCoefficients, m_R, m_t);
    }

    Projector()
    {
        m_distortionCoefficients = cv::Mat::zeros(8, 1, CV_64F);
        m_K = cv::Mat::eye(3, 3, CV_64F);
        m_worldSpaceCorners = std::vector<std::vector<cv::Point3f>>(1);
    }
};
#endif // PROJECTOR_H
