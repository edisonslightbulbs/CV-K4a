#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>

#include "chessboard.h"
#include "file.h"
#include "kinect.h"
#include "usage.h"

class Camera {
private:
    void findCameraSpaceCorners();
    void findWorldSpaceCorners(const cv::Size& boardSize, float blockWidth);

public:
    std::vector<cv::Mat> m_R;
    std::vector<cv::Mat> m_t;
    cv::Mat m_K; // <- camera matrix
    cv::Mat m_distortionCoefficients;

    std::vector<cv::Mat> m_images;
    std::vector<std::vector<cv::Point3f>> m_worldSpaceCorners;
    std::vector<std::vector<cv::Point2f>> m_cameraSpaceCorners;

    /**
     * Evaluates the R (rotation) , t (translation), camera matrix,
     * & K (distance coefficients).
     *
     * @param images
     *   Images of chessboard captured by the camera.
     * @param boardSize
     *   Dimensions of the chessboard.
     * @param blockWidth
     *   Width of a single chessboard block
     */
    void calibrate(const cv::Size& boardSize, float blockWidth);

    Camera();
};
#endif // CAMERA_H
