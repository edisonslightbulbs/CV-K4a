#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "chessboard.h"
#include "kinect.h"
#include "parameters.h"
#include "usage.h"

cv::Mat grabFrame(std::shared_ptr<Kinect>& sptr_kinect)
{
    sptr_kinect->capture();
    sptr_kinect->imgCapture();
    uint8_t* data = k4a_image_get_buffer(sptr_kinect->m_img);
    int w = k4a_image_get_width_pixels(sptr_kinect->m_img);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_img);
    sptr_kinect->releaseK4aCapture();
    sptr_kinect->releaseK4aImages();
    return cv::Mat(h, w, CV_8UC4, (void*)data, cv::Mat::AUTO_STEP).clone();
}

#ifndef CAMERA_H
#define CAMERA_H
class Camera {
public:
    std::vector<cv::Mat> m_R;
    std::vector<cv::Mat> m_t;

    cv::Mat m_K;
    cv::Mat m_matrix;

    std::vector<cv::Mat> m_cameraSpaceCorners;
    std::vector<std::vector<cv::Point3f>> m_worldSpaceCorners;
    std::vector<std::vector<cv::Point2f>> m_imageSpaceCorners;

    /**
     * Given;
     *
     * @param images
     *   Images of chessboard captured by the camera.
     * @param boardSize
     *   Dimensions of the chessboard.
     * @param blockWidth
     *   Width of a single chessboard block
     *
     * calibrate evaluates the:
     * R (rotation) , t (translation), camera matrix, & K (distance
     * coefficients).
     */
    void calibrate(std::vector<cv::Mat> images, const cv::Size& boardSize,
        float blockWidth)
    {
        chessboard::findImageSpaceCorners(images, m_imageSpaceCorners, false);
        chessboard::findWorldSpaceCorners(
            boardSize, blockWidth, m_worldSpaceCorners[0]);

        m_worldSpaceCorners.resize(
            m_imageSpaceCorners.size(), m_worldSpaceCorners[0]);

        cv::calibrateCamera(m_worldSpaceCorners, m_imageSpaceCorners, boardSize,
            m_matrix, m_K, m_R, m_t);
    }

    Camera()
    {
        m_K = cv::Mat::zeros(8, 1, CV_64F);
        m_matrix = cv::Mat::eye(3, 3, CV_64F);
        m_worldSpaceCorners = std::vector<std::vector<cv::Point3f>>(1);
    };
};
#endif // CAMERA_H

int main()
{
    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    Camera camera;
    // setup camera matrix and initialize empty calibration parameters
    // -- dep cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    // -- dep cv::Mat coefficients;

    // initialize calibration window chessboard image frames
    const std::string CALIBRATION_WINDOW = "calibration window";
    cv::namedWindow(CALIBRATION_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::Mat frame, frameCopy;

    // define chessboard dimensions
    const cv::Size chessboardDim = cv::Size(9, 6);

    // show usage
    usage::prompt(USAGE);

    // find corners in camera space images
    bool cornersFound;
    // std::vector<cv::Mat> cameraImages;

    // start calibration process
    bool calibrationDone = false;
    while (!calibrationDone) {
        frame = grabFrame(sptr_kinect);

        // find corners in camera space
        std::vector<cv::Point2f> cameraSpaceCorners;
        cornersFound = cv::findChessboardCorners(frame, chessboardDim,
            cameraSpaceCorners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        // copy camera's chessboard image and draw on found corners
        frame.copyTo(frameCopy);
        cv::drawChessboardCorners(
            frameCopy, chessboardDim, cameraSpaceCorners, cornersFound);

        // ... if corners found
        if (cornersFound) {
            // ... show chessboard image with highlighted corners
            cv::imshow(CALIBRATION_WINDOW, frameCopy);
        } else {
            // ... show non-highlighted chessboard image
            cv::imshow(CALIBRATION_WINDOW, frame);
        }

        // get user input
        int key = cv::waitKey(30);
        switch (key) {

        // on enter keypress: get camera image
        case ENTER_KEY:
            if (cornersFound) {
                cv::Mat temp;
                frame.copyTo(temp);
                camera.m_cameraSpaceCorners.emplace_back(temp);
                std::cout << "-- # images : "
                          << camera.m_cameraSpaceCorners.size() << std::endl;
            }
            break;

            // on escape keypress: exit calibration application
        case ESCAPE_KEY:
            if (camera.m_cameraSpaceCorners.size() > 15) {
                usage::prompt(COMPUTING_CALIBRATION_PARAMETERS);
                camera.calibrate(camera.m_cameraSpaceCorners, chessboardDim,
                    chessboard::PHYSICAL_BOARD_BLOCK_WIDTH);
                usage::prompt(WRITING_CALIBRATION_PARAMETERS);
                parameters::write("./output/calibration/camera.txt",
                    camera.m_matrix, camera.m_K);
                std::this_thread::sleep_for(std::chrono::seconds(5));
                calibrationDone = true;
            } else {
                usage::prompt(MORE_CHESSBOARD_IMAGES_REQUIRED);
            }
        default:
            break;
        }
    }
    return 0;
}
