#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "chessboard.h"
#include "kinect.h"
#include "parameters.h"
#include "usage.h"

class Camera {

private:
public:
    void calibrate(std::vector<cv::Mat> images, const cv::Size& boardSize,
                   float blockLength)
    {
        // todo we find world space points in a different way for projector calibration
        // chessboard::findWorldSpaceCorners( boardSize, blockLength, m_worldSpaceCorners[0]);

        m_worldSpaceCorners.resize( m_imageSpaceCorners.size(), m_worldSpaceCorners[0]);

        cv::calibrateCamera(m_worldSpaceCorners, m_imageSpaceCorners, boardSize,
                            m_matrix, m_k, m_rVectors, m_tVectors);
    }

    cv::Mat m_k;
    cv::Mat m_matrix;
    std::vector<cv::Mat> m_rVectors;
    std::vector<cv::Mat> m_tVectors;

    std::vector<cv::Point2f> m_cameraSpaceCorners;
    //std::vector<std::vector<cv::Point2f>> m_imageSpaceCorners;
    std::vector<cv::Point2f> m_imageSpaceCorners;
    std::vector<std::vector<cv::Point3f>> m_worldSpaceCorners;

    Camera()
    {
        m_k = cv::Mat::zeros(8, 1, CV_64F);
        m_matrix = cv::Mat::eye(3, 3, CV_64F);
        m_worldSpaceCorners = std::vector<std::vector<cv::Point3f>>(1);
    };
};
#endif // CAMERA_H

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

int calibrateCamera()
{
    // create camera object
    Camera camera;

    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // initialize chessboard window and chessboard images
    const std::string CHESSBOARD_WINDOW = "virtual chessboard";
    cv::namedWindow(CHESSBOARD_WINDOW, cv::WINDOW_NORMAL);
    cv::setWindowProperty( CHESSBOARD_WINDOW, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::Size boardSize = cv::Size(9, 6);
    cv::Size imgSize = cv::Size(1080, 720);
    cv::Mat chessboard = chessboard::create(imgSize, boardSize, camera.m_imageSpaceCorners);
    cv::imshow(CHESSBOARD_WINDOW, chessboard);
    cv::moveWindow(CHESSBOARD_WINDOW, 3000, 0);

    // initialize calibration window and calibration images
    const std::string CALIBRATION_WINDOW = "calibration window";
    cv::namedWindow(CALIBRATION_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::Mat frame, frameCopy;

    // specify chessboard dimensions
    const cv::Size chessboardDim = cv::Size(9, 6);

    // show usage
    usage::prompt(USAGE);

    // find corners in camera space images
    std::vector<cv::Mat> chessboardImages;
    bool found;

    // start calibration process
    bool done = false;
    while (!done) {
        frame = grabFrame(sptr_kinect);

        // find corners in camera camera space
        found = cv::findChessboardCorners(frame, chessboardDim,
                                          camera.m_cameraSpaceCorners,
                                          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        // copy camera's chessboard image and draw on found corners
        frame.copyTo(frameCopy);
        cv::drawChessboardCorners(
                frameCopy, chessboardDim, camera.m_cameraSpaceCorners, found);

        // ... if corners found
        if (found) {
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
                if (found) {
                    cv::Mat temp;
                    frame.copyTo(temp);
                    chessboardImages.emplace_back(temp);
                    std::cout << "-- # images : " << chessboardImages.size()
                              << std::endl;
                }
                break;

                // on escape keypress: exit calibration application
            case ESCAPE_KEY:
                if (chessboardImages.size() > 15) {
                    usage::prompt(COMPUTING_CALIBRATION_PARAMETERS);
                    camera.calibrate(chessboardImages, chessboardDim,
                                     chessboard::PROJECTED_BOARD_BLOCK_WIDTH);
                    usage::prompt(WRITING_CALIBRATION_PARAMETERS);
                    parameters::write(
                            "calibration.txt", camera.m_matrix, camera.m_k);
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    done = true;
                } else {
                    usage::prompt(MORE_CHESSBOARD_IMAGES_REQUIRED);
                }
            default:
                break;
        }
    }
}
