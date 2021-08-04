#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "camera.h"
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

    std::vector<cv::Point2f> m_cameraSpaceCorners;
    std::vector<std::vector<cv::Point3f>> m_worldSpaceCorners;
    std::vector<std::vector<cv::Point2f>> m_imageSpaceCorners;

    void findCameraSpaceCorners(std::vector<cv::Mat>& images,
        std::vector<std::vector<cv::Point2f>>& corners)
    {
        for (auto& image : images) {
            std::vector<cv::Point2f> corner;
            bool found = cv::findChessboardCorners(image, cv::Size(9, 6),
                corner,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

            if (found) {
                corners.emplace_back(corner);
            }
        }
    }

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
    void calibrate(std::vector<cv::Mat> images, const cv::Size& boardSize,
        float blockWidth)
    {
        findCameraSpaceCorners(images, m_imageSpaceCorners);
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

bool calibrateCamera(Camera& camera, std::vector<cv::Mat>& chessboardImages,
    const cv::Size& dChessboard)
{
    bool done = false;
    if (chessboardImages.size() > 15) {
        usage::prompt(COMPUTING_CALIBRATION_PARAMETERS);
        camera.calibrate(
            chessboardImages, dChessboard, chessboard::R_BLOCK_WIDTH);
        usage::prompt(WRITING_CALIBRATION_PARAMETERS);
        parameters::write(
            "./output/calibration/camera.txt", camera.m_matrix, camera.m_K);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        done = true;
    } else {
        usage::prompt(MORE_CHESSBOARD_IMAGES_REQUIRED);
    }
    return done;
}

void getChessboardImage(
    const bool& pass, cv::Mat& src, std::vector<cv::Mat>& chessboardImages)
{
    if (pass) {
        cv::Mat temp;
        src.copyTo(temp);
        chessboardImages.emplace_back(temp);
        int imgCount = (int)chessboardImages.size();
        std::cout << "-- # images: " << imgCount << std::endl;
    }
}

bool overlayCorners(const cv::Mat& src, cv::Mat dst,
    const cv::Size& dChessboard, const std::string& window)
{
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(src, dChessboard, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    if (found) {
        src.copyTo(dst);
        cv::drawChessboardCorners(dst, dChessboard, corners, found);
        cv::imshow(window, dst);
    } else {
        cv::imshow(window, src);
    }
    return found;
}

int main()
{
    // initialize camera, window, and images
    Camera camera;
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    const std::string WINDOW = "calibration window";
    cv::namedWindow(WINDOW, cv::WINDOW_AUTOSIZE);

    cv::Mat src, dst;
    std::vector<cv::Mat> chessboardImages;
    const cv::Size dChessboard = cv::Size(9, 6);

    // start calibration
    bool done = false;
    usage::prompt(USAGE);

    while (!done) {
        src = grabFrame(sptr_kinect);
        bool pass = overlayCorners(src, dst, dChessboard, WINDOW);

        int key = cv::waitKey(30);
        switch (key) {
        case ENTER_KEY: // capture chessboard images
            getChessboardImage(pass, src, chessboardImages);
            break;
        case ESCAPE_KEY: // start calibration
            done = calibrateCamera(camera, chessboardImages, dChessboard);
        default:
            break;
        }
    }
    return 0;
}
