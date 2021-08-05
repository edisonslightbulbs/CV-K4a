#include <opencv2/opencv.hpp>

#include "file.h"
#include "kinect.h"
#include "usage.h"

#ifndef PROJECTOR_H
#define PROJECTOR_H

#include "chessboard.h"
#include "pcloud.h"

using t_RGBD = std::pair<cv::Mat, std::vector<t_rgbd>>;

class Projector {
public:
    cv::Mat m_K;
    std::vector<cv::Mat> m_R;
    std::vector<cv::Mat> m_t;
    cv::Mat m_distortionCoefficients;

    std::vector<t_RGBD> m_RGBDCollection;
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
    }

    void calibrate(const cv::Size& boardSize, float blockWidth)
    {
        findCameraSpaceCorners();
        findWorldSpaceCorners(boardSize, blockWidth);
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
bool calibrateCamera(
        Projector& projector, const cv::Size& dChessboard, const std::string& file)
{
    bool done = false;
    if (projector.m_RGBDCollection.size() > 15) {
        usage::prompt(CALIBRATING);
        projector.calibrate(dChessboard, chessboard::R_BLOCK_WIDTH);
        usage::prompt(SAVING_PARAMETERS);
        parameters::write(
                file, projector.m_K, projector.m_distortionCoefficients);
        done = true;
    } else {
        usage::prompt(MORE_IMAGES_REQUIRED);
    }
    return done;
}

t_RGBD grabFrame(std::shared_ptr<Kinect>& sptr_kinect)
{
    sptr_kinect->capture();
    sptr_kinect->depthCapture();
    sptr_kinect->pclCapture();
    sptr_kinect->imgCapture();
    sptr_kinect->c2dCapture();
    sptr_kinect->transform(RGB_TO_DEPTH);

    // get depth image dimensions
    int w = k4a_image_get_width_pixels(sptr_kinect->m_depth);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_depth);

    // get synchronous RGB-D captures
    auto* pCloudData
            = (int16_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_pcl);
    auto* rgbData = k4a_image_get_buffer(sptr_kinect->m_c2d);

    const std::string file = "./output/pcloud/3dPCloud.ply";
    std::vector<t_rgbd> rgbd = pcloud::retrieve(w, h, pCloudData, rgbData);
    // pcloud::write(rgbdPCloud, file);
    cv::Mat frame
            = cv::Mat(h, w, CV_8UC4, (void*)rgbData, cv::Mat::AUTO_STEP).clone();

    sptr_kinect->releaseK4aCapture();
    sptr_kinect->releaseK4aImages();

    t_RGBD data = std::make_pair(frame, rgbd);

    // couple frame and point cloud
    return data;
}

void projectChessboard(const cv::Size& dChessboard)
{
    const std::string CHESSBOARD_WINDOW = "VIRTUAL CHESSBOARD";
    cv::namedWindow(CHESSBOARD_WINDOW, cv::WINDOW_NORMAL);
    cv::setWindowProperty(
            CHESSBOARD_WINDOW, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    std::vector<cv::Point2f> imageSpaceCorners;
    cv::Size imgSize = cv::Size(1080, 720);
    cv::Mat chessboard
            = chessboard::create(imgSize, dChessboard, imageSpaceCorners);

    cv::imshow(CHESSBOARD_WINDOW, chessboard);
    cv::moveWindow(CHESSBOARD_WINDOW, 3000, 0);
}

int main()
{
    // initialize image frames and  kinect
    cv::Mat src, dst;
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // create projector
    Projector projector;

    // project chessboard
    cv::Size dChessboard = cv::Size(9, 6);
    projectChessboard(dChessboard);

    // setup calibration window
    std::string window = "calibration window";
    cv::namedWindow(window, cv::WINDOW_AUTOSIZE);

    // calibrate projector
    bool done = false;
    usage::prompt(USAGE);
    std::string file = "./output/calibration/projector.txt";

    while (!done) {
        t_RGBD rgbdData = grabFrame(sptr_kinect);

        src = rgbdData.first;
        bool pass = chessboard::overlay(src, dst, dChessboard, window);

        int key = cv::waitKey(30);
        switch (key) {
            case ENTER_KEY: // capture images
                chessboard::capture(pass, rgbdData, projector.m_RGBDCollection);
                break;
            case ESCAPE_KEY: // start calibration
                done = calibrateCamera(projector, dChessboard, file);
            default:
                break;
        }
    }
    return 0;
}
