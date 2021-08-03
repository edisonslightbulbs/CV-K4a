#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "chessboard.h"
#include "kinect.h"
#include "parameters.h"
#include "usage.h"
#include "camera.h"

void calibrate(std::vector<cv::Mat> images, const cv::Size& boardSize,
               float blockLength, cv::Mat& cameraMatrix, cv::Mat& k)
{
    std::vector<cv::Mat> rVectors, tVectors;
    std::vector<std::vector<cv::Point2f>> imageSpaceCorners;
    std::vector<std::vector<cv::Point3f>> worldSpaceSquareCorners(1);

    chessboard::findImageSpaceCorners(images, imageSpaceCorners, false);
    chessboard::findWorldSpaceCorners(
            boardSize, blockLength, worldSpaceSquareCorners[0]);

    worldSpaceSquareCorners.resize(
            imageSpaceCorners.size(), worldSpaceSquareCorners[0]);
    k = cv::Mat::zeros(8, 1, CV_64F);

    // cv::calibrateCamera()
    // returns (i) camera matrix, (ii) distortion coefficients, (iii) rotation
    // matrix & (iv) translation matrix.
    cv::calibrateCamera(worldSpaceSquareCorners, imageSpaceCorners, boardSize,
                        cameraMatrix, k, rVectors, tVectors);
}


int main()
{
    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // setup camera matrix and calibration parameters
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat coefficients;

    // initialize chessboard window and chessboard images
    const std::string CHESSBOARD_WINDOW = "chessboard";
    cv::namedWindow(CHESSBOARD_WINDOW, cv::WINDOW_NORMAL);
    cv::setWindowProperty(
            CHESSBOARD_WINDOW, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    std::vector<cv::Point2f> imageSpaceCorners;
    cv::Size imgSize = cv::Size(1080, 720);
    cv::Size boardSize = cv::Size(9, 6);
    cv::Mat chessboard
            = chessboard::create(imgSize, boardSize, imageSpaceCorners);

    // project chessboard
    cv::imshow(CHESSBOARD_WINDOW, chessboard);
    cv::moveWindow(CHESSBOARD_WINDOW, 3000, 0);
    // cv::waitKey(30000);

    // initialize calibration window and calibration images
    const std::string CALIBRATION_WINDOW = "calibration";
    cv::namedWindow(CALIBRATION_WINDOW, cv::WINDOW_AUTOSIZE);
    cv::Mat frame, frameCopy;

    // specify chessboard dimensions
    const cv::Size chessboardDim = cv::Size(9, 6);

    // show usage
    usage::prompt(USAGE);

    // find corners in camera space images
    bool found;
    std::vector<cv::Mat> cameraImages;

    // start calibration process
    bool done = false;
    while (!done) {
        frame = grabFrame(sptr_kinect);

        // find corners in camera camera space
        std::vector<cv::Point2f> cameraSpaceCorners;
        found = cv::findChessboardCorners(frame, chessboardDim,
                                          cameraSpaceCorners,
                                          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        // copy camera's chessboard image and draw on found corners
        frame.copyTo(frameCopy);
        cv::drawChessboardCorners(
                frameCopy, chessboardDim, cameraSpaceCorners, found);

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
                    cameraImages.emplace_back(temp);
                    std::cout << "-- # images : " << cameraImages.size()
                              << std::endl;
                }
                break;

                // on escape keypress: exit calibration application
            case ESCAPE_KEY:
                if (cameraImages.size() > 15) {
                    usage::prompt(COMPUTING_CALIBRATION_PARAMETERS);
                    calibrate(cameraImages, chessboardDim,
                              chessboard::PROJECTED_BOARD_BLOCK_WIDTH, cameraMatrix,
                              coefficients);
                    usage::prompt(WRITING_CALIBRATION_PARAMETERS);
                    parameters::write(
                            "calibration.txt", cameraMatrix, coefficients);
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    done = true;
                } else {
                    usage::prompt(MORE_CHESSBOARD_IMAGES_REQUIRED);
                }
            default:
                break;
        }
    }
    return 0;
}
