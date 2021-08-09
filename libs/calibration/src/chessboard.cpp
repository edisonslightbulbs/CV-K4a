#include "chessboard.h"

void chessboard::checkAspectRatio(const int& width, const int& height)
{
    std::setprecision(4);
    double ratio = (double)width / height;
    double expected = (double)16 / 9;

    if (ratio != expected) {
        std::cerr << "-- please set chessboard image to a 16:9 aspect ratio";
        exit(1);
    }
}

cv::Mat chessboard::create(const cv::Size& imgSize, const cv::Size& boardSize,
    std::vector<cv::Point2f>& imgSpaceCorners)
{
    int offset = 50;
    unsigned char color = 255;
    // checkAspectRatio(imgSize.width, imgSize.height); // todo: test function

    cv::Mat board(imgSize, CV_8UC3, cv::Scalar::all(255));
    int blockWidth
        = floor((imgSize.height - 2 * offset) / (boardSize.height + 1));
    int maxWidth = (blockWidth * (boardSize.width + 1)) + (2 * offset);

    std::cout << "--      image height: " << imgSize.height << std::endl;
    std::cout << "--      image  width: " << imgSize.width << std::endl;
    std::cout << "--     aspect ration: 16:9 " << std::endl;
    std::cout << "-- chessboard height: " << boardSize.height << std::endl;
    std::cout << "--  chessboard width: " << boardSize.width << std::endl;

    for (int y = offset; y < imgSize.height - offset; y = y + blockWidth) {
        if (y + blockWidth > imgSize.height - offset) {
            break;
        }
        for (int x = offset; x < imgSize.width - offset; x = x + blockWidth) {
            color = ~color;
            if (x + blockWidth > maxWidth) {
                break;
            }
            // collect image space points
            if (x > offset && y > offset) {
                imgSpaceCorners.emplace_back(x, y);
            }
            cv::Mat block = board(cv::Rect(x, y, blockWidth, blockWidth));
            block.setTo(cv::Scalar::all(color));
        }
    }
    return board;
}

void chessboard::project(const cv::Size& dChessboard)
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

void chessboard::capture(
    const bool& pass, t_pCloudFrame& data, std::vector<t_pCloudFrame>& pCloud)
{
    if (pass) {
        pCloud.emplace_back(data);
        int imgCount = (int)pCloud.size();
        std::cout << "-- # of RGBD images: " << imgCount << std::endl;
    }
}

void chessboard::capture(
    const bool& pass, cv::Mat& image, std::vector<cv::Mat>& chessboardImages)
{
    if (pass) {
        chessboardImages.emplace_back(image);
        int imgCount = (int)chessboardImages.size();
        std::cout << "-- # of RGB images: " << imgCount << std::endl;
    }
}

bool chessboard::overlay(const cv::Mat& src, cv::Mat dst,
    const cv::Size& dChessboard, const std::string& window)
{
    std::vector<cv::Point2f> chessboardCorners;
    bool found = cv::findChessboardCorners(src, dChessboard, chessboardCorners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    if (found) {
        src.copyTo(dst);
        cv::drawChessboardCorners(dst, dChessboard, chessboardCorners, found);
        cv::imshow(window, dst);
    } else {
        cv::imshow(window, src);
    }
    return found;
}
