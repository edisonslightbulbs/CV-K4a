#include "scene.h"
#include <opencv2/opencv.hpp>

#include "kinect.h"

void saturate(const cv::Mat& src, cv::Mat& dst)
{
    int beta = 100;     // brightness | range 1 - 100
    double alpha = 3.0; // contrast | range 1.0 - 3.0]

    dst = cv::Mat::zeros(src.size(), src.type());
    for (int y = 0; y < src.rows; y++) {
        for (int x = 0; x < src.cols; x++) {
            for (int c = 0; c < src.channels(); c++) {
                dst.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(
                    alpha * src.at<cv::Vec3b>(y, x)[c] + beta);
            }
        }
    }
}

cv::Rect segment(cv::Mat& background, cv::Mat& foreground)
{

    // subtract images and contrast resulting image
    cv::Mat diff, contrast;
    diff = background - foreground;
    saturate(diff, contrast);

    // split saturate image
    cv::Mat bgr[3];
    cv::split(contrast, bgr);
    // cv::equalizeHist(src, dst); // one other good approach to contrasting

    // threshold blue channel
    cv::Mat thresh;
    cv::threshold(bgr[0], thresh, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

    // clean using morphological operations
    cv::Mat shape, proposal;
    shape = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(thresh, proposal, cv::MORPH_OPEN, shape);

    // de-noise
    cv::Mat blur, dst;
    cv::Size dBlur = cv::Size(35, 35);
    cv::GaussianBlur(proposal, blur, dBlur, 0);

    // threshold to extract flux
    cv::threshold(blur, dst, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

#define show 0
#if show == 1
    cv::imshow("1: Background subtraction", diff);
    cv::imshow("2: Contrast", contrast);
    cv::imshow("3: Blue channel", bgr[0]);
    cv::imshow("4: Binary inverted threshold", thresh);
    cv::imshow("5: Proposal", proposal);
    cv::imshow("6: Final segment", dst);
#endif

#define write 0
#if write == 1
    cv::imwrite("./output/diff.png", diff);
    cv::imwrite("./output/contrast.png", contrast);
    cv::imwrite("./output/bluechannel.png", bgr[0]);
    cv::imwrite("./output/thresh.png", thresh);
    cv::imwrite("./output/proposal.png", proposal);
    cv::imwrite("./output/segment.png", dst);

#endif

    cv::waitKey();
    return cv::boundingRect(dst);
}

int main()
{
    // initialize kinect and scene container
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);
    std::vector<cv::Mat> scene;

    // setup window
    int w = 1366;
    int h = 768;
    const std::string window = "Area of projection";
    scene::alternateDisplayColor(sptr_kinect, window, w, h, scene);

    // // query roi for re-projection
    // cv::Rect roiBoundary = segment(scene[0], scene[1]);
    // cv::Mat roi = scene[0](roiBoundary);
    return 0;
}
