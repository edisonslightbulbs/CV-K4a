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

cv::Rect segment(const cv::Mat& src1, const cv::Mat& src2)
{
    cv::Mat background, foreground;
    foreground = src1;
    background = src2;

    // subtract images and contrast resulting image
    cv::Mat diff, contrast;
    diff = background - foreground;
    saturate(diff, contrast);
    // todo: undistort

    // split high contrast image
    cv::Mat rgb[3];
    cv::Mat bgr;
    cv::cvtColor(contrast, bgr,  cv::COLOR_BGR2RGB);

    cv::split(bgr, rgb);
    // cv::equalizeHist(src, dst); // one other good approach to contrasting

    // threshold blue channel
    cv::Mat thresh;
    cv::threshold(rgb[2], thresh, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

    // clean using morphological operations
    cv::Mat shape, proposal;
    shape = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(thresh, proposal, cv::MORPH_OPEN, shape);

    // de-noise
    cv::Mat blur, secThresh;
    cv::Size dBlur = cv::Size(75, 75);
    cv::GaussianBlur(proposal, blur, dBlur, 0);

    // threshold denoised frame
    cv::threshold(blur, secThresh, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

    // flood fill
    cv::Mat floodFill = secThresh.clone();
    cv::floodFill(floodFill, cv::Point(0, 0), cv::Scalar(255));

    // invert flood fill
    cv::Mat floodFillInv;
    cv::bitwise_not(floodFill, floodFillInv);

    // combine threshold and flood fill inverse
    cv::Mat roi = (secThresh | floodFillInv);

#define show 0
#if show == 1
    cv::imshow("1: Background subtraction", diff);
    cv::imshow("2: Contrast", contrast);
    cv::imshow("3.1: Red channel", rgb[0]);
    cv::imshow("3.2: Green channel", rgb[1]);
    cv::imshow("3.3: Blue channel", rgb[2]);
    cv::imshow("4: Binary inverted threshold", thresh);
    cv::imshow("5.1: Morphological cleaning", proposal);
    cv::imshow("5.2: Morphological cleaning", blur);
    cv::imshow("5.3: Morphological cleaning", secThresh);
    cv::imshow("6.1: Flood-fill", floodFill);
    cv::imshow("6.2: Flood-fill inverse", floodFillInv);
    cv::imshow("7: ROI", roi);
    cv::waitKey();
#endif

#define write 1
#if write == 1
    cv::imwrite("./output/background.png", src1);
    cv::imwrite("./output/foreground.png", src2);
    cv::imwrite("./output/01___diff.png", diff);
    cv::imwrite("./output/02___contrast.png", contrast);
    cv::imwrite("./output/03___redchannel.png", rgb[0]);
    cv::imwrite("./output/04___greenchannel.png", rgb[1]);
    cv::imwrite("./output/05___bluechannel.png", rgb[2]);
    cv::imwrite("./output/06___thresh.png", thresh);
    cv::imwrite("./output/07___proposal.png", proposal);
    cv::imwrite("./output/08___blur.png", blur);
    cv::imwrite("./output/09___secthresh.png", secThresh);
    cv::imwrite("./output/10___floodfill.png", floodFill);
    cv::imwrite("./output/11___floodfillinverse.png", floodFillInv);
    cv::imwrite("./output/12___segment.png", roi);
#endif
    return cv::boundingRect(roi);
}

cv::Mat blackBackground(const cv::Mat& background, const cv::Mat& foreground, const cv::Rect& boundary){

    // rotate foreground clockwise by 90 degrees
    cv::Mat foregroundRotated;
    cv::rotate(foreground, foregroundRotated, cv::ROTATE_90_CLOCKWISE);

    // create black background rotated 90 col,row assignment swapped
    cv::Mat blackMask(background.cols, background.rows, CV_8UC4, cv::Scalar(0, 0, 0, 0));

    // make sure images are in the same format
    cv::cvtColor(foregroundRotated, foregroundRotated, cv::COLOR_BGR2BGRA);
    cv::cvtColor(blackMask, blackMask, cv::COLOR_BGR2BGRA);

    // specify starting position, and
    // width & height of foreground image
    int xMin = boundary.x/2;
    int yMin = boundary.y;
    int width = foregroundRotated.cols;
    int height = foregroundRotated.rows;

    // find roi on background image and segment it
    cv::Rect roi = cv::Rect(xMin, yMin, width, height);
    cv::Mat segment = blackMask(roi);

    // overlay segment with foreground
    foregroundRotated.copyTo(segment);

    return blackMask;
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

    // query roi for re-projection
    cv::Rect boundary = segment(scene[0], scene[1]);
    cv::Mat roi = scene[0](boundary);
    //cv::imshow("ROI", roi);
    cv::imwrite("./output/roi.png", roi);

    // black background (as opposed to cropping it)
    cv::Mat roiBlackBackground = blackBackground(scene[0], roi, boundary);
    cv::imshow("test", roiBlackBackground);
    cv::imwrite("./output/roiBlacked.png", roiBlackBackground);
    cv::waitKey();
    return 0;
}
