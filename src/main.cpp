#include <opencv2/opencv.hpp>
#include "kinect.h"

#define MOG2_MODEL cv::createBackgroundSubtractorMOG2()
#define KNN_MODEL cv::createBackgroundSubtractorKNN()

void writeScene(std::vector<cv::Mat> scene){
    const std::string file_0 = "./output/scene/black.png";
    const std::string file_1 = "./output/scene/white.png";
    cv::imwrite(file_0, scene[0]);
    cv::imwrite(file_1, scene[1]);
}

cv::Mat contrastBackground(const bool& contrast, const int& w, const int& h){
    // create black and white images
    cv::Mat black(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat white(h, w, CV_8UC3, cv::Scalar(255, 255, 255));

    if (contrast){
        return white;
    } else {
        return black;
    }
}

cv::Mat grabFrame(std::shared_ptr<Kinect>& sptr_kinect)
{
    sptr_kinect->capture();
    sptr_kinect->depthCapture();
    sptr_kinect->pclCapture();
    sptr_kinect->imgCapture();
    sptr_kinect->c2dCapture();
    sptr_kinect->transform(RGB_TO_DEPTH);

    auto* rgbData = k4a_image_get_buffer(sptr_kinect->m_c2d);
    int w = k4a_image_get_width_pixels(sptr_kinect->m_c2d);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_c2d);

    cv::Mat frame
            = cv::Mat(h, w, CV_8UC4, (void*)rgbData, cv::Mat::AUTO_STEP).clone();

    sptr_kinect->releaseK4aCapture();
    sptr_kinect->releaseK4aImages();

    return frame;
}

void flicker(std::shared_ptr<Kinect>& sptr_kinect, const std::string& window, const int& w, const int& h, std::vector<cv::Mat>& scene){
    bool contrast = false;
    while (true) {
        // show black | white images for a second
        cv::Mat img = contrastBackground(contrast, w, h);
        cv::imshow(window, img);
        cv::moveWindow(window, 3000, 0);
        if (cv::waitKey(2000) >= 0) {
            break;
        }

        // capture scene
        cv::Mat frame = grabFrame(sptr_kinect);
        scene.emplace_back(frame);
        contrast = !contrast;

        if(scene.size() == 2){
            cv::waitKey(1000);
            break;
        }
    }
}

int main()
{
    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // create full screen window for scene
    const int w = 1366;
    const int h = 768;
    std::vector<cv::Mat> scene;
    std::string window = "TRACELESS";
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    cv::setWindowProperty(window, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    // flicker projector
    flicker(sptr_kinect, window, w, h, scene);

    writeScene(scene);

    // create background subtractor: use either MOG2 or KNN
    cv::Ptr<cv::BackgroundSubtractor> subtractor;
    subtractor = MOG2_MODEL;
    cv::Mat frame, gray, mask, diff, thresh;

    //do background subtraction: using models
   // frame = scene[1];
   // mask = scene[0];
    // subtractor->apply(frame, mask);
    //show the current frame and the fg masks
    // imshow("Frame", frame);
    // imshow("Mask", mask);

    //do background subtraction: using absolute difference
    cv::absdiff(scene[0], scene[1], diff);
    cv::imshow("Absolute difference", diff);

    // cropping region of interest using
    cv::cvtColor(diff, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, thresh, 0, 255, cv::THRESH_OTSU);
    cv::imshow("Threshold", thresh);

    cv::Rect roiBoundary = cv::boundingRect(thresh);

    cv::Mat roi = scene[0](roiBoundary);
    cv::imshow("Cropped", roi);

    cv::waitKey();

    return 0;
}
