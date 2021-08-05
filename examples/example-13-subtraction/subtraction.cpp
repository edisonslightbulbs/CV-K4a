#include <opencv2/opencv.hpp>

#include "kinect.h"

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

int main()
{
    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // create full screen window
    const std::string WINDOW = "TRACELESS";
    const int w = 1366;
    const int h = 768;

    cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);
    cv::setWindowProperty(WINDOW, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    // tabletop scene
    std::vector<cv::Mat> scene;

    bool contrast = false;
    while (true) {
        cv::Mat img = contrastBackground(contrast, w, h);
        cv::imshow(WINDOW, img);
        cv::moveWindow(WINDOW, 3000, 0);

        // toggle contrast flag every second
        if (cv::waitKey(2000) >= 0) {
            break;
        }
        contrast = !contrast;

        // grab current contrast
        cv::Mat frame = grabFrame(sptr_kinect);
        scene.emplace_back(frame);
        if(scene.size() == 2){
            cv::waitKey(1000);
            break;
        }
    }

    // background subtraction
    // find projection boundary
    // save boundary
    // use boundary with kinect images
    // start calibration

    return 0;
}
