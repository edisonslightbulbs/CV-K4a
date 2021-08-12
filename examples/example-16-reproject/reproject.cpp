#include "scene.h"

int main()
{
    std::vector<cv::Mat> sceneImages;

    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    int w = 1366;
    int h = 768;
    const std::string window = "re-projection window";
    scene::alternateDisplayColor(sptr_kinect, window, w, h, sceneImages);

    // find area of projection (aop)
    cv::Rect boundary
        = scene::findProjectionArea(sceneImages[1], sceneImages[0]);
    cv::Mat background = sceneImages[1];

    // scene::undistort(background);
    cv::Mat roi = background(boundary);

    // reproject aop
    cv::Mat R, t;
    // scene::predistort(background);
    scene::project(window, w, h, roi, R, t);
    // cv::imshow("Region of interest", roi);

    return 0;
}
