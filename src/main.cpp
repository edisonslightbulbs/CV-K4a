#include "scene.h"
#include <iostream>
#include <opencv2/opencv.hpp>

int main(){
    // initialize camera
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);
    cv::Mat frame = scene::grabFrame(sptr_kinect);

    std::cout << "hello world" << std::endl;
    cv::imshow("my window", frame);
    cv::waitKey();
}