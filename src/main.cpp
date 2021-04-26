#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>

#include "kinect.h"

const int fps = 20;
int main(int argc, char* argv[])
{
    /** initialize kinect */
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);
    int rgbWidth = k4a_image_get_width_pixels(sptr_kinect->m_rgbImage);
    int rgbHeight = k4a_image_get_height_pixels(sptr_kinect->m_rgbImage);


    while (true){
        /** get next frame from kinect */
        sptr_kinect->getFrame(RGB_TO_DEPTH);

        /** get image from kinect */
        uint8_t* color_image_data = k4a_image_get_buffer(sptr_kinect->m_rgbImage);

        /** release resources */
        sptr_kinect->release();

        /** cast to cv::Mat */
        cv::Mat frame = cv::Mat(rgbHeight, rgbWidth, CV_8UC4, (void*)color_image_data, cv::Mat::AUTO_STEP);

        cv::imshow("kinect", frame);
        if(cv::waitKey(1000/fps) >=0){
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return 0;
}
