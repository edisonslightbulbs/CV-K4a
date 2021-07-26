//-- // #include <iostream>
//-- //
//-- // int main()
//-- // {
//-- //     std::cout << "-- see in examples directory for the code " <<
//std::endl;
//-- //     std::cout << "-- see in build/bin directory for the binaries " <<
//-- //     std::endl;
//-- // }
//--
//-- #include <chrono>
//-- #include <opencv2/opencv.hpp>
//-- #include <thread>
//--
//-- #include "kinect.h"
//-- #include "chessboard.h"
//--
//-- #define WIDTH 1366
//-- #define HEIGHT 768
//-- #define WINDOW "TRACELESS"
//-- #define SETUP_WINDOW cv::moveWindow(WINDOW, 3000, 0)
//--
//-- cv::Mat grabFrame(std::shared_ptr<Kinect>& sptr_kinect)
//-- {
//--     sptr_kinect->capture();
//--     sptr_kinect->imgCapture();
//--     uint8_t* data = k4a_image_get_buffer(sptr_kinect->m_img);
//--     int w = k4a_image_get_width_pixels(sptr_kinect->m_img);
//--     int h = k4a_image_get_height_pixels(sptr_kinect->m_img);
//--     sptr_kinect->releaseK4aCapture();
//--     sptr_kinect->releaseK4aImages();
//--     return cv::Mat(h, w, CV_8UC4, (void*)data, cv::Mat::AUTO_STEP).clone();
//-- }
//--
//-- cv::Mat contrastReference(const bool& contrast)
//-- {
//--     // create black and white images
//--     cv::Mat black(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
//--     cv::Mat white(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
//--
//--     if (contrast) {
//--         return white;
//--     } else {
//--         return black;
//--     }
//-- }
//--
//-- void captureReference(std::vector<cv::Mat>& images,
//std::shared_ptr<Kinect>& sptr_kinect){
//--     // background contrast flag
//--     bool contrast = false;
//--
//--     while (true) {
//--         cv::Mat img = contrastReference(contrast);
//--         cv::imshow(WINDOW, img);
//--         SETUP_WINDOW;
//--
//--         // toggle contrast flag every second
//--         if (cv::waitKey(1000) >= 0) {
//--             break;
//--         }
//--         contrast = !contrast;
//--
//--         // grab current contrast
//--         cv::Mat frame = grabFrame(sptr_kinect);
//--         images.emplace_back(frame);
//--         if (images.size() == 2) {
//--             break;
//--         }
//--     }
//-- }
//--
//-- void projectChessboard(const cv::Mat& chessboard){
//--     while (true) {
//--         cv::imshow(WINDOW, chessboard);
//--         SETUP_WINDOW;
//--
//--         // toggle contrast flag every second
//--         if (cv::waitKey(1000) >= 0) {
//--             break;
//--         }
//--     }
//-- }
//--
//-- int main()
//-- {
//--     // initialize kinect
//--     std::shared_ptr<Kinect> sptr_kinect(new Kinect);
//--
//--     // create full screen window
//--     cv::namedWindow(WINDOW, cv::WINDOW_NORMAL);
//--     cv::setWindowProperty( WINDOW, cv::WND_PROP_FULLSCREEN,
//cv::WINDOW_FULLSCREEN);
//--
//--     // generate chessboard
//--     std::vector<cv::Point2f> corners;
//--     cv::Size imgSize = cv::Size(800, 600);
//--     cv::Size boardSize = cv::Size(9, 6);
//--     cv::Mat chessboard = chessboard::generate(imgSize, boardSize, corners);
//--
//--     // tabletop images
//--     std::vector<cv::Mat> kinectImages;
//--
//--     // todo implement: reference scene function
//--
//--     std::thread chessboardWorker(projectChessboard, chessboard);
//--
//--     chessboardWorker.join();
//--     return 0;
//-- }
//--
