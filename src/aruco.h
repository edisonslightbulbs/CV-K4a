#include <opencv2/aruco.hpp>

namespace aruco {

    const float arucoSquareDimension = 0.0565f;

    int find(const cv::Mat& cameraMatrix,
             const cv::Mat& distanceCoefficients, float arucoSquareDimensions)
    {
        cv::Mat frame;
        cv::aruco::DetectorParameters parameters;

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCorners;

        cv::Ptr<cv::aruco::Dictionary> markerDictionary
                = cv::aruco::getPredefinedDictionary(
                        cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

        /** create named window */
        cv::namedWindow("kinect", cv::WINDOW_AUTOSIZE);

        /** calibration R and t */
        std::vector<cv::Vec3d> rotationVectors, translationVectors;

        /** initialize kinect */
        std::shared_ptr<Kinect> sptr_kinect(new Kinect);
        int rgbWidth = k4a_image_get_width_pixels(sptr_kinect->m_rgbImage);
        int rgbHeight = k4a_image_get_height_pixels(sptr_kinect->m_rgbImage);

        std::cout << "-- place aruco marker/s in front of camera" << std::endl;
        while (true) {
            /** get next frame from kinect */
            sptr_kinect->getFrame(RGB_TO_DEPTH);

            /** get image from kinect */
            uint8_t* color_image_data
                    = k4a_image_get_buffer(sptr_kinect->m_rgbImage);

            /** release resources */
            sptr_kinect->release();

            /** cast to cv::Mat */
            frame = cv::Mat(rgbHeight, rgbWidth, CV_8UC4, (void*)color_image_data,
                            cv::Mat::AUTO_STEP);

            cv::cvtColor(frame, frame, cv::COLOR_BGRA2RGB);

            cv::aruco::detectMarkers(
                    frame, markerDictionary, markerCorners, markerIds);
            cv::aruco::estimatePoseSingleMarkers(markerCorners,
                                                 arucoSquareDimension, cameraMatrix, distanceCoefficients,
                                                 rotationVectors, translationVectors);

            /** draw axis on the detected aruco markers */
            for (int i = 0; i < markerIds.size(); i++) {
                cv::aruco::drawAxis(frame, cameraMatrix, distanceCoefficients,
                                    rotationVectors[i], translationVectors[i], 0.1f);
            }
            cv::imshow("kinect", frame);
            if (cv::waitKey(30) >= 0)
                break;
        }
        return 1;
    }
    void create4x4Markers() {
        cv::Mat outputMarker;
        cv::Ptr<cv::aruco::Dictionary> markerDictionary
                = cv::aruco::getPredefinedDictionary(
                        cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
        for (int i = 0; i < 50; i++) {
            cv::aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
            std::ostringstream convert;
            std::string imageName = "4x4Marker_";
            convert << imageName << i << ".jpg";
            cv::imwrite(convert.str(), outputMarker);
        }
    }
}
