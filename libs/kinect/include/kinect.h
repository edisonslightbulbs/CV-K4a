#ifndef KINECT_H
#define KINECT_H

#include <atomic>
#include <cfloat>
#include <k4a/k4a.h>
#include <mutex>
#include <shared_mutex>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "point.h"

/**
 * @struct DEVICE_CONF
 *    Single container for all kinect config
 *
 * @b Reference
 *    https://docs.microsoft.com/en-us/azure/kinect-dk/hardware-specification#depth-camera-supported-operating-modes
 */
struct t_config {
    const int32_t TIMEOUT = 1000;
    k4a_device_t m_device = nullptr;
    k4a_device_configuration_t m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    t_config()
    {
        /**  for fast point cloud, use: */
        // m_config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
        // m_config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;

        /** option B: for real-time rendering  */
        m_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        m_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

        m_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        m_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        m_config.synchronized_images_only = true;
    }
};

extern const int RGB_TO_DEPTH;
extern const int DEPTH_TO_RGB;

/**
 * @file kinect.h
 *    Kinect device.
 */
class Kinect {

public:
    /** thread guards */
    std::mutex m_mutex;

    /** image resolution */
    int m_numPoints = 640 * 576;

    /** device config */
    int32_t m_timeout = 0;
    k4a_device_t m_device;

    /** k4a images */
    k4a_image_t m_rgbImage = nullptr;
    k4a_image_t m_pclImage = nullptr;
    k4a_image_t m_depthImage = nullptr;
    k4a_image_t m_rgb2depthImage = nullptr;
    k4a_image_t m_depth2rgbImage = nullptr;

    /** k4a capture, calibration, and transformation */
    k4a_capture_t m_capture = nullptr;
    k4a_calibration_t m_calibration {};
    k4a_transformation_t m_transform = nullptr;

    /** output PLY file */
    // std::string m_file;

    /**
     * capture
     *   Capture depth and color images.
     */
    void capture();

    /**
     * transform
     *   Calibrates point cloud image resolution.
     *
     * @param sptr_points
     *   "Safe global" share pointer to point cloud points.
     */
    void transform(const int& transformType);

    /**
     * record
     *   Records a frame using the kinect.
     *
     *  @param transformType
     *    Specification of transformation type:
     *      option 1: RGB_TO_DEPTH
     *      option 2: DEPTH_TO_RGB
     */
    void getFrame(const int& transformType);

    /**
     * close
     *   Closes connection to kinect.
     */
    void close() const;

    /**
     * release
     *   Releases kinect resources.
     */
    void release() const;

    /**
     * Kinect
     *   Initialize kinect device.
     */
    Kinect();

    /**
     * Kinect
     *   Destroy kinect object
     */
    ~Kinect();

    k4a_image_t getRgb2DepthImage();

    k4a_image_t getPclImage();

    void projection(){
        std::vector<k4a_float3_t> points_3d = { { { 0.f, 0.f, 1000.f } },          // color camera center
                                           { { -1000.f, -1000.f, 1000.f } },  // color camera top left
                                           { { 1000.f, -1000.f, 1000.f } },   // color camera top right
                                           { { 1000.f, 1000.f, 1000.f } },    // color camera bottom right
                                           { { -1000.f, 1000.f, 1000.f } } }; // color camera bottom left

        // k4a project function
        std::vector<k4a_float2_t> k4a_points_2d(points_3d.size());
        for (size_t i = 0; i < points_3d.size(); i++)
        {
            int valid = 0;
            k4a_calibration_3d_to_2d(&m_calibration,
                                     &points_3d[i],
                                     K4A_CALIBRATION_TYPE_COLOR,
                                     K4A_CALIBRATION_TYPE_DEPTH,
                                     &k4a_points_2d[i],
                                     &valid);
        }

        // converting the calibration data to OpenCV format
        // extrinsic transformation from color to depth camera
        cv::Mat se3 =
                cv::Mat(3, 3, CV_32FC1, m_calibration.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].rotation);
        cv::Mat r_vec = cv::Mat(3, 1, CV_32FC1);
        Rodrigues(se3, r_vec);
        cv::Mat t_vec =
                cv::Mat(3, 1, CV_32F, m_calibration.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH].translation);

        // intrinsic parameters of the depth camera
        k4a_calibration_intrinsic_parameters_t *intrinsics = &m_calibration.depth_camera_calibration.intrinsics.parameters;
        std::vector<float> _camera_matrix = {
                intrinsics->param.fx, 0.f, intrinsics->param.cx, 0.f, intrinsics->param.fy, intrinsics->param.cy, 0.f, 0.f, 1.f
        };
        cv::Mat camera_matrix = cv::Mat(3, 3, CV_32F, &_camera_matrix[0]);
        std::vector<float> _dist_coeffs = { intrinsics->param.k1, intrinsics->param.k2, intrinsics->param.p1,
                                       intrinsics->param.p2, intrinsics->param.k3, intrinsics->param.k4,
                                       intrinsics->param.k5, intrinsics->param.k6 };
        cv::Mat dist_coeffs = cv::Mat(8, 1, CV_32F, &_dist_coeffs[0]);

        // OpenCV project function
        std::vector<cv::Point2f> cv_points_2d(points_3d.size());
        projectPoints(*reinterpret_cast<std::vector<cv::Point3f> *>(&points_3d),
                      r_vec,
                      t_vec,
                      camera_matrix,
                      dist_coeffs,
                      cv_points_2d);

        for (size_t i = 0; i < points_3d.size(); i++)
        {
            printf("3d point:\t\t\t(%.5f, %.5f, %.5f)\n", points_3d[i].v[0], points_3d[i].v[1], points_3d[i].v[2]);
            printf("OpenCV projectPoints:\t\t(%.5f, %.5f)\n", cv_points_2d[i].x, cv_points_2d[i].y);
            printf("k4a_calibration_3d_to_2d:\t(%.5f, %.5f)\n\n", k4a_points_2d[i].v[0], k4a_points_2d[i].v[1]);
        }

    }
};
#endif /* KINECT_H */
