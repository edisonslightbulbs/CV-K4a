#ifndef KINECT_H
#define KINECT_H

#include <cfloat>
#include <k4a/k4a.h>
#include <mutex>
#include <shared_mutex>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

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

    void openCVKinectCalibration();
};
#endif /* KINECT_H */
