#include "kinect.h"
#include "io.h"
//#include "ply.h"

extern const int RGB_TO_DEPTH = 1;
extern const int DEPTH_TO_RGB = 2;

void Kinect::capture()
{
    /** initiate capture sequence */
    switch (k4a_device_get_capture(m_device, &m_capture, m_timeout)) {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        throw std::runtime_error("Capture timed out!");
    case K4A_WAIT_RESULT_FAILED:
        throw std::runtime_error("Failed to capture!");
    }

    /** capture colour image */
    m_rgbImage = k4a_capture_get_color_image(m_capture);
    if (m_rgbImage == nullptr) {
        throw std::runtime_error("Failed to get color image!");
    }

    /** capture depth image */
    m_depthImage = k4a_capture_get_depth_image(m_capture);
    if (m_depthImage == nullptr) {
        throw std::runtime_error("Failed to get depth image!");
    }

    int depthWidth = k4a_image_get_width_pixels(m_depthImage);
    int depthHeight = k4a_image_get_height_pixels(m_depthImage);

    /** initialize rgb image using depth image dimensions */
    m_rgb2depthImage = nullptr;
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, depthWidth,
            depthHeight, depthWidth * 4 * (int)sizeof(uint8_t),
            &m_rgb2depthImage)) {
        throw std::runtime_error(
            "Failed to initialize rgb image using depth image dimensions!");
    }

    /** initialize pcl image using depth image dimensions */
    m_pclImage = nullptr;
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, depthWidth, depthHeight,
            depthWidth * 3 * (int)sizeof(int16_t), &m_pclImage)) {
        throw std::runtime_error("Failed to initialize point cloud image using "
                                 "depth image dimensions!");
    }
}

void Kinect::transform(const int& transformType)
{
    /** DEPTH -> POINT CLOUD */
    if (K4A_RESULT_SUCCEEDED
        != k4a_transformation_depth_image_to_point_cloud(m_transform,
            m_depthImage, K4A_CALIBRATION_TYPE_DEPTH, m_pclImage)) {
        throw std::runtime_error(
            "Failed to transform depth image to point cloud image!");
    }

    switch (transformType) {

    case RGB_TO_DEPTH: {
        if (K4A_RESULT_SUCCEEDED
            != k4a_transformation_color_image_to_depth_camera(
                m_transform, m_depthImage, m_rgbImage, m_rgb2depthImage)) {
            throw std::runtime_error(
                "Failed to create rgb2depth point cloud image!");
        }
        /** dev option: */
        // m_file = path + "/output/rgb2depth.ply";
        // ply::write(m_pclImage, m_rgb2depthImage, m_file);
        break;
    }

    case DEPTH_TO_RGB: {
        int colorWidth = k4a_image_get_width_pixels(m_rgbImage);
        int colorHeight = k4a_image_get_height_pixels(m_rgbImage);

        /** initialize depth2rgb image using rgb image dimensions */
        m_depth2rgbImage = nullptr;
        if (K4A_RESULT_SUCCEEDED
            != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, colorWidth,
                colorHeight, colorWidth * (int)sizeof(uint16_t),
                &m_depth2rgbImage)) {
            throw std::runtime_error(
                "Failed to initialize depth2rgb point cloud image!");
        }
        /** re-initialize pcl image using rgb image dimensions */
        m_pclImage = nullptr;
        if (K4A_RESULT_SUCCEEDED
            != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, colorWidth,
                colorHeight, colorWidth * 3 * (int)sizeof(int16_t),
                &m_pclImage)) {
            throw std::runtime_error("Failed to initialize point cloud image "
                                     "using rgb image dimensions!");
        }
        /** transform: depth -> rgb */
        if (K4A_RESULT_SUCCEEDED
            != k4a_transformation_depth_image_to_color_camera(
                m_transform, m_depthImage, m_depth2rgbImage)) {
            throw std::runtime_error(
                "Failed to create depth2rgb point cloud image!");
        }
        /** transform: depth -> point cloud */
        if (K4A_RESULT_SUCCEEDED
            != k4a_transformation_depth_image_to_point_cloud(m_transform,
                m_depth2rgbImage, K4A_CALIBRATION_TYPE_COLOR, m_pclImage)) {
            throw std::runtime_error("Failed to compute point cloud!");
        }
        /** dev option: */
        // m_file = path + "/output/depth2rgb.ply";
        // ply::write(m_pclImage, m_depth2rgbImage, m_file);
        break;
    }
    default: {
        break;
    }
    }
}

k4a_image_t Kinect::getPclImage()
{
    std::lock_guard<std::mutex> lck(m_mutex);
    return m_pclImage;
}

k4a_image_t Kinect::getRgb2DepthImage()
{
    std::lock_guard<std::mutex> lck(m_mutex);
    return m_rgb2depthImage;
}

void Kinect::getFrame(const int& transformType)
{
    /** block threads from accessing
     *  resources during recording */
    std::lock_guard<std::mutex> lck(m_mutex);
    capture();
    transform(transformType);
}

void Kinect::release() const
{
    if (m_capture != nullptr) {
        k4a_capture_release(m_capture);
    }
    if (m_rgbImage != nullptr) {
        k4a_image_release(m_rgbImage);
    }
    if (m_depthImage != nullptr) {
        k4a_image_release(m_depthImage);
    }
    if (m_pclImage != nullptr) {
        k4a_image_release(m_pclImage);
    }
    if (m_rgb2depthImage != nullptr) {
        k4a_image_release(m_rgb2depthImage);
    }
    if (m_depth2rgbImage != nullptr) {
        k4a_image_release(m_depth2rgbImage);
    }
}

void Kinect::close() const
{
    // TODO: destroying this nonchalant yields undesired results
    // if (m_transform != nullptr) {
    //     k4a_transformation_destroy(m_transform);
    // }
    if (m_device != nullptr) {
        k4a_device_close(m_device);
    }
}

Kinect::~Kinect() { close(); }

Kinect::Kinect()
{
    /** setup kinect  */
    t_config deviceConf;
    m_device = deviceConf.m_device;
    m_timeout = deviceConf.TIMEOUT;

    /** check for kinect */
    uint32_t deviceCount = k4a_device_get_installed_count();
    if (deviceCount == 0) {
        throw std::runtime_error("No device found!");
    }
    /** open kinect */
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_open(K4A_DEVICE_DEFAULT, &m_device)) {
        throw std::runtime_error("Unable to open device!");
    }
    /** calibrate */
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_get_calibration(m_device, deviceConf.m_config.depth_mode,
            deviceConf.m_config.color_resolution, &m_calibration)) {
        throw std::runtime_error("Unable to calibrate!");
    }
    /** start cameras */
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_start_cameras(m_device, &deviceConf.m_config)) {
        throw std::runtime_error("Failed to start cameras!");
    }
    /** get transform */
    m_transform = k4a_transformation_create(&m_calibration);

    /** capture dry run */
    capture();
    //projection();
}
