#ifndef IO_H
#define IO_H

#include "point.h"

#include <k4a/k4a.h>
#include <vector>

class io {

public:
    /**
     * io::pwd
     *   Queries the present working directory.
     *
     * @retval
     *   Returns the present working directory.
     */
    static std::string pwd();

    /**
     * io::sortLog
     *   Sort output performance log.
     */
    static void sortLog();

    /**
     * io::write
     *   Output k4a_image_t as an *.png file.
     *
     * @param k4a_image_t
     *   RGB image from kinect.
     */
    static void write(const k4a_image_t& t_rgb);

    /**
     * io::write
     *   Writes any given list of values for graphing.
     *
     * @param y
     *   A vector list of (float) y values.
     *
     * @param file
     *   Absolute path to file with point cloud.
     */
    static void write(std::vector<float>& y, const std::string& file);

    /**
     * io::head
     *   Quick-show the head (leading points) in a point cloud.
     *
     * @param points
     *   A vector list of (Point) 3D points.
     *
     * @param count
     *   Number of leading points to show.
     */
    static void head(const std::vector<Point>& points, const int& count);

    /**
     * io::tail
     *   Quick-show the tail (last points) in a point cloud.
     *
     * @param points
     *   A vector list of (Point) 3D points.
     *
     * @param count
     *   Number of last points to show.
     */
    static void tail(const std::vector<Point>& points, const int& count);

    /**
     * io::read
     *   Reads in a space-delimited point cloud file.
     *
     * @param points
     *   Empty point-cloud container.
     *
     * @param file
     *   Absolute path to file with point cloud.
     *
     * @retval
     *   Returns parsed 3D point cloud.
     */
    static std::vector<Point> read(std::vector<Point> points, const char* file);

    /**
     * io::performance
     *   Output performance log of pipeline operations.
     *
     * @retval void
     */
    static void performance(const float& rawData, const float& filteredData,
        const std::string& filterTime, const float& coarseSeg,
        const std::string& coarseSegTime, const float& finalSeg,
        const std::string& finalSegTime, const std::string& totalRuntime);
};

#endif /* IO_H */
