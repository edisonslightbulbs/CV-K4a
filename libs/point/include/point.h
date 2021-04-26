#ifndef POINT_H
#define POINT_H

#include <array>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

extern const int R;    // <- dimensional space
extern const int xCol; // x column
extern const int yCol; // y column
extern const int zCol; // z column

extern const int NOISE;
extern const int UNLABELED;

class Point {

public:
    int m_cluster;                       // <- point cluster
    std::string m_clusterColor;          // <- cluster color (from clustering)
    std::array<int, 3> m_rgb {};         // <- point color (from rgb image)
    std::array<float, 3> m_xyz {};       // <- point coordinates
    std::pair<Point*, float> m_distance; // <- Euclidean distance to Point*

    /** 3D point constructors */
    Point();
    Point(float x, float y, float z);

    /**
     * unlabeled
     *   Checks if 'this' point is classified as noise or has unlabelled
     * cluster.
     */
    [[nodiscard]] bool unlabeled() const;

    /**
     * sort
     *   Sorts a given set of points using m_distance as a criteria.
     *
     *  @param points
     *    Set of points with distance measures to a common/shared point.
     */
    static void sort(std::vector<Point>& points);

    /**
     * setColor
     *   Set the color corresponding color for each xyz point.
     *
     *  @param rgb
     *    Color.
     */
    void setColor(const std::vector<float>& rgb);

    /**
     * centroid
     *   Computes the centroid for a given set of points.
     *
     *  @param points
     *    Points of interest.
     *
     *  @retval
     *    Centroid.
     */
    static Point centroid(std::vector<Point>& points);

    /**
     * distance
     *   Computes distance of *this Point to another point.
     *
     *  @param other
     *    Other Point.
     *
     *  @retval
     *    Euclidean distance to other point.
     */
    [[nodiscard]] float distance(const Point& other) const;

    /** operator overrides */
    bool operator<(const Point& rhs) const;
    bool operator!=(const Point& rhs) const;
    bool operator==(const Point& rhs) const;
    friend std::ostream& operator<<(std::ostream& t_stream, const Point& point);
    friend std::istream& operator>>(std::istream& t_stream, Point& point);
};
#endif /* POINT_H */
