#include <algorithm>
#include <vector>

#include "axis.h"

void axis::sortX(std::vector<Point>& points)
{
    std::sort(points.begin(), points.end(),
        [](const Point& point, const Point& other) {
            return point.m_xyz[0] < other.m_xyz[0];
        });
}

void axis::sortY(std::vector<Point>& points)
{
    std::sort(points.begin(), points.end(),
        [](const Point& point, const Point& other) {
            return point.m_xyz[1] < other.m_xyz[1];
        });
}

void axis::sortZ(std::vector<Point>& points)
{
    std::sort(points.begin(), points.end(),
        [](const Point& point, const Point& other) {
            return point.m_xyz[2] < other.m_xyz[2];
        });
}
