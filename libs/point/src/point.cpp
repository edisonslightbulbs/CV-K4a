#include "point.h"

extern const int NOISE = -2;
extern const int UNLABELED = -1;

extern const int R = 3;
extern const int xCol = 0;
extern const int yCol = 1;
extern const int zCol = 2;

bool compare(const Point& point, const Point& other)
{
    return point.m_distance.second < other.m_distance.second;
}

void Point::sort(std::vector<Point>& points)
{
    std::sort(points.begin(), points.end(), compare);
}

Point::Point()
    : m_xyz({ 0.0, 0.0, 0.0 })
    , m_cluster(UNLABELED)
    , m_distance(nullptr, __DBL_MAX__)
{
    m_clusterColor = " 0 0 0";
    m_rgb = { 0, 0, 0 };
}

Point::Point(float x, float y, float z)
    : m_xyz({ x, y, z })

    , m_cluster(UNLABELED)
    , m_distance(nullptr, __DBL_MAX__)
{
    m_clusterColor = " 0 0 0";
    m_rgb = { 0, 0, 0 };
}

void Point::setColor(const std::vector<float>& rgb)
{
    for (int i = 0; i < rgb.size(); i++) {
        m_rgb[i] = rgb[i];
    }
}

float Point::distance(const Point& other) const
{
    float x = m_xyz[0] - other.m_xyz[0];
    float y = m_xyz[1] - other.m_xyz[1];
    float z = m_xyz[2] - other.m_xyz[2];
    return (float)std::sqrt((x * x) + (y * y) + (z * z));
}

// TODO: quick test
Point Point::centroid(std::vector<Point>& points)
{
    float xSum = 0;
    float ySum = 0;
    float zSum = 0;
    for (const auto& point : points) {
        xSum = xSum + point.m_xyz[0];
        ySum = ySum + point.m_xyz[1];
        zSum = zSum + point.m_xyz[2];
    }
    return Point { xSum / points.size(), ySum / points.size(),
        zSum / points.size() };
}

bool Point::unlabeled() const
{
    return (m_cluster == UNLABELED || m_cluster == NOISE);
}

bool Point::operator==(const Point& rhs) const
{
    return (m_xyz[0] == rhs.m_xyz[0] && m_xyz[1] == rhs.m_xyz[1]
        && m_xyz[2] == rhs.m_xyz[2]);
}

bool Point::operator!=(const Point& rhs) const
{
    return (m_xyz[0] != rhs.m_xyz[0] || m_xyz[1] != rhs.m_xyz[1]
        || m_xyz[2] != rhs.m_xyz[2]);
}

bool Point::operator<(const Point& rhs) const
{
    return (this->m_distance.second < rhs.m_distance.second);
}

std::ostream& operator<<(std::ostream& stream, const Point& point)
{
    stream << point.m_xyz[0] << " " << point.m_xyz[1] << " " << point.m_xyz[2];
    return stream;
}

std::istream& operator>>(std::istream& stream, Point& point)
{
    stream >> point.m_xyz[0] >> point.m_xyz[1] >> point.m_xyz[2];
    return stream;
}
