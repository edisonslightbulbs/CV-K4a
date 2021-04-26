#ifndef AXIS_H
#define AXIS_H

#include "point.h"

namespace axis {

/**
 * sortX
 *   Sorts a vector of points based on x-axis values.
 *
 * @param points
 *   Vector of 3D points.
 *
 * @retval
 *    Return a x-sorted vector of points.
 */
void sortX(std::vector<Point>& points);

/**
 * sortY
 *   Sorts a vector of points based on y-axis values.
 *
 * @param points
 *   Vector of 3D points.
 *
 * @retval
 *    Return a y-sorted vector of points.
 */
void sortY(std::vector<Point>& points);

/**
 * sortZ
 *   Sorts a vector of points based on z-axis values.
 *
 * @param points
 *   Vector of 3D points.
 *
 * @retval
 *    Return a z-sorted vector of points.
 */
void sortZ(std::vector<Point>& points);

}
#endif /* AXIS_H */
